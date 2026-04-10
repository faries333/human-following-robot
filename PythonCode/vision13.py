import cv2
import serial
import time
import threading

# ── Serial ────────────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 57600

# ── Camera ────────────────────────────────────────────────────────────────────
FRAME_W = 640
FRAME_H = 480

# ── Detection ─────────────────────────────────────────────────────────────────
CONFIDENCE_TH    = 0.5
PERSON_CLASS     = 15
REVALIDATE_EVERY = 15       # frames between background MobileNet checks

PROTOTXT   = '/home/piuser/robot/deploy.prototxt'
CAFFEMODEL = '/home/piuser/robot/mobilenet_iter_73000.caffemodel'

# ── Control ───────────────────────────────────────────────────────────────────
# MIRROR FIX: set to True if camera image is physically flipped left/right.
# When True the frame is flipped so the display looks natural, but the turn
# direction is automatically corrected — no manual L/R swap needed.
FLIP_CAMERA   = False        # ← set True only if your camera is mounted upside-down
                             #   or you want a mirrored preview.  Default = False.

DEAD_ZONE     = 0.15         # normalised half-width of centre dead-band (0.0–0.5)
SMOOTH_FRAMES = 2            # cx history length — reduce lag vs. jitter tradeoff
LOST_TIMEOUT  = 2.0          # seconds before tracker gives up and re-scans

# ── Turn burst control ────────────────────────────────────────────────────────
# Instead of hammering turn commands at 20 fps, we send ONE turn command then
# coast (send F/S) for BURST_COAST_S seconds.  The burst duration itself is
# scaled by error magnitude so small corrections stay small.
#
#   total turn impulse ≈ TURN_PWM × BURST_DURATION_S
#
# Tune these two knobs first:
#   BURST_SCALE   — how many milliseconds of burst per unit of normalised error
#   BURST_MAX_S   — hard cap so the robot never spins more than this at once
BURST_SCALE_S = 0.18         # seconds of turn per 1.0 of norm_cx error
BURST_MAX_S   = 0.12         # maximum single burst (seconds)
BURST_MIN_S   = 0.03         # minimum burst (avoids jitter at tiny errors)
BURST_COAST_S = 0.05         # mandatory coast gap between bursts

# ── PD gains for turn PWM speed (not duration) ───────────────────────────────
Kp        = 170.0
Kd        = 25.0
MIN_SPEED = 60
MAX_SPEED = 220

# ── Globals ───────────────────────────────────────────────────────────────────
net     = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # always read the freshest frame

timestamp   = time.strftime("%Y%m%d_%H%M%S")
output_path = f"/home/piuser/robot/recording_{timestamp}.avi"
fourcc      = cv2.VideoWriter_fourcc(*'XVID')
out         = cv2.VideoWriter(output_path, fourcc, 20.0, (FRAME_W, FRAME_H))
print(f"Recording to: {output_path}")

# ── State ─────────────────────────────────────────────────────────────────────
tracker      = None
tracking     = False
lost_time    = None
cx_history   = []
frame_count  = 0
prev_error   = 0.0

# Turn burst timing
burst_end_time  = 0.0   # time when current burst expires → switch to coast
coast_end_time  = 0.0   # time when coast gap expires → ready for next burst
in_burst        = False

# Revalidation thread state
revalidation_result  = {'box': None, 'done': True}
revalidation_pending = False

# ── Helpers ───────────────────────────────────────────────────────────────────

def send_raw(cmd_char, speed=0):
    """Send 2-byte binary packet [CMD, SPEED] directly — no cooldown gate."""
    arduino.write(bytes([ord(cmd_char), speed]))

def detect_person(frame):
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()
    best_conf, best_box = 0, None
    for i in range(detections.shape[2]):
        conf  = float(detections[0, 0, i, 2])
        label = int(detections[0, 0, i, 1])
        if label == PERSON_CLASS and conf > CONFIDENCE_TH and conf > best_conf:
            best_conf = conf
            box = detections[0, 0, i, 3:7] * [FRAME_W, FRAME_H, FRAME_W, FRAME_H]
            x1, y1, x2, y2 = box.astype(int)
            best_box = (x1, y1, x2 - x1, y2 - y1)
    return best_box

def run_detection_async(frame):
    """Kick off MobileNet in a daemon thread so the main loop never blocks."""
    revalidation_result['done'] = False
    def _detect():
        revalidation_result['box']  = detect_person(frame)
        revalidation_result['done'] = True
    threading.Thread(target=_detect, daemon=True).start()

def smooth_cx(new_cx):
    cx_history.append(new_cx)
    if len(cx_history) > SMOOTH_FRAMES:
        cx_history.pop(0)
    return sum(cx_history) / len(cx_history)

def pd_speed(error, prev_err):
    """PD controller → turn PWM speed (0–MAX_SPEED)."""
    p   = Kp * abs(error)
    d   = Kd * abs(error - prev_err)
    spd = int(p + d)
    return max(MIN_SPEED, min(MAX_SPEED, spd))

def burst_duration(error_magnitude):
    """Scale burst duration to error size, clamped to [BURST_MIN_S, BURST_MAX_S]."""
    dur = error_magnitude * BURST_SCALE_S
    return max(BURST_MIN_S, min(BURST_MAX_S, dur))

def reset_tracking_state():
    global tracking, tracker, lost_time, frame_count, prev_error
    global burst_end_time, coast_end_time, in_burst, revalidation_pending
    tracking     = False
    tracker      = None
    lost_time    = None
    frame_count  = 0
    prev_error   = 0.0
    burst_end_time  = 0.0
    coast_end_time  = 0.0
    in_burst        = False
    revalidation_pending = False
    cx_history.clear()

# ── Main loop ─────────────────────────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        # ── Mirror / flip ─────────────────────────────────────────────────────
        # FLIP_CAMERA = False  → no flip, raw image used as-is.
        #                        norm_cx > 0 means person is to the RIGHT
        #                        → robot turns RIGHT.  Correct.
        # FLIP_CAMERA = True   → flip(frame, 1) mirrors for natural display,
        #                        but we INVERT norm_cx before using it so the
        #                        turn direction is still correct.
        if FLIP_CAMERA:
            frame = cv2.flip(frame, 1)

        out.write(frame)
        now = time.time()

        # ── SCAN MODE ─────────────────────────────────────────────────────────
        if not tracking:
            cv2.putText(frame, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            box = detect_person(frame)
            if box is not None:
                tracker = cv2.TrackerCSRT_create()
                tracker.init(frame, box)
                tracking = True
                print("Locked onto person")
            send_raw('S', 0)

        # ── TRACKING MODE ─────────────────────────────────────────────────────
        else:
            success, box = tracker.update(frame)
            frame_count += 1

            # Async re-validation trigger
            if frame_count % REVALIDATE_EVERY == 0 and not revalidation_pending:
                run_detection_async(frame.copy())
                revalidation_pending = True

            # Consume re-validation result
            if revalidation_pending and revalidation_result['done']:
                revalidation_pending = False
                if revalidation_result['box'] is None:
                    print("Re-validation: no person — re-scanning")
                    reset_tracking_state()
                    send_raw('S', 0)
                    continue

            if success:
                lost_time = None
                x, y, w, h  = [int(v) for v in box]
                cx          = x + w // 2
                smoothed_cx = smooth_cx(cx)

                # norm_cx: −0.5 = far left, 0 = centre, +0.5 = far right
                raw_norm_cx = (smoothed_cx / FRAME_W) - 0.5

                # If camera is flipped, invert so turn direction is correct
                norm_cx = -raw_norm_cx if FLIP_CAMERA else raw_norm_cx

                error = norm_cx
                spd   = pd_speed(error, prev_error)
                prev_error = error

                # ── Burst turn logic ──────────────────────────────────────────
                if norm_cx < -DEAD_ZONE:
                    # Person is to the LEFT → turn LEFT
                    if not in_burst and now >= coast_end_time:
                        dur = burst_duration(abs(norm_cx))
                        burst_end_time = now + dur
                        coast_end_time = burst_end_time + BURST_COAST_S
                        in_burst = True

                    if in_burst and now < burst_end_time:
                        send_raw('L', spd)
                        cv2.putText(frame, f"TURN L  spd:{spd}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
                    else:
                        in_burst = False
                        send_raw('S', 0)    # coast — let the burst settle

                elif norm_cx > DEAD_ZONE:
                    # Person is to the RIGHT → turn RIGHT
                    if not in_burst and now >= coast_end_time:
                        dur = burst_duration(abs(norm_cx))
                        burst_end_time = now + dur
                        coast_end_time = burst_end_time + BURST_COAST_S
                        in_burst = True

                    if in_burst and now < burst_end_time:
                        send_raw('R', spd)
                        cv2.putText(frame, f"TURN R  spd:{spd}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
                    else:
                        in_burst = False
                        send_raw('S', 0)

                else:
                    # Person is centred → move forward
                    in_burst       = False
                    burst_end_time = 0.0
                    coast_end_time = 0.0
                    send_raw('F', 120)
                    cv2.putText(frame, "FORWARD", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Debug overlays
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(smoothed_cx), y + h // 2), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"cx:{norm_cx:+.2f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(frame, f"fr:{frame_count}", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

            else:
                # Tracker lost the target
                in_burst = False
                if lost_time is None:
                    lost_time = now
                    print("Person lost, waiting...")

                elapsed = now - lost_time
                cv2.putText(frame, f"LOST {elapsed:.1f}s", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                send_raw('S', 0)

                if elapsed > LOST_TIMEOUT:
                    print("Re-scanning for person...")
                    reset_tracking_state()

        # Dead-zone guide lines
        lx = int(FRAME_W * (0.5 - DEAD_ZONE))
        rx = int(FRAME_W * (0.5 + DEAD_ZONE))
        cv2.line(frame, (lx, 0), (lx, FRAME_H), (0, 100, 255), 1)
        cv2.line(frame, (rx, 0), (rx, FRAME_H), (0, 100, 255), 1)

        cv2.imshow("Human Following Robot", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    send_raw('S', 0)
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    arduino.close()
    print(f"Recording saved to: {output_path}")
    print("Stopped.")