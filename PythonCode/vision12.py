import cv2
import serial
import time
import threading

# ── Serial ──────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 57600          # ← was 9600

# ── Camera ──────────────────────────────────────────────
FRAME_W = 640
FRAME_H = 480

# ── Detection ───────────────────────────────────────────
CONFIDENCE_TH    = 0.5
PERSON_CLASS     = 15
REVALIDATE_EVERY = 15        # frames between MobileNet checks

PROTOTXT   = '/home/piuser/robot/deploy.prototxt'
CAFFEMODEL = '/home/piuser/robot/mobilenet_iter_73000.caffemodel'

# ── Control ─────────────────────────────────────────────
DEAD_ZONE     = 0.15         # ← was 0.20 (tighter = more responsive)
SMOOTH_FRAMES = 2            # ← was 3 (less lag)
CMD_COOLDOWN  = 0.08         # ← was 0.30 (was killing responsiveness)
LOST_TIMEOUT  = 2.0

# ── PID for turn intensity ───────────────────────────────
# These replace the fixed map_range. Tune Kp first, then Kd.
Kp = 180.0   # Proportional gain  (increase = faster turns)
Kd = 30.0    # Derivative gain    (increase = damps oscillation)
MIN_SPEED = 60
MAX_SPEED = 220              # ← now matches Arduino's wider range

# ────────────────────────────────────────────────────────
net     = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # ← prevent camera buffer buildup

timestamp   = time.strftime("%Y%m%d_%H%M%S")
output_path = f"/home/piuser/robot/recording_{timestamp}.avi"
fourcc      = cv2.VideoWriter_fourcc(*'XVID')
out         = cv2.VideoWriter(output_path, fourcc, 20.0, (FRAME_W, FRAME_H))

# ── State ────────────────────────────────────────────────
last_cmd      = None
last_cmd_time = 0
tracker       = None
tracking      = False
lost_time     = None
cx_history    = []
frame_count   = 0
prev_error    = 0.0

# ── Threaded re-validation ───────────────────────────────
revalidation_result = {'box': None, 'done': False}

def run_detection_async(frame):
    """Run MobileNet in background thread so main loop never blocks."""
    revalidation_result['done'] = False
    def _detect():
        box = detect_person(frame)
        revalidation_result['box']  = box
        revalidation_result['done'] = True
    threading.Thread(target=_detect, daemon=True).start()

# ── Helpers ──────────────────────────────────────────────
def send_command(cmd_char, speed=0):
    """Send 2-byte binary packet: [CMD, SPEED]"""
    global last_cmd, last_cmd_time
    now = time.time()
    key = (cmd_char, speed)
    if key != last_cmd or (now - last_cmd_time) > CMD_COOLDOWN:
        arduino.write(bytes([ord(cmd_char), speed]))
        last_cmd      = key
        last_cmd_time = now

def create_tracker():
    return cv2.TrackerCSRT_create()

def smooth_cx(new_cx):
    cx_history.append(new_cx)
    if len(cx_history) > SMOOTH_FRAMES:
        cx_history.pop(0)
    return sum(cx_history) / len(cx_history)

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

def pid_speed(error, prev_err):
    """Simple PD controller → turn speed (no integral needed here)."""
    p   = Kp * abs(error)
    d   = Kd * abs(error - prev_err)
    spd = int(p + d)
    return max(MIN_SPEED, min(MAX_SPEED, spd))

# ── Main loop ────────────────────────────────────────────
try:
    revalidation_pending = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed"); break

        frame = cv2.flip(frame, 1)
        out.write(frame)

        if not tracking:
            cv2.putText(frame, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            box = detect_person(frame)
            if box is not None:
                tracker = create_tracker()
                tracker.init(frame, box)
                tracking    = True
                lost_time   = None
                frame_count = 0
                prev_error  = 0.0
                cx_history.clear()
                revalidation_pending = False
                print("Locked onto person")
            send_command('S', 0)

        else:
            success, box = tracker.update(frame)
            frame_count += 1

            # ── Async re-validation ──────────────────────
            if frame_count % REVALIDATE_EVERY == 0 and not revalidation_pending:
                run_detection_async(frame.copy())
                revalidation_pending = True

            if revalidation_pending and revalidation_result['done']:
                revalidation_pending = False
                if revalidation_result['box'] is None:
                    print("Re-validation failed — re-scanning")
                    tracking = False; tracker = None
                    lost_time = None; cx_history.clear(); prev_error = 0.0
                    send_command('S', 0)
                    continue

            if success:
                lost_time = None
                x, y, w, h = [int(v) for v in box]
                cx = x + w // 2
                smoothed_cx = smooth_cx(cx)
                norm_cx     = (smoothed_cx / FRAME_W) - 0.5   # –0.5 to +0.5

                # ── PD control ──────────────────────────
                error = norm_cx
                spd   = pid_speed(error, prev_error)
                prev_error = error

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(frame, f"err:{error:.2f} spd:{spd}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

                if norm_cx < -DEAD_ZONE:
                    send_command('L', spd)
                elif norm_cx > DEAD_ZONE:
                    send_command('R', spd)
                else:
                    send_command('F', 120)   # fixed forward speed

            else:
                if lost_time is None:
                    lost_time = time.time()
                elapsed = time.time() - lost_time
                cv2.putText(frame, f"LOST {elapsed:.1f}s", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                send_command('S', 0)
                if elapsed > LOST_TIMEOUT:
                    tracking = False; tracker = None
                    lost_time = None; frame_count = 0
                    cx_history.clear(); prev_error = 0.0

        # Dead-zone visual guides
        lx = int(FRAME_W * (0.5 - DEAD_ZONE))
        rx = int(FRAME_W * (0.5 + DEAD_ZONE))
        cv2.line(frame, (lx,0),(lx,FRAME_H),(0,100,255),1)
        cv2.line(frame, (rx,0),(rx,FRAME_H),(0,100,255),1)
        cv2.imshow("Human Following Robot", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    send_command('S', 0)
    cap.release(); out.release()
    cv2.destroyAllWindows(); arduino.close()
    print(f"Recording saved: {output_path}")