import cv2
import serial
import time

SERIAL_PORT   = '/dev/ttyACM0'
BAUD_RATE     = 9600
CONFIDENCE_TH = 0.5
DEAD_ZONE     = 0.20
FRAME_W       = 640
FRAME_H       = 480
PERSON_CLASS  = 15
LOST_TIMEOUT  = 2.0
SMOOTH_FRAMES = 3
CMD_COOLDOWN  = 0.3
MIN_TURN      = 50
MAX_TURN      = 80

TARGET_HEIGHT    = 280   # box height to stop at
HEIGHT_MARGIN    = 40    # tolerance
REVALIDATE_EVERY = 20    # check with MobileNet every 20 frames

PROTOTXT   = '/home/piuser/robot/deploy.prototxt'
CAFFEMODEL = '/home/piuser/robot/mobilenet_iter_73000.caffemodel'

net     = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

timestamp   = time.strftime("%Y%m%d_%H%M%S")
output_path = f"/home/piuser/robot/recording_{timestamp}.avi"
fourcc      = cv2.VideoWriter_fourcc(*'XVID')
out         = cv2.VideoWriter(output_path, fourcc, 20.0, (FRAME_W, FRAME_H))
print(f"Recording to: {output_path}")

last_cmd         = None
last_cmd_time    = 0
tracker          = None
tracking         = False
lost_time        = None
cx_history       = []
frame_count      = 0
locked_cx        = None
prev_box_height  = None   # track sudden box height jumps

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(in_min, min(in_max, value))
    return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

def send_command(cmd):
    global last_cmd, last_cmd_time
    now = time.time()
    if cmd != last_cmd or (now - last_cmd_time) > CMD_COOLDOWN:
        arduino.write((cmd + '\n').encode())
        last_cmd      = cmd
        last_cmd_time = now
        print(f"CMD: {cmd}")

def create_tracker():
    return cv2.TrackerCSRT_create()

def smooth_cx(new_cx):
    cx_history.append(new_cx)
    if len(cx_history) > SMOOTH_FRAMES:
        cx_history.pop(0)
    return sum(cx_history) / len(cx_history)

def detect_person(frame):
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)),
        0.007843, (300, 300), 127.5
    )
    net.setInput(blob)
    detections = net.forward()

    best_conf = 0
    best_box  = None

    for i in range(detections.shape[2]):
        conf  = float(detections[0, 0, i, 2])
        label = int(detections[0, 0, i, 1])
        if label == PERSON_CLASS and conf > CONFIDENCE_TH and conf > best_conf:
            best_conf = conf
            box = detections[0, 0, i, 3:7] * [FRAME_W, FRAME_H, FRAME_W, FRAME_H]
            x1, y1, x2, y2 = box.astype(int)
            best_box = (x1, y1, x2 - x1, y2 - y1)

    return best_box

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        out.write(frame)

        if not tracking:
            # --- SCAN MODE ---
            cv2.putText(frame, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            box = detect_person(frame)
            if box is not None:
                tracker = create_tracker()
                tracker.init(frame, box)
                tracking        = True
                lost_time       = None
                frame_count     = 0
                prev_box_height = None
                cx_history.clear()
                x, y, w, h  = box
                locked_cx   = (x + w / 2) / FRAME_W - 0.5
                print(f"Locked onto person at cx={locked_cx:.2f}")

            send_command('S')

        else:
            # --- TRACKING MODE ---
            success, box = tracker.update(frame)

            if success:
                lost_time    = None
                frame_count += 1
                x, y, w, h  = [int(v) for v in box]
                cx           = x + w // 2
                box_height   = h

                # --- REVALIDATION: confirm person still exists ---
                if frame_count % REVALIDATE_EVERY == 0:
                    person_box = detect_person(frame)
                    if person_box is None:
                        # No person found at all — go to lost mode
                        print("Revalidation: no person found — re-scanning")
                        tracking    = False
                        tracker     = None
                        lost_time   = None
                        frame_count = 0
                        locked_cx   = None
                        cx_history.clear()
                        send_command('S')
                        continue
                    else:
                        # Person found — update locked_cx
                        px, py, pw, ph = person_box
                        locked_cx = (px + pw / 2) / FRAME_W - 0.5

                # --- BLOCKING DETECTION: sudden large box = someone walked in front ---
                person_blocked = False
                if prev_box_height is not None:
                    height_jump = box_height - prev_box_height
                    if height_jump > 80:   # box grew by 80px suddenly = someone stepped in
                        person_blocked = True
                        print(f"Block detected — height jumped by {height_jump}px")

                prev_box_height = box_height

                if person_blocked:
                    cv2.putText(frame, "BLOCKED - WAITING", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    send_command('S')

                else:
                    smoothed_cx = smooth_cx(cx)
                    norm_cx     = (smoothed_cx / FRAME_W) - 0.5
                    locked_cx   = norm_cx

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (int(smoothed_cx), y + h // 2), 5, (0, 255, 0), -1)
                    cv2.putText(frame, "LOCKED", (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"cx:{norm_cx:.2f} h:{box_height}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                    # --- MOVEMENT LOGIC ---
                    if norm_cx < -DEAD_ZONE:
                        intensity = int(map_range(abs(norm_cx), DEAD_ZONE, 0.5, MIN_TURN, MAX_TURN))
                        send_command(f'L{intensity}')

                    elif norm_cx > DEAD_ZONE:
                        intensity = int(map_range(abs(norm_cx), DEAD_ZONE, 0.5, MIN_TURN, MAX_TURN))
                        send_command(f'R{intensity}')

                    else:
                        # Person centered — check distance
                        if box_height > TARGET_HEIGHT + HEIGHT_MARGIN:
                            send_command('S')   # too close — stop
                        elif box_height < TARGET_HEIGHT - HEIGHT_MARGIN:
                            send_command('F')   # too far — move forward
                        else:
                            send_command('S')   # correct distance — stop

            else:
                # Tracker lost completely
                if lost_time is None:
                    lost_time = time.time()
                    print("Person lost, waiting...")

                elapsed = time.time() - lost_time
                cv2.putText(frame, f"LOST {elapsed:.1f}s", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                send_command('S')

                if elapsed > LOST_TIMEOUT:
                    print("Re-scanning for person...")
                    tracking        = False
                    tracker         = None
                    lost_time       = None
                    frame_count     = 0
                    locked_cx       = None
                    prev_box_height = None
                    cx_history.clear()

        left_x  = int(FRAME_W * (0.5 - DEAD_ZONE))
        right_x = int(FRAME_W * (0.5 + DEAD_ZONE))
        cv2.line(frame, (left_x,  0), (left_x,  FRAME_H), (0, 100, 255), 1)
        cv2.line(frame, (right_x, 0), (right_x, FRAME_H), (0, 100, 255), 1)

        cv2.imshow("Human Following Robot", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    send_command('S')
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    arduino.close()
    print(f"Recording saved to: {output_path}")
    print("Stopped.")