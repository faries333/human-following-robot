import cv2
import serial
import time
import numpy as np

SERIAL_PORT   = '/dev/ttyACM0'
BAUD_RATE     = 9600
CONFIDENCE_TH = 0.5
DEAD_ZONE     = 0.20
FRAME_W       = 640
FRAME_H       = 480
PERSON_CLASS  = 15
LOST_TIMEOUT  = 3.0
SMOOTH_FRAMES = 3
CMD_COOLDOWN  = 0.3
MIN_TURN      = 50
MAX_TURN      = 80

TARGET_HEIGHT = 280   # box height in pixels to stop at
HEIGHT_MARGIN = 40    # tolerance around target height

# Max distance (normalized) Person A can move between frames
# If no detection within this distance → someone else is blocking
MAX_MOVE      = 0.30

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

last_cmd      = None
last_cmd_time = 0
cx_history    = []
tracking      = False
lost_time     = None

# Last known position of Person A (normalized 0.0 to 1.0)
person_a_cx   = None
person_a_cy   = None

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

def smooth_cx(new_cx):
    cx_history.append(new_cx)
    if len(cx_history) > SMOOTH_FRAMES:
        cx_history.pop(0)
    return sum(cx_history) / len(cx_history)

def get_all_persons(frame):
    """Run MobileNet and return ALL person detections above threshold."""
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)),
        0.007843, (300, 300), 127.5
    )
    net.setInput(blob)
    detections = net.forward()

    persons = []
    for i in range(detections.shape[2]):
        conf  = float(detections[0, 0, i, 2])
        label = int(detections[0, 0, i, 1])
        if label == PERSON_CLASS and conf > CONFIDENCE_TH:
            box = detections[0, 0, i, 3:7] * [FRAME_W, FRAME_H, FRAME_W, FRAME_H]
            x1, y1, x2, y2 = box.astype(int)
            persons.append({
                'box' : (x1, y1, x2 - x1, y2 - y1),
                'conf': conf,
                'cx'  : (x1 + x2) / 2 / FRAME_W,   # normalized 0-1
                'cy'  : (y1 + y2) / 2 / FRAME_H,    # normalized 0-1
            })
    return persons

def find_person_a(persons, last_cx, last_cy):
    """
    From all detections, find the one closest to Person A's
    last known position. Returns None if no match within MAX_MOVE.
    """
    if not persons or last_cx is None:
        return None

    best      = None
    best_dist = float('inf')

    for p in persons:
        dist = ((p['cx'] - last_cx) ** 2 + (p['cy'] - last_cy) ** 2) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best      = p

    # If closest person is too far from last known position → blocked
    if best_dist > MAX_MOVE:
        return None

    return best

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        out.write(frame)
        persons = get_all_persons(frame)

        if not tracking:
            # --- SCAN MODE: wait for first person ---
            cv2.putText(frame, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            if persons:
                # Lock onto the most confident person
                best = max(persons, key=lambda p: p['conf'])
                person_a_cx = best['cx']
                person_a_cy = best['cy']
                tracking    = True
                lost_time   = None
                cx_history.clear()
                print(f"Locked onto Person A at cx={person_a_cx:.2f}")

            send_command('S')

        else:
            # --- TRACKING MODE: always follow Person A by position ---
            match = find_person_a(persons, person_a_cx, person_a_cy)

            if match is not None:
                # Found Person A — update their last known position
                lost_time   = None
                person_a_cx = match['cx']
                person_a_cy = match['cy']

                x, y, w, h  = match['box']
                box_height  = h
                print(f"box_height: {box_height}")
                cx_px       = x + w // 2

                smoothed_cx = smooth_cx(cx_px)
                norm_cx     = (smoothed_cx / FRAME_W) - 0.5

                # Draw green box around Person A
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(smoothed_cx), y + h // 2), 5, (0, 255, 0), -1)
                cv2.putText(frame, "LOCKED - Person A", (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"cx:{norm_cx:.2f} h:{box_height}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # Draw red boxes around ALL other people
                for p in persons:
                    if p is not match:
                        px, py, pw, ph = p['box']
                        cv2.rectangle(frame, (px, py),
                                      (px + pw, py + ph), (0, 0, 255), 1)
                        cv2.putText(frame, "ignored", (px, py - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                # --- MOVEMENT LOGIC ---
                if norm_cx < -DEAD_ZONE:
                    intensity = int(map_range(abs(norm_cx), DEAD_ZONE, 0.5,
                                              MIN_TURN, MAX_TURN))
                    send_command(f'L{intensity}')

                elif norm_cx > DEAD_ZONE:
                    intensity = int(map_range(abs(norm_cx), DEAD_ZONE, 0.5,
                                              MIN_TURN, MAX_TURN))
                    send_command(f'R{intensity}')

                else:
                    # Centered — check distance
                    if box_height > TARGET_HEIGHT + HEIGHT_MARGIN:
                        send_command('S')   # too close — stop
                    elif box_height < TARGET_HEIGHT - HEIGHT_MARGIN:
                        send_command('F')   # too far — move forward
                    else:
                        send_command('S')   # perfect distance — stop

            else:
                # Person A not found — either blocked or lost
                if persons:
                    # Other people visible but not Person A → blocked
                    cv2.putText(frame, "BLOCKED - WAITING FOR PERSON A", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    # Draw red boxes on blocking people
                    for p in persons:
                        px, py, pw, ph = p['box']
                        cv2.rectangle(frame, (px, py),
                                      (px + pw, py + ph), (0, 0, 255), 1)
                    send_command('S')
                    # Don't start lost timer — just wait
                    lost_time = None

                else:
                    # Nobody visible at all → truly lost
                    if lost_time is None:
                        lost_time = time.time()
                        print("Person A lost completely...")

                    elapsed = time.time() - lost_time
                    cv2.putText(frame, f"LOST {elapsed:.1f}s", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    send_command('S')

                    if elapsed > LOST_TIMEOUT:
                        print("Re-scanning...")
                        tracking    = False
                        lost_time   = None
                        person_a_cx = None
                        person_a_cy = None
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