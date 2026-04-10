import cv2
import serial
import time

SERIAL_PORT   = '/dev/ttyACM0'
BAUD_RATE     = 9600
CONFIDENCE_TH = 0.5
DEAD_ZONE     = 0.25
FRAME_W       = 640
FRAME_H       = 480
PERSON_CLASS  = 15
LOST_TIMEOUT  = 2.0          # seconds before re-scanning for new person

PROTOTXT   = '/home/piuser/robot/deploy.prototxt'
CAFFEMODEL = '/home/piuser/robot/mobilenet_iter_73000.caffemodel'

net = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

last_cmd     = None
tracker      = None
tracking     = False
lost_time    = None

def send_command(cmd):
    global last_cmd
    if cmd != last_cmd:
        arduino.write(cmd.encode())
        last_cmd = cmd
        print(f"CMD: {cmd}")

def create_tracker():
    # CSRT is the most accurate tracker available in OpenCV
    return cv2.TrackerCSRT_create()

def detect_person(frame):
    """Run MobileNet SSD and return the best person bounding box or None."""
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
            # Return as (x, y, w, h) format for tracker
            best_box = (x1, y1, x2 - x1, y2 - y1)

    return best_box

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        frame = cv2.flip(frame, 1)

        if not tracking:
            # --- SCAN MODE: looking for a person to lock onto ---
            cv2.putText(frame, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

            box = detect_person(frame)
            if box is not None:
                # Person found — initialize tracker and lock on
                tracker = create_tracker()
                tracker.init(frame, box)
                tracking  = True
                lost_time = None
                print("Locked onto person")

            send_command('S')  # Stay still while scanning

        else:
            # --- TRACKING MODE: following the locked person ---
            success, box = tracker.update(frame)

            if success:
                lost_time = None  # Reset lost timer
                x, y, w, h = [int(v) for v in box]
                cx = x + w // 2

                # Draw locked bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + h // 2), 5, (0, 255, 0), -1)
                cv2.putText(frame, "LOCKED", (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Decide movement based on horizontal position
                norm_cx = (cx / FRAME_W) - 0.5
                if   norm_cx < -DEAD_ZONE: send_command('L')
                elif norm_cx >  DEAD_ZONE: send_command('R')
                else:                      send_command('F')

            else:
                # Tracker lost the person
                if lost_time is None:
                    lost_time = time.time()
                    print("Person lost, waiting...")

                elapsed = time.time() - lost_time
                cv2.putText(frame, f"LOST {elapsed:.1f}s", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                send_command('S')  # Stop while lost

                # If lost for too long, go back to scan mode
                if elapsed > LOST_TIMEOUT:
                    print("Re-scanning for person...")
                    tracking  = False
                    tracker   = None
                    lost_time = None

        # Draw dead zone lines
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
    cv2.destroyAllWindows()
    arduino.close()
    print("Stopped.")