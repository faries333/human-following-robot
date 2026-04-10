# Human Following Robot — Raspberry Pi Vision + Control
import cv2
import serial
import time

# ── Config ──────────────────────────────────────────────
SERIAL_PORT   = '/dev/ttyUSB0'   # or /dev/ttyACM0 — check with: ls /dev/tty*
BAUD_RATE     = 9600
CONFIDENCE_TH = 0.5              # Min detection confidence (0–1)
DEAD_ZONE     = 0.15             # Center fraction — no turn if person is within this band
FRAME_W       = 640
FRAME_H       = 480
PERSON_CLASS  = 15               # MobileNet-SSD class index for "person"

PROTOTXT  = 'deploy.prototxt'
CAFFEMODEL = 'mobilenet_ssd.caffemodel'
# ────────────────────────────────────────────────────────

# Load model
net = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)

# Open serial to Arduino
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)   # Wait for Arduino reset

# Open camera
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

last_cmd = None

def send_command(cmd):
    global last_cmd
    if cmd != last_cmd:           # Only send on change — reduces serial spam
        arduino.write(cmd.encode())
        last_cmd = cmd
        print(f"[CMD] {cmd}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Prepare blob for MobileNet-SSD (300×300 input expected)
        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)),
            0.007843, (300, 300), 127.5
        )
        net.setInput(blob)
        detections = net.forward()

        best_conf  = 0
        best_cx    = None

        # Find the most-confident person detection
        for i in range(detections.shape[2]):
            conf  = float(detections[0, 0, i, 2])
            label = int(detections[0, 0, i, 1])
            if label == PERSON_CLASS and conf > CONFIDENCE_TH and conf > best_conf:
                best_conf = conf
                box  = detections[0, 0, i, 3:7] * [FRAME_W, FRAME_H, FRAME_W, FRAME_H]
                x1, y1, x2, y2 = box.astype(int)
                best_cx = (x1 + x2) / 2   # Horizontal center of bounding box

                # Draw box (for debug preview)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
                cv2.putText(frame, f"Person {conf:.2f}", (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

        # ── Decision logic ──────────────────────────────
        if best_cx is None:
            send_command('S')   # No person detected → stop
        else:
            # Normalise cx to -1 (far left) … +1 (far right)
            norm_cx = (best_cx / FRAME_W) - 0.5   # range: -0.5 to +0.5
            if   norm_cx < -DEAD_ZONE:  send_command('L')
            elif norm_cx >  DEAD_ZONE:  send_command('R')
            else:                       send_command('F')   # Person is centred → go forward

        # Draw center dead-zone lines (visual debug)
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
