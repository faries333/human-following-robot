import cv2
import serial
import time

SERIAL_PORT   = '/dev/ttyACM0'
BAUD_RATE     = 9600
CONFIDENCE_TH = 0.5
DEAD_ZONE     = 0.15
FRAME_W       = 640
FRAME_H       = 480
PERSON_CLASS  = 15

PROTOTXT   = '/home/piuser/robot/deploy.prototxt'
CAFFEMODEL = '/home/piuser/robot/mobilenet_iter_73000.caffemodel'

net = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

last_cmd = None

def send_command(cmd):
    global last_cmd
    if cmd != last_cmd:
        arduino.write(cmd.encode())
        last_cmd = cmd
        print(f"CMD: {cmd}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)),
            0.007843, (300, 300), 127.5
        )
        net.setInput(blob)
        detections = net.forward()

        best_conf = 0
        best_cx   = None

        for i in range(detections.shape[2]):
            conf  = float(detections[0, 0, i, 2])
            label = int(detections[0, 0, i, 1])
            if label == PERSON_CLASS and conf > CONFIDENCE_TH and conf > best_conf:
                best_conf = conf
                box = detections[0, 0, i, 3:7] * [FRAME_W, FRAME_H, FRAME_W, FRAME_H]
                x1, y1, x2, y2 = box.astype(int)
                best_cx = (x1 + x2) / 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Person {conf:.2f}", (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if best_cx is None:
            send_command('S')
        else:
            norm_cx = (best_cx / FRAME_W) - 0.5
            if   norm_cx < -DEAD_ZONE: send_command('L')
            elif norm_cx >  DEAD_ZONE: send_command('R')
            else:                      send_command('F')

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
