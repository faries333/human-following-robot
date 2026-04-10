# Human Following Robot 🤖

An autonomous two-wheeled robot that detects, identifies, and follows a single designated person in real time using a camera and deep learning — even when multiple people are present in the frame.

![Human Following Robot](AboutRobot.png)

---

## How It Works

1. **Raspberry Pi 4** captures live video from a USB webcam at 640×480
2. **MobileNet-SSD** (via OpenCV DNN) detects all people in the frame
3. At startup, the robot locks onto the person closest to the frame centre and captures a **colour histogram identity signature** of their torso
4. **OpenCV CSRT tracker** follows the locked person's bounding box frame-by-frame
5. Every 15 frames, MobileNet-SSD re-validates the tracked box against the identity signature using **Bhattacharyya distance** — ensuring the robot never switches to a different person
6. The horizontal position of the tracked person is fed into a **PD controller** which calculates turn speed
7. Commands are sent to Arduino via **57600 baud binary serial** as 2-byte packets `[CMD, SPEED]`
8. **Arduino Uno** drives two DC gear motors through an **L298N motor driver**
9. A **300ms watchdog** on the Arduino stops the motors automatically if the Pi goes silent

---

## Hardware

| Component | Details |
|-----------|---------|
| Raspberry Pi 4 Model B | 4GB RAM — vision and control computer |
| Arduino Uno R3 | Motor controller |
| USB Webcam | Visual input |
| L298N Motor Driver | Dual H-bridge for 2 DC motors |
| 2× TT Gear Motor (6V) | Blue DC motors with metal gears |
| 3-Wheel Chassis | 2 driven wheels + 1 caster |
| 7.4V LiPo Battery Pack | Power supply |
| USB-A to USB-B Cable | Pi to Arduino serial communication |

---

## Circuit Diagram

![Circuit Diagram](OriginalCircuitDiagram.png)

## Pin Connections

| Arduino Pin | L298N | Function |
|-------------|-------|----------|
| Pin 5 (PWM) | ENA | Left motor speed |
| Pin 6 | IN1 | Left motor direction |
| Pin 7 | IN2 | Left motor direction |
| Pin 8 | IN3 | Right motor direction |
| Pin 9 | IN4 | Right motor direction |
| Pin 10 (PWM) | ENB | Right motor speed |

---

## Software & Libraries

| Software | Version | Purpose |
|----------|---------|---------|
| Raspberry Pi OS | Ubuntu-based Linux | Operating system |
| Python 3 | 3.x | Main programming language |
| OpenCV (`opencv-contrib-python`) | 4.x | Computer vision, DNN, CSRT tracker |
| PySerial | Latest | Serial communication with Arduino |
| NumPy | Latest | Array operations for histogram |
| Arduino IDE | Latest | Motor controller firmware |

### Install dependencies on Raspberry Pi

```bash
pip3 install opencv-contrib-python pyserial numpy
```

---

## Serial Command Protocol

Commands are sent as **2-byte binary packets**: `[CMD_BYTE, SPEED_BYTE]` at **57600 baud**.

| CMD Byte | Action | SPEED Byte |
|----------|--------|------------|
| `F` | Move Forward | PWM value (40–220) |
| `B` | Move Backward | PWM value (40–220) |
| `L` | Turn Left (burst) | PWM value (40–220) |
| `R` | Turn Right (burst) | PWM value (40–220) |
| `S` | Stop | 0 |

---

## Repository Structure

```
human-following-robot/
├── PythonCode/
│   ├── robot_test.py        # Initial hardware connectivity test
│   ├── vision.py            # v1 — basic person detection
│   ├── vision2.py           # v2 — added CSRT tracker
│   ├── vision3.py           # v3 — serial communication
│   ├── vision4.py           # v4 — motor command mapping
│   ├── vision5.py           # v5 — dead zone control
│   ├── vision6.py           # v6 — smoothing and cooldown
│   ├── vision7.py           # v7 — PD controller
│   ├── vision8.py           # v8 — mirror fix + burst turns
│   ├── vision9.py           # v9 — single-person identity lock
│   ├── vision10.py          # v10 — async re-validation
│   ├── vision11.py          # v11 — histogram threshold tuning
│   ├── vision12.py          # v12 — performance optimisation
│   ├── vision13.py          # v13 — full system integration
│   └── FinalWorkingPythonCode.py          # ✅ FINAL — production-ready code
├── ArduinoCode/
│   ├── ArduinoCode1.ino     # v1 — basic motor test
│   ├── ArduinoCode2.ino     # v2 — direction control
│   ├── ArduinoCode3.ino     # v3 — PWM speed control
│   ├── ArduinoCode4.ino     # v4 — serial command parsing
│   └── FinalWorkingMotorCode.ino  # ✅ FINAL — 57600 baud, watchdog, no blocking delays
├── COMMANDS.md              # All terminal and Git commands used in this project
├── AboutRobot.png           # Robot photo
└── README.md
```

---

## Key Parameters (FinalWorkingPythonCode.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `BAUD_RATE` | 57600 | Serial speed (6× faster than default) |
| `DEAD_ZONE` | 0.15 | Normalised centre dead-band |
| `HIST_MATCH_THRESHOLD` | 0.45 | Bhattacharyya distance for identity match |
| `REVALIDATE_EVERY` | 15 | Frames between identity re-checks |
| `Kp` | 170.0 | PD proportional gain |
| `Kd` | 25.0 | PD derivative gain |
| `BURST_MAX_S` | 0.12s | Maximum single turn burst duration |
| `LOST_TIMEOUT` | 2.0s | Wait time before re-scanning after loss |
| `LEFT_TRIM` | 20 | PWM compensation for motor imbalance |

---

## Running the Robot

```bash
# On the Raspberry Pi
python3 /home/piuser/robot/FinalWorkingPythonCode.py

# Press Q to quit
```

---

## Project Status

🟢 **Working**

### Done ✅
- Real-time person detection with MobileNet-SSD (COCO, Caffe)
- Single-person identity lock using HS colour histogram + Bhattacharyya distance
- Frame-by-frame tracking with OpenCV CSRT tracker
- Threaded background re-validation (no main loop stalls)
- PD controller for proportional turn speed
- Burst turn control to prevent over-turning
- Mirror/flip correction with automatic turn direction fix
- Binary 2-byte serial protocol at 57600 baud
- Arduino watchdog — auto-stop if Pi goes silent
- Motor imbalance compensation via LEFT_TRIM
- Video recording saved to `~/robot/`

### To Do 📋
- Distance control using bounding box height (stop when close enough)
- Obstacle avoidance using ultrasonic sensors
- Migrate control architecture to ROS2
- Depth camera integration for 3D distance measurement

---

## Project Info

| | |
|---|---|
| **Student** | Khader Faries |
| **Register No.** | 23BCARI058 |
| **Course** | BCA (AI, ML, Robotics & IoT) with Microsoft |
| **Institution** | Yenepoya Institute of Arts, Science, Commerce and Management, Mangalore |
| **Guide** | Ms. Shreya, Department of Computer Science |
| **Program** | TCE Internship — Sahyadri College of Engineering & Management |
| **Mentor** | Pulkit Garg, Technical Career Education |
