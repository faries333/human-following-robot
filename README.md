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

## 🛠️ Complete Setup Guide (Beginner Friendly)

> **Follow every step in order. Copy and paste each command exactly as shown.**  
> You will need: Raspberry Pi 4 running Raspberry Pi OS, Arduino Uno connected via USB, and a USB webcam plugged in.

---

### Step 1 — Update Your Raspberry Pi

Open a terminal on the Raspberry Pi and run:

```bash
sudo apt update && sudo apt upgrade -y
```

> This makes sure your Pi has the latest software. It may take a few minutes.

---

### Step 2 — Install Git (if not already installed)

```bash
sudo apt install git -y
```

Check it worked:

```bash
git --version
```

You should see something like `git version 2.x.x`.

---

### Step 3 — Clone This Repository

```bash
cd ~
git clone https://github.com/faries333/human-following-robot.git
```

This downloads all the project files into a folder called `human-following-robot` on your Pi.

Navigate into it:

```bash
cd human-following-robot
```

---

### Step 4 — Install Python Dependencies

```bash
pip3 install opencv-contrib-python pyserial numpy
```

> ⏳ `opencv-contrib-python` is a large package — this may take **5–10 minutes** on a Pi. Let it finish.

Verify the install:

```bash
python3 -c "import cv2, serial, numpy; print('All libraries installed successfully!')"
```

You should see: `All libraries installed successfully!`

---

### Step 5 — Upload the Arduino Code

1. On a **laptop or desktop**, download and install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Open the file: `ArduinoCode/FinalWorkingMotorCode.ino`
3. Connect the Arduino Uno to your laptop via USB
4. In the Arduino IDE:
   - Go to **Tools → Board** → select `Arduino Uno`
   - Go to **Tools → Port** → select the port that shows your Arduino (e.g. `COM3` on Windows or `/dev/ttyUSB0` on Linux)
5. Click the **Upload** button (→ arrow icon)
6. Wait for `Done uploading.` to appear at the bottom
7. Disconnect the Arduino from the laptop and connect it to the Raspberry Pi via the USB-A to USB-B cable

---

### Step 6 — Find the Arduino Port on the Raspberry Pi

Run this command **after** plugging the Arduino into the Pi:

```bash
ls /dev/ttyUSB*
```

You should see something like `/dev/ttyUSB0` or `/dev/ttyACM0`.

> If nothing shows up, try `ls /dev/ttyACM*` instead.

If you see a port other than `/dev/ttyUSB0`, open the Python file and update the port:

```bash
nano ~/human-following-robot/PythonCode/FinalWorkingPythonCode.py
```

Find the line that says:

```python
SERIAL_PORT = '/dev/ttyUSB0'
```

Change it to match your port (e.g. `/dev/ttyACM0`), then save with `Ctrl+O` → `Enter` → `Ctrl+X`.

---

### Step 7 — Give Permission to Access the Serial Port

```bash
sudo usermod -a -G dialout $USER
```

Then **reboot** the Pi:

```bash
sudo reboot
```

After it restarts, open a terminal again and go back to the project folder:

```bash
cd ~/human-following-robot
```

---

## ▶️ How to Run the Robot

> Make sure the Arduino is plugged into the Pi, the webcam is plugged in, and the motors are connected before running.

```bash
python3 PythonCode/FinalWorkingPythonCode.py
```

**What happens next:**

1. A camera window opens showing the live feed
2. Stand in front of the robot — it will detect you automatically
3. The robot locks onto the person closest to the centre of the frame
4. It will start following you as you move left or right
5. Press **`Q`** on the keyboard to stop the program and shut down the motors safely

---

### ⚠️ Troubleshooting

| Problem | Fix |
|--------|-----|
| `No module named cv2` | Run `pip3 install opencv-contrib-python` again |
| `Serial port not found` | Check the port with `ls /dev/ttyUSB*` and update `SERIAL_PORT` in the script |
| `Permission denied: '/dev/ttyUSB0'` | Run `sudo usermod -a -G dialout $USER` and reboot |
| Camera window doesn't open | Make sure the webcam is plugged in before running the script |
| Robot doesn't move | Check motor wiring to L298N and confirm Arduino was uploaded successfully |
| Robot turns the wrong way | The LEFT_TRIM or motor wires may be swapped — check L298N connections |

---

## Project Status

🟢 **Working**

### Done ✅
- Real-time person detection with MobileNet-SSD (COCO, Caffe)
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

| **Student** | Khader Faries |
| **Register No.** | 23BCARI058 |
| **Course** | BCA (AI, ML, Robotics & IoT) with Microsoft |
| **Institution** | Yenepoya Institute of Arts, Science, Commerce and Management, Mangalore |
| **Guide** | Ms. Shreya, Department of Computer Science |
| **Program** | TCE Internship — Sahyadri College of Engineering & Management |
| **Mentor** | Pulkit Garg, Technical Career Education |
