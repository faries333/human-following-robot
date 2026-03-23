# Human Following Robot 🤖

An autonomous two-wheeled robot that detects and follows a human in real time using a camera and deep learning.

## About The Robot
![About the Robot](Human_Following_Robot.png)

## How It Works
- Raspberry Pi captures live video from a USB webcam
- MobileNet SSD neural network detects people in the frame
- OpenCV CSRT tracker locks onto a single person
- Position is calculated and commands are sent to Arduino via Serial
- Arduino drives two DC gear motors based on the command received

## Hardware
- Arduino Uno
- Raspberry Pi
- USB Webcam
- L298N Motor Driver
- 2x Blue 6V Gear TT Motor (Metal Gears)
- Battery Pack
  
## Circuit Diagram
![Circuit Diagram](circuit_diagram.svg)

## Pin Connections
| Arduino Pin | L298N |
|-------------|-------|
| Pin 5 (PWM) | ENA — Left motor speed |
| Pin 6 | IN1 — Left motor direction |
| Pin 7 | IN2 — Left motor direction |
| Pin 8 | IN3 — Right motor direction |
| Pin 9 | IN4 — Right motor direction |
| Pin 10 (PWM) | ENB — Right motor speed |

## Commands
| Command | Action |
|---------|--------|
| F | Move Forward |
| B | Move Backward |
| L | Turn Left |
| R | Turn Right |
| S | Stop |

## Files
| File | Description |
|------|-------------|
| `robot.py` | Main Python script — runs on Raspberry Pi |
| `robot.ino` | Arduino motor control code |

## Project Status
🟡 Work in Progress

### Done ✅
- Person detection with MobileNet SSD
- Single person lock-on with CSRT tracker
- Motor speed balancing with LEFT_TRIM
- Turn overshoot fix with burst turns
- Video recording saved to ~/robot

### In Progress 🔧
- Fix tracker locking onto objects
- Fix robot stopping when person moves sideways
- Distance control using bounding box height

### To Do 📋
- Person re-identification after occlusion
- Obstacle avoidance
- Proportional speed control
