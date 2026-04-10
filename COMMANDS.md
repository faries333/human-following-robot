# Commands Used — Human Following Robot Project

All commands used during the development, testing, and deployment of the
Vision-Based Human Following Robot project.

---

## 1. Raspberry Pi Setup

### Update and upgrade the system
```bash
sudo apt update
sudo apt upgrade -y
```

### Check Python version
```bash
python3 --version
```

### Check pip version
```bash
pip3 --version
```

---

## 2. Installing Python Libraries

### Install OpenCV (with DNN and CSRT tracker support)
```bash
pip3 install opencv-contrib-python
```

### Install PySerial (for Arduino serial communication)
```bash
pip3 install pyserial
```

### Install NumPy
```bash
pip3 install numpy
```

### Install all dependencies at once (alternative)
```bash
pip3 install opencv-contrib-python pyserial numpy
```

### Verify installations
```bash
python3 -c "import cv2; print('OpenCV:', cv2.__version__)"
python3 -c "import serial; print('PySerial OK')"
python3 -c "import numpy as np; print('NumPy:', np.__version__)"
```

---

## 3. Downloading MobileNet-SSD Model Files

### Create the project directory
```bash
mkdir -p /home/piuser/robot
cd /home/piuser/robot
```

### Download the model files
```bash
wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt
wget https://drive.google.com/uc?id=0B3gersZ2cHIxRm5PMWRoTkdHdHc -O mobilenet_iter_73000.caffemodel
```

### Verify model files exist
```bash
ls -lh /home/piuser/robot/
```

---

## 4. Serial Port Setup (Pi to Arduino)

### List available serial ports
```bash
ls /dev/tty*
```

### Check which port the Arduino is on
```bash
dmesg | grep tty
```

### Give permission to access the serial port
```bash
sudo chmod 666 /dev/ttyACM0
```

### Add user to dialout group (permanent fix for serial access)
```bash
sudo usermod -aG dialout piuser
```

### Verify serial connection (sends a test byte)
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyACM0', 57600, timeout=1); print('Serial OK'); s.close()"
```

---

## 5. Creating Python Files

### Create each Python testing file
```bash
touch /home/piuser/robot/robot_test.py
touch /home/piuser/robot/vision.py
touch /home/piuser/robot/vision2.py
touch /home/piuser/robot/vision3.py
touch /home/piuser/robot/vision4.py
touch /home/piuser/robot/vision5.py
touch /home/piuser/robot/vision6.py
touch /home/piuser/robot/vision7.py
touch /home/piuser/robot/vision8.py
touch /home/piuser/robot/vision9.py
touch /home/piuser/robot/vision10.py
touch /home/piuser/robot/vision11.py
touch /home/piuser/robot/vision12.py
touch /home/piuser/robot/vision13.py
touch /home/piuser/robot/finalWorkingPythonCode.py
```

### Open and edit a file using nano
```bash
nano /home/piuser/robot/finalWorkingPythonCode.py
```

---

## 6. Running Python Files

### Run the initial robot test
```bash
python3 /home/piuser/robot/robot_test.py
```

### Run each vision iteration
```bash
python3 /home/piuser/robot/vision.py
python3 /home/piuser/robot/vision2.py
python3 /home/piuser/robot/vision3.py
python3 /home/piuser/robot/vision4.py
python3 /home/piuser/robot/vision5.py
python3 /home/piuser/robot/vision6.py
python3 /home/piuser/robot/vision7.py
python3 /home/piuser/robot/vision8.py
python3 /home/piuser/robot/vision9.py
python3 /home/piuser/robot/vision10.py
python3 /home/piuser/robot/vision11.py
python3 /home/piuser/robot/vision12.py
python3 /home/piuser/robot/vision13.py
python3 /home/piuser/robot/finalWorkingPythonCode.py
```

### Run the final working code
```bash
python3 /home/piuser/robot/finalWorkingPythonCode.py
```

### Stop a running script
```
Press Ctrl + C
```

---

## 7. Arduino Setup and Upload

### Check Arduino IDE version
```bash
arduino --version
```

### Verify Arduino is detected on the serial port
```bash
ls /dev/ttyACM*
```

### Upload Arduino sketch via command line (Arduino CLI)
```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno ArduinoCode/FinalWorkingMotorCode.ino
```

### Set baud rate for serial monitor (57600)
```bash
stty -F /dev/ttyACM0 57600
```

---

## 8. Camera Testing

### Test if camera is detected
```bash
ls /dev/video*
```

### Test camera with a quick Python capture
```bash
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print('Camera OK:', ret, frame.shape if ret else 'No frame')
cap.release()
"
```

---

## 9. Checking System Resources on Pi

### Check CPU and memory usage while running
```bash
htop
```

### Check CPU temperature (important for Pi under load)
```bash
vcgencmd measure_temp
```

### Check disk space
```bash
df -h
```

### Check running Python processes
```bash
ps aux | grep python3
```

### Kill a stuck Python process
```bash
pkill -f vision14.py
```

---

## 10. Copying Files from Pi to PC (via SCP)

Run these on your **Windows PC** (using PowerShell or Git Bash):

### Copy all Python files from Pi to PC
```bash
scp piuser@192.168.255.19:/home/piuser/robot/*.py "C:\Users\Hp\Documents\humanFollow\Codes\PythonCode\"
```

### Copy all Arduino files from Pi to PC
```bash
scp piuser@<PI_IP_ADDRESS>:/home/piuser/robot/*.ino "C:\Users\Hp\Documents\humanFollow\Codes\ArduinoCode\"
```

### Copy all files at once
```bash
scp -r piuser@192.168.255.19:/home/piuser/robot/ "C:\Users\Hp\Documents\humanFollow\Codes\"
```


---

## 11. Git Commands (on Windows PC)

### Configure Git identity
```bash
git config --global user.name "faries333"
git config --global user.email "farishudyawar@gmail.com"
```

### Clone the repository
```bash
git clone https://github.com/faries333/human-following-robot.git
cd human-following-robot
```

### Create and switch to the internship branch
```bash
git checkout -b internship
```

### Check current branch
```bash
git branch
```

### Check status of files
```bash
git status
```

### Stage all files
```bash
git add .
```

### Stage specific files
```bash
git add PythonCode/finalWorkingPythonCode.py
git add ArduinoCode/FinalWorkingMotorCode.ino
git add COMMANDS.md
```

### Commit with a message
```bash
git commit -m "Add all project files: Python vision iterations and Arduino motor code"
```

### Push the internship branch to GitHub
```bash
git push origin internship
```

### View commit history
```bash
git log --oneline
```

### Check remote URL
```bash
git remote -v
```

---

## 12. Creating a Pull Request

After pushing the `internship` branch:

1. Go to **https://github.com/faries333/human-following-robot**
2. Click the **"Compare & pull request"** button that appears
3. Set:
   - **base branch**: `main`
   - **compare branch**: `internship`
4. Title: `Internship Project — Human Following Robot (All Files)`
5. Description: mention what files are included and tag your mentor if possible
6. Click **"Create pull request"**

---

*Project: Vision-Based Human Following Robot*
*Student: Khader Faries | Register No: 23BCARI058*
*Course: BCA (AI, ML, Robotics & IoT) with Microsoft*
*Institution: Yenepoya Institute, Mangalore* 
