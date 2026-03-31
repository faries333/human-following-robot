// Human Following Robot — Arduino Motor Controller (v2 Optimised)
//
// Key changes from original:
//   • 57600 baud  — 6× faster than 9600
//   • 2-byte binary protocol: [CMD_BYTE, SPEED_BYTE] — no string parsing
//   • NO delay() in turn functions — Pi loop controls burst duration
//   • Watchdog: stops motors if Pi goes silent for > 300 ms
//   • Speed range widened to 0–220 (was capped at 100)
//   • LEFT_TRIM applied consistently in all motion functions

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

// Compensates for the stronger left motor.
// Increase if robot still pulls left; decrease if it pulls right.
const int LEFT_TRIM = 20;

// Watchdog: if no command arrives within this many ms, stop for safety.
const unsigned long WATCHDOG_MS = 300;
unsigned long lastCmdTime = 0;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(57600);
  stopMotors();
  lastCmdTime = millis();
}

void loop() {
  // ── Watchdog ──────────────────────────────────────────────────────────────
  if (millis() - lastCmdTime > WATCHDOG_MS) {
    stopMotors();
  }

  // ── Read 2-byte packet ────────────────────────────────────────────────────
  if (Serial.available() >= 2) {
    byte cmd   = Serial.read();   // 'F', 'B', 'L', 'R', 'S'
    byte speed = Serial.read();   // 0–255 raw PWM value from Pi
    lastCmdTime = millis();

    // Clamp to a safe operating range
    speed = constrain(speed, 40, 220);

    switch (cmd) {
      case 'F': moveForward(speed);   break;
      case 'B': moveBackward(speed);  break;
      case 'L': turnLeft(speed);      break;
      case 'R': turnRight(speed);     break;
      case 'S': stopMotors();         break;
    }
  }
}

// ── Motion functions ──────────────────────────────────────────────────────────
// All four functions apply LEFT_TRIM to ENA so the robot tracks straight.
// Clamp ENA to 255 so trim never wraps around.

void moveForward(int speed) {
  analogWrite(ENA, min(speed + LEFT_TRIM, 255));
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // left  motor forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // right motor forward
}

void moveBackward(int speed) {
  analogWrite(ENA, min(speed + LEFT_TRIM, 255));
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // left  motor backward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // right motor backward
}

void turnLeft(int speed) {
  // Left wheel backward, right wheel forward → pivot left in place.
  // NO delay — burst duration is controlled by the Pi (vision8.py).
  analogWrite(ENA, min(speed + LEFT_TRIM, 255));
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // left  backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // right forward
}

void turnRight(int speed) {
  // Left wheel forward, right wheel backward → pivot right in place.
  analogWrite(ENA, min(speed + LEFT_TRIM, 255));
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // left  forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // right backward
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
