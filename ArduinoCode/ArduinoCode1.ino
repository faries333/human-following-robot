// Human Following Robot — Arduino Motor Controller
// Pins: ENA=5, IN1=6, IN2=7, IN3=8, IN4=9, ENB=10

const int ENA = 5;   // Left motor speed (PWM)
const int IN1 = 6;   // Left motor direction
const int IN2 = 7;
const int ENB = 10;  // Right motor speed (PWM)
const int IN3 = 8;   // Right motor direction
const int IN4 = 9;

const int BASE_SPEED = 100;   // Reduced from 160
const int TURN_SPEED = 80;    // Reduced from 130

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'F': moveForward();  break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft();     break;
      case 'R': turnRight();    break;
      case 'S': stopMotors();   break;
    }
  }
}

void moveForward() {
  analogWrite(ENA, BASE_SPEED);  analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left fwd
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right fwd
}

void moveBackward() {
  analogWrite(ENA, BASE_SPEED);  analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, TURN_SPEED);  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right forward
}

void turnRight() {
  analogWrite(ENA, TURN_SPEED);  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right backward
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}