// Human Following Robot — Arduino Motor Controller
// Motors: 2x Blue 6V Gear TT Motor with Metal Gears
// Pins: ENA=5, IN1=6, IN2=7, IN3=8, IN4=9, ENB=10

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

const int BASE_SPEED = 85;
const int TURN_SPEED = 50;    // Reduced from 70
const int LEFT_TRIM  = 20;    // Your tuned value

const int L_SPEED = BASE_SPEED + LEFT_TRIM;
const int R_SPEED = BASE_SPEED;
const int L_TURN  = TURN_SPEED + LEFT_TRIM;
const int R_TURN  = TURN_SPEED;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  stopMotors();
  delay(500);
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
  analogWrite(ENA, L_SPEED);
  analogWrite(ENB, R_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, L_SPEED);
  analogWrite(ENB, R_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, L_TURN);
  analogWrite(ENB, R_TURN);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right forward
  delay(150);                                         // Short burst turn
  stopMotors();
}

void turnRight() {
  analogWrite(ENA, L_TURN);
  analogWrite(ENB, R_TURN);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right backward
  delay(150);                                         // Short burst turn
  stopMotors();
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}