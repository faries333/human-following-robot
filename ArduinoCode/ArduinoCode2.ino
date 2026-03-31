// Human Following Robot — Arduino Motor Controller
// Motors: 2x Blue 6V Gear TT Motor with Metal Gears

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

const int BASE_SPEED = 85;
const int LEFT_TRIM  = 20;

const int L_SPEED = BASE_SPEED + LEFT_TRIM;
const int R_SPEED = BASE_SPEED;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  stopMotors();
  delay(500);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char cmd = input.charAt(0);

    int speed = 60;   // safe default
    if (input.length() > 1) {
      speed = input.substring(1).toInt();
      speed = constrain(speed, 40, 100);  // hard cap at 100 for safety
    }

    switch (cmd) {
      case 'F': moveForward();    break;
      case 'B': moveBackward();   break;
      case 'L': turnLeft(speed);  break;
      case 'R': turnRight(speed); break;
      case 'S': stopMotors();     break;
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

void turnLeft(int speed) {
  analogWrite(ENA, speed + LEFT_TRIM);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(100);       // short burst — prevents 90 degree overshoot
  stopMotors();
}

void turnRight(int speed) {
  analogWrite(ENA, speed + LEFT_TRIM);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(100);       // short burst — prevents 90 degree overshoot
  stopMotors();
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}