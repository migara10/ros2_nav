// Differential drive controller with encoders + HC-SR04 front sonar
// In:  "L<speed> R<speed>\n"   speed range -255..255
// Out: "E<left_ticks> <right_ticks>\n" every 50 ms
//      "U<distance_cm>\n" every 100 ms ("U-1" when no echo)

#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 8
#define IN4 9

#define L_ENC_A 2
#define L_ENC_B 4
#define R_ENC_A 3
#define R_ENC_B 11

#define TRIG_PIN 12
#define ECHO_PIN 13

// set to -1 if the count decreases while moving forward
const int LEFT_ENC_SIGN  = -1;
const int RIGHT_ENC_SIGN = 1;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

String inputString = "";
bool stringComplete = false;

unsigned long lastCommandTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastSonarTime = 0;
const unsigned long TIMEOUT_MS = 1000;
const unsigned long PUBLISH_MS = 50;
const unsigned long SONAR_MS = 100;
const unsigned long SONAR_TIMEOUT_US = 12000UL;  // about 2 m

void leftISR() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) leftTicks--;
  else leftTicks++;
}

void rightISR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) rightTicks--;
  else rightTicks++;
}

float readSonarCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, CHANGE);

  stopMotors();
  Serial.println("Ready");
}

void loop() {
  unsigned long now = millis();

  if (lastCommandTime != 0 && now - lastCommandTime > TIMEOUT_MS) {
    stopMotors();
  }

  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = now;
  }

  if (now - lastPublishTime >= PUBLISH_MS) {
    lastPublishTime = now;
    long l, r;
    noInterrupts();
    l = leftTicks;
    r = rightTicks;
    interrupts();
    Serial.print("E");
    Serial.print(l * LEFT_ENC_SIGN);
    Serial.print(" ");
    Serial.println(r * RIGHT_ENC_SIGN);
  }

  if (now - lastSonarTime >= SONAR_MS) {
    lastSonarTime = now;
    float d = readSonarCm();
    Serial.print("U");
    Serial.println(d, 1);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void parseCommand(String cmd) {
  int lIndex = cmd.indexOf('L');
  int rIndex = cmd.indexOf('R');
  if (lIndex == -1 || rIndex == -1 || rIndex < lIndex) return;

  int leftSpeed = cmd.substring(lIndex + 1, rIndex).toInt();
  int rightSpeed = cmd.substring(rIndex + 1).toInt();

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotor(leftSpeed, ENA, IN1, IN2);
  setMotor(rightSpeed, ENB, IN3, IN4);
}

void setMotor(int speed, int enPin, int in1Pin, int in2Pin) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

void stopMotors() {
  setMotor(0, ENA, IN1, IN2);
  setMotor(0, ENB, IN3, IN4);
}
