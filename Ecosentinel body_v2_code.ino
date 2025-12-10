// Motor A
const int ENA = 11;
const int IN1 = 10;
const int IN2 = 9;
#define s 90   // base speed
#define t 255   // turning speed

// Motor B
const int ENB = 6;
const int IN3 = 8;
const int IN4 = 7;

const int IRSensorLeft = 3;
const int IRSensorRight = 2;

// Ultrasonic Sensors
const int trig1 = 12; 
const int echo1 = 13;
const int trig2 = 4;
const int echo2 = 5;

const int switchPin = A0;
const int buzzer = A1;

bool botRunning = false;
unsigned long lastBeepTime = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IRSensorLeft, INPUT);
  pinMode(IRSensorRight, INPUT);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(switchPin, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);

  Serial.begin(9600);

  // System ON beep
  tone(buzzer, 1000, 200);
  delay(300);
  noTone(buzzer);
}

void loop() {
  static bool lastSwitchState = HIGH;
  bool switchState = digitalRead(switchPin);

  // Start/stop toggle
  if (lastSwitchState == HIGH && switchState == LOW) {
    botRunning = !botRunning;
    tone(buzzer, 1000, 200);
    delay(300);
    noTone(buzzer);
  }
  lastSwitchState = switchState;

  if (!botRunning) {
    stopMotors();
    return;
  }

  // Read ultrasonic distances
  long distance1 = getDistance(trig1, echo1);
  long distance2 = getDistance(trig2, echo2);

  // Treat invalid readings (0 or >400cm) as obstacles (fail-safe)
  if (distance1 == 0 || distance1 > 400) distance1 = 999;
  if (distance2 == 0 || distance2 > 400) distance2 = 999;

  // Stop if any sensor detects an obstacle within 25cm
  bool obstacleDetected = (distance1 <= 25) || (distance2 <= 25);

  if (obstacleDetected) {
    stopMotors();

    // Buzzer beep every 1 sec while obstacle present
    if (millis() - lastBeepTime > 1000) {
      tone(buzzer, 1000, 200);
      delay(200);
      noTone(buzzer);
      lastBeepTime = millis();
    }

    Serial.print("Obstacle Detected! D1: ");
    Serial.print(distance1);
    Serial.print("  D2: ");
    Serial.println(distance2);
    return;
  }

  // Line following logic (unchanged)
  bool leftSensor = digitalRead(IRSensorLeft);
  bool rightSensor = digitalRead(IRSensorRight);

  Serial.print("Left: ");
  Serial.print(leftSensor);
  Serial.print(" | Right: ");
  Serial.print(rightSensor);
  Serial.print(" | D1: ");
  Serial.print(distance1);
  Serial.print(" | D2: ");
  Serial.println(distance2);

  if (leftSensor == LOW && rightSensor == LOW) {
    moveForward();
  } 
  else if (leftSensor == LOW && rightSensor == HIGH) {
    turnRight();
  } 
  else if (leftSensor == HIGH && rightSensor == LOW) {
    turnLeft();
  } 
  else {
    stopMotors();
  }

  delay(50);
}

// More reliable ultrasonic reading
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 20000); // shorter timeout
  if (duration == 0) return 0;  // No echo received
  long distance = duration * 0.034 / 2;
  return distance;
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, s);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, s);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, t);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, t);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, t);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, t);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
