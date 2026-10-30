#include <Servo.h>

// Pin Definitions
#define trigPin 5
#define echoPin 18
#define ENA 14
#define ENB 13
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define servoPin 15

Servo myServo;

// Variables for distance measurement
long duration;
int distance;
int rightDistance, leftDistance;

// Function to calculate distance
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 250); // Adjust speed using
  analogWrite(ENB, 250); // Adjust speed using
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 250); // Adjust speed using
  analogWrite(ENB, 250); // Adjust speed using
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150); // Adjust speed for turning
  analogWrite(ENB, 150); // Adjust speed for turning
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150); // Adjust speed for turning
  analogWrite(ENB, 150); // Adjust speed for turning
}

void setup() {
  // Setup pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Servo setup
  myServo.attach(servoPin);
  myServo.write(90); // Start servo at center position

  Serial.begin(9600);
}

void loop() {
  // Scan forward
  myServo.write(90); // Center position (looking straight ahead)
  delay(500);
  int frontDistance = getDistance();
