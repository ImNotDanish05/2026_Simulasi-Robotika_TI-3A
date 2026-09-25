#include <Servo.h>

// PASTIKAN nomor pin ini sama dengan pin yang kamu colok di Arduino
const int pingPin = 7;   // Pin SIG ultrasonik
const int servoPin = 9;  // Pin Signal servo

Servo myServo;

// Fungsi pembaca jarak standar Tinkercad
long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  // Tembak sinyal trigger 10 us
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // Dengar pantulan sinyal
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  
  // Posisi awal dipaksa ke 0 agar terlihat pergerakannya saat start
  myServo.write(0);
  delay(500);

  Serial.println("=== SYSTEM READY ===");
}

void loop() {
  // 0.01723 = kecepatan suara (cm/us) dibagi 2
  long duration = readUltrasonicDistance(pingPin, pingPin);
  long x = duration * 0.01723;

  Serial.print("Jarak (X): ");
  Serial.print(x);
  Serial.print(" cm -> ");

  if (x < 100) {
    myServo.write(180);
    Serial.println("Servo: 180 Derajat");
  } 
  else if (x >= 100 && x <= 200) {
    myServo.write(90);
    Serial.println("Servo: 90 Derajat");
  } 
  else {
    myServo.write(45);
    Serial.println("Servo: 45 Derajat");
  }

  delay(150);
}