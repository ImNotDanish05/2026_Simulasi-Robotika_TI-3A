#include <Servo.h>

const int button1 = 2;
const int button2 = 3;
const int button3 = 4;
const int servoPin = 9;

Servo myServo;

void setup() {
  Serial.begin(9600);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  myServo.attach(servoPin);
  myServo.write(0); // Posisi awal servo saat dinyalakan

  Serial.println("=== SERVO & BUTTON READY ===");
}

void loop() {
  int state1 = digitalRead(button1);
  int state2 = digitalRead(button2);
  int state3 = digitalRead(button3);

  // Deteksi logika LOW (0) saat tombol ditekan
  if (state1 == LOW) {
    myServo.write(0);
    Serial.println("Button 1 -> 0 Derajat");
    delay(200); // Debounce sederhana agar tidak spam instruksi
  } 
  else if (state2 == LOW) {
    myServo.write(90);
    Serial.println("Button 2 -> 90 Derajat");
    delay(200);
  } 
  else if (state3 == LOW) {
    myServo.write(180);
    Serial.println("Button 3 -> 180 Derajat");
    delay(200);
  }
}