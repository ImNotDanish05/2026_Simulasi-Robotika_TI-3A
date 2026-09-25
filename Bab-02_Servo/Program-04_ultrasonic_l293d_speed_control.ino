// Pin Ultrasonik
const int pingPin = 12;

// Pin PWM Kecepatan (Enable L293D)
const int enA = 9;
const int enB = 10;

// Pin Kontrol Arah Motor
const int in1 = 2;
const int in2 = 3;
const int in3 = 4;
const int in4 = 5;

// Fungsi pembaca jarak sensor ultrasonik 3-pin
long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set arah putaran kedua motor ke depan (Maju)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  Serial.println("=== SMART MOTOR SPEED READY ===");
}

void loop() {
  long duration = readUltrasonicDistance(pingPin, pingPin);
  long x = duration * 0.01723; // Jarak dalam cm

  int motorSpeed = 0;

  if (x < 200) {
    // Jarak dekat: Lambat (PWM 130 dari max 255)
    motorSpeed = 130;
    Serial.print("Jarak: ");
    Serial.print(x);
    Serial.println(" cm -> Mode LAMBAT (PWM: 130)");
  } 
  else {
    // Jarak jauh: Cepat (PWM 255 / Kecepatan Penuh)
    motorSpeed = 255;
    Serial.print("Jarak: ");
    Serial.print(x);
    Serial.println(" cm -> Mode CEPAT (PWM: 255)");
  }

  // Terapkan kecepatan ke kedua motor
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  delay(150);
}