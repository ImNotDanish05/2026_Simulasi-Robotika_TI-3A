#include <ESP32Servo.h>

// --- PIN MOTOR DRIVER L298N KE ESP32 ---
const int ENA = 13; // D13 - PWM Kecepatan Motor Kiri
const int IN1 = 14; // D14 - Arah Motor Kiri
const int IN2 = 27; // D27 - Arah Motor Kiri

const int IN3 = 26; // D26 - Arah Motor Kanan
const int IN4 = 25; // D25 - Arah Motor Kanan
const int ENB = 33; // D33 - PWM Kecepatan Motor Kanan

// --- PIN SENSOR ULTRASONIK (HC-SR04) ---
const int TRIG_PIN = 18; // D18
const int ECHO_PIN = 19; // D19 (Gunakan pembagi tegangan 1kΩ & 2kΩ)

// Sensor 2 (kanan) - TRIG pakai pin bekas servo
const int TRIG_PIN_2 = 4;  // D4 (dulu SERVO_PIN)
const int ECHO_PIN_2 = 34; // D34 (input only, pin baru)
 
// Sensor 3 (kiri) - pin baru
const int TRIG_PIN_3 = 32; // D32
const int ECHO_PIN_3 = 35; // D35 (input only, pin baru)
 


// --- PIN SERVO ---
const int SERVO_PIN = 4; // D4

// --- PARAMETER PENGATURAN ---
const int JARAK_AMAN = 25;       // Batas deteksi halangan (dalam cm)
const int WAKTU_BELOK_90 = 500;   // Durasi (ms) agar robot berbelok ~90 derajat
const int WAKTU_PUTAR_180 = 500; // Durasi (ms) agar robot putar balik ~180 derajat

// --- KECEPATAN BELOK MOTOR ---
const int BELOK_KIRI = 180; // Kecepatan motor saat berbelok (0-255)
const int BELOK_KANAN = 180; // Kecepatan motor saat berbelok (0-255)
const int PUTAR_BALIK = 255; // Kecepatan motor saat putar balik (0-255)
    
Servo myServo;

void setup() {
  Serial.begin(115200);

  // Pin motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Pin sensor ultrasonik
  pinMode(TRIG_DEPAN, OUTPUT);
  pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KANAN, OUTPUT);
  pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);
  pinMode(ECHO_KIRI, INPUT);

  berhenti();
  delay(2000); // Jeda awal 2 detik sebelum mobil bergerak
}

void loop() {
  int jarakDepan = bacaJarak(TRIG_DEPAN, ECHO_DEPAN);
  Serial.print("Depan: ");
  Serial.print(jarakDepan);
  Serial.println(" cm");

  // Depan aman -> maju terus
  if (jarakDepan > JARAK_AMAN) {
    maju(KECEPATAN_MAJU);
    return;
  }

  // Depan terhalang -> berhenti lalu cek kanan & kiri
  berhenti();
  delay(300);

  int jarakKanan = bacaJarak(TRIG_KANAN, ECHO_KANAN);
  delay(30);
  int jarakKiri = bacaJarak(TRIG_KIRI, ECHO_KIRI);

  Serial.print("Kanan: ");
  Serial.print(jarakKanan);
  Serial.print(" cm | Kiri: ");
  Serial.print(jarakKiri);
  Serial.println(" cm");

  bool kananAda = (jarakKanan <= JARAK_AMAN); // true = ada tembok di kanan
  bool kiriAda  = (jarakKiri  <= JARAK_AMAN); // true = ada tembok di kiri

  if (!kananAda) {
    // Kanan kosong -> belok kanan
    belokKanan(BELOK_KANAN);
    delay(WAKTU_BELOK_90);
    berhenti();
    delay(300);
  }
  else if (!kiriAda) {
    // Kanan ada tembok, kiri kosong -> belok kiri
    belokKiri(BELOK_KIRI);
    delay(WAKTU_BELOK_90);
    berhenti();
    delay(300);
  }
  else {
    // DEPAN, KANAN, KIRI semuanya terhalang
    Serial.println("Buntu! Mundur sampai salah satu sisi kosong...");
    mundurSampaiAdaCelah();

    // Setelah mundur, baca ulang kanan & kiri untuk menentukan arah
    jarakKanan = bacaJarak(TRIG_KANAN, ECHO_KANAN);
    delay(30);
    jarakKiri = bacaJarak(TRIG_KIRI, ECHO_KIRI);

    kananAda = (jarakKanan <= JARAK_AMAN);
    kiriAda  = (jarakKiri  <= JARAK_AMAN);

    if (kiriAda && !kananAda) {
      // Tembok kiri ada -> belok kanan
      Serial.println("Setelah mundur: tembok kiri ada -> belok kanan");
      belokKanan(BELOK_KANAN);
      delay(WAKTU_BELOK_90);
    }
    else if (kananAda && !kiriAda) {
      // Tembok kanan ada -> belok kiri
      Serial.println("Setelah mundur: tembok kanan ada -> belok kiri");
      belokKiri(BELOK_KIRI);
      delay(WAKTU_BELOK_90);
    }
    else {
      // Kanan & kiri sama-sama kosong (atau timeout mundur) -> putar balik
      Serial.println("Setelah mundur: kanan & kiri kosong -> putar balik");
      putarBalik(PUTAR_BALIK);
      delay(WAKTU_PUTAR_180);
    }

    berhenti();
    delay(300);
  }
}

// ==========================================
//        FUNGSI MUNDUR SAMPAI ADA CELAH
// ==========================================
// Mundur terus selama kanan DAN kiri masih ada tembok.
// Berhenti begitu salah satu sisi kosong, atau kalau sudah melewati batas waktu.
void mundurSampaiAdaCelah() {
  unsigned long mulai = millis();

  while (millis() - mulai < MAX_WAKTU_MUNDUR) {
    mundur(KECEPATAN_MUNDUR);

    int jarakKanan = bacaJarak(TRIG_KANAN, ECHO_KANAN);
    delay(30);
    int jarakKiri = bacaJarak(TRIG_KIRI, ECHO_KIRI);

    bool kananKosong = (jarakKanan > JARAK_AMAN);
    bool kiriKosong  = (jarakKiri  > JARAK_AMAN);

    if (kananKosong || kiriKosong) {
      break;
    }
  }

  berhenti();
  delay(300);
}

// ==========================================
//          FUNGSI SENSOR ULTRASONIK
// ==========================================
int bacaJarak(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout 30ms (~5 meter)
  long durasi = pulseIn(echoPin, HIGH, 30000);

  if (durasi == 0) {
    return 400; // Tidak ada pantulan -> anggap sangat jauh
  }

  int jarak = durasi * 0.034 / 2;
  return jarak;
}

// ==========================================
//           FUNGSI KONTROL MOTOR
// ==========================================
void maju(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void mundur(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void belokKiri(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void belokKanan(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void putarBalik(int speed) {
  belokKanan(speed); // Berputar di tempat ke arah kanan
}

void berhenti() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}