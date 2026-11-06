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

  // Konfigurasi pin motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Konfigurasi pin sensor ultrasonik
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inisialisasi Servo ke posisi depan (90 derajat)
  myServo.attach(SERVO_PIN);
  myServo.write(90);

  berhenti();
  delay(2000); // Jeda awal 2 detik sebelum mobil bergerak
}

void loop() {
  // Pastikan servo selalu menghadap ke depan saat bergerak
  myServo.write(90);
  
  int jarakDepan = bacaJarak();
  Serial.print("Jarak Depan: ");
  Serial.print(jarakDepan);
  Serial.println(" cm");

  if (jarakDepan > JARAK_AMAN) {
    // Jalur depan aman -> Mobil otomatis maju
    maju(255);
  } else {
    // 1. ADA HALANGAN DI DEPAN -> Mobil Berhenti
    berhenti();
    delay(300);

    // 2. Servo tengok ke kanan (0 derajat)
    myServo.write(0);
    delay(500); // Beri waktu servo berputar
    int jarakKanan = bacaJarak();
    Serial.print("Jarak Kanan: ");
    Serial.print(jarakKanan);
    Serial.println(" cm");

    if (jarakKanan > JARAK_AMAN) {
      // Kanan Aman -> Belok Kanan lalu Servo kembali ke Depan (90)
      belokKanan(BELOK_KANAN);
      delay(WAKTU_BELOK_90);
      berhenti();

      myServo.write(90);
      delay(300);
    } 
    else {
      // 3. Kanan Ada Halangan -> Servo tengok ke kiri (180 derajat)
      myServo.write(180);
      delay(500);
      int jarakKiri = bacaJarak();
      Serial.print("Jarak Kiri: ");
      Serial.print(jarakKiri);
      Serial.println(" cm");

      if (jarakKiri > JARAK_AMAN) {
        // Kiri Aman -> Belok Kiri lalu Servo kembali ke Depan (90)
        belokKiri(BELOK_KIRI);
        delay(WAKTU_BELOK_90);
        berhenti();

        myServo.write(90);
        delay(300);
      } 
      else {
        // 4. Kanan & Kiri Terhalang -> Putar Balik
        myServo.write(90); // Servo kembali menghadap depan
        delay(300);

        putarBalik(PUTAR_BALIK);
        delay(WAKTU_PUTAR_180);
        berhenti();
        delay(300);

        // Setelah putar balik, loop() otomatis akan mengukur jarak depan lagi.
        // Jika depan aman -> Maju. Jika masih ada halangan -> Ulangi siklus pengecekan.
      }
    }
  }
}

// ==========================================
//          FUNGSI SENSOR ULTRASONIK
// ==========================================
int bacaJarak() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Menerima pantulan gelombang (timeout 30ms ~ 5 meter)
  long durasi = pulseIn(ECHO_PIN, HIGH, 30000); 
  
  if (durasi == 0) {
    return 400; // Jika tidak ada pantulan, anggap jarak sangat jauh
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