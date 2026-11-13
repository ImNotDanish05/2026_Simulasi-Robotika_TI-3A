// --- PIN MOTOR DRIVER L298N KE ESP32 ---
const int ENA = 13; // PWM Kecepatan Motor Kiri
const int IN1 = 14; // Arah Motor Kiri
const int IN2 = 27; // Arah Motor Kiri

const int IN3 = 26; // Arah Motor Kanan
const int IN4 = 25; // Arah Motor Kanan
const int ENB = 33; // PWM Kecepatan Motor Kanan

// --- PIN SENSOR GARIS ---
const int SENSOR_KIRI  = 22;
const int SENSOR_KANAN = 23;

// --- PENGATURAN SENSOR ---
// true  = sensor membaca HIGH saat di atas garis
// false = sensor membaca LOW saat di atas garis
const bool GARIS_TERBACA_HIGH = true;

// --- KECEPATAN MOTOR (0-255, negatif = mundur) ---
const int KECEPATAN_MAJU = 120;  // turunkan dulu, naikkan setelah stabil
const int KOREKSI_AWAL   = 25;   // koreksi awal (lembut)
const int KOREKSI_MAKS   = 140;  // batas koreksi untuk tikungan tajam
const int KOREKSI_NAIK   = 6;    // penambahan koreksi tiap langkah
const int INTERVAL_MS    = 10;   // seberapa cepat koreksi naik

const bool DEBUG = false; // true = tampilkan status sensor di Serial Monitor

int koreksi = 0;
unsigned long waktuTerakhir = 0;

// ==========================================
//                 SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor
  pinMode(SENSOR_KIRI, INPUT);
  pinMode(SENSOR_KANAN, INPUT);

  berhenti();
  delay(2000); // Jeda awal 2 detik sebelum robot bergerak
}

// ==========================================
//                  LOOP
// ==========================================
void loop() {
  bool kiriGaris  = terdeteksiGaris(SENSOR_KIRI);
  bool kananGaris = terdeteksiGaris(SENSOR_KANAN);

  if (DEBUG) {
    Serial.printf("Kiri: %d | Kanan: %d | Koreksi: %d\n", kiriGaris, kananGaris, koreksi);
  }

  if (kiriGaris == kananGaris) {
    // Tengah (dua-duanya tidak kena) atau persimpangan -> lurus
    koreksi = 0;
    gerak(KECEPATAN_MAJU, KECEPATAN_MAJU);
    return;
  }

  // Salah satu sensor kena garis -> koreksi naik bertahap
  if (millis() - waktuTerakhir >= INTERVAL_MS) {
    waktuTerakhir = millis();
    if (koreksi == 0) koreksi = KOREKSI_AWAL;
    else koreksi = min(koreksi + KOREKSI_NAIK, KOREKSI_MAKS);
  }

  // Roda dalam diperlambat, roda luar sedikit dipercepat
  int dalam = KECEPATAN_MAJU - koreksi;
  int luar  = KECEPATAN_MAJU + koreksi / 3;

  if (kiriGaris) gerak(dalam, luar);   // belok kiri
  else           gerak(luar, dalam);   // belok kanan
}

// ==========================================
//            FUNGSI SENSOR GARIS
// ==========================================
bool terdeteksiGaris(int pin) {
  int nilai = digitalRead(pin);
  return GARIS_TERBACA_HIGH ? (nilai == HIGH) : (nilai == LOW);
}

// ==========================================
//            FUNGSI KONTROL MOTOR
// ==========================================
// kecepatan: -255 s/d 255 (positif = maju, negatif = mundur, 0 = berhenti)
void gerak(int kiri, int kanan) {
  kiri  = constrain(kiri, -255, 255);
  kanan = constrain(kanan, -255, 255);

  // Motor kiri
  if (kiri > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (kiri < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(kiri));

  // Motor kanan
  if (kanan > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (kanan < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(kanan));
}

void berhenti() {
  gerak(0, 0);
}