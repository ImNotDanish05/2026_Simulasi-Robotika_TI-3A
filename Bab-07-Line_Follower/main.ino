// --- PIN MOTOR DRIVER L298N KE ESP32 ---
const int ENA = 13; // D13 - PWM Kecepatan Motor Kiri
const int IN1 = 14; // D14 - Arah Motor Kiri
const int IN2 = 27; // D27 - Arah Motor Kiri

const int IN3 = 26; // D26 - Arah Motor Kanan
const int IN4 = 25; // D25 - Arah Motor Kanan
const int ENB = 33; // D33 - PWM Kecepatan Motor Kanan

// --- PIN SENSOR GARIS ---
const int SENSOR_KIRI   = 22; // D22
const int SENSOR_KANAN  = 23; // D23
// const int SENSOR_TENGAH = 35; // D35 - Sensor Tengah (input only)

// --- PENGATURAN SENSOR ---
// true  = sensor membaca HIGH saat di atas garis
// false = sensor membaca LOW saat di atas garis
const bool GARIS_TERBACA_HIGH = true;

// --- KECEPATAN MOTOR (0-255, negatif = mundur) ---
const int KECEPATAN_MAJU   = 150; // Kecepatan lurus
const int BELOK_LUAR       = 150; // Motor sisi luar saat belok
const int BELOK_DALAM      = 0; // Motor sisi dalam saat belok (negatif = mundur biar tajam)

const bool DEBUG = false; // true = tampilkan status sensor di Serial Monitor

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
  // pinMode(SENSOR_TENGAH, INPUT);
  pinMode(SENSOR_KANAN, INPUT);

  berhenti();
  delay(2000); // Jeda awal 2 detik sebelum robot bergerak
}

void loop() {
  bool kiriGaris  = terdeteksiGaris(SENSOR_KIRI);
  bool kananGaris = terdeteksiGaris(SENSOR_KANAN);

  if (DEBUG) {
    Serial.printf("Kiri: %d | Kanan: %d\n", kiriGaris, kananGaris);
  }

  if (!kiriGaris && !kananGaris) {
    // Garis ada di tengah -> lurus
    gerak(KECEPATAN_MAJU, KECEPATAN_MAJU);
  }
  else if (kiriGaris && !kananGaris) {
    // Garis menyimpang ke kiri -> belok kiri
    gerak(BELOK_DALAM, BELOK_LUAR);
  }
  else if (!kiriGaris && kananGaris) {
    // Garis menyimpang ke kanan -> belok kanan
    gerak(BELOK_LUAR, BELOK_DALAM);
  }
  else {
    // Dua-duanya kena garis (persimpangan / garis tebal) -> lurus pelan
    gerak(KECEPATAN_MAJU, KECEPATAN_MAJU);
  }
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