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

// --- KECEPATAN MOTOR (0-255) ---
const int KECEPATAN_MAJU = 120;  // kalau masih lepas di tikungan, coba 100
const int KOREKSI_AWAL   = 12;   // koreksi awal (lembut)
const int KOREKSI_MAKS   = 90;   // batas koreksi lembut
const int KOREKSI_NAIK   = 2;    // penambahan koreksi tiap langkah
const int INTERVAL_MS    = 20;   // jeda antar penambahan koreksi

// --- FILTER KEDIPAN SENSOR ---
const int TUNDA_DETEKSI_MS = 30; // sensor harus kena garis minimal sekian ms baru belok

// --- TIKUNGAN TAJAM ---
const int TAJAM_MS           = 120; // sensor kena garis selama ini -> dianggap tikungan tajam
const int BELOK_DALAM_TAJAM  = -80; // roda dalam saat tikungan tajam (negatif = mundur)

const bool DEBUG = false; // true = tampilkan status sensor di Serial Monitor

// --- VARIABEL KERJA ---
int koreksi = 0;
unsigned long waktuTerakhir = 0;
int arah = 0;                    // -1 = kiri, 1 = kanan, 0 = tidak ada
unsigned long mulaiDeteksi = 0;

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

  // 0 0 (garis kecil di tengah) atau 1 1 (persimpangan) -> lurus
  if (kiriGaris == kananGaris) {
    koreksi = 0;
    arah = 0;
    gerak(KECEPATAN_MAJU, KECEPATAN_MAJU);
    return;
  }

  // Salah satu sensor kena garis
  int arahBaru = kiriGaris ? -1 : 1;
  if (arahBaru != arah) {
    // Deteksi baru: catat waktu mulai
    arah = arahBaru;
    mulaiDeteksi = millis();
    koreksi = 0;
  }

  unsigned long lama = millis() - mulaiDeteksi;

  // Belum cukup lama terdeteksi -> anggap kedipan, tetap lurus
  if (lama < TUNDA_DETEKSI_MS) {
    gerak(KECEPATAN_MAJU, KECEPATAN_MAJU);
    return;
  }

  // Sensor kena garis cukup lama -> tikungan tajam, putar tegas
  if (lama >= TAJAM_MS) {
    if (kiriGaris) gerak(BELOK_DALAM_TAJAM, KECEPATAN_MAJU);  // putar kiri
    else           gerak(KECEPATAN_MAJU, BELOK_DALAM_TAJAM);  // putar kanan
    return;
  }

  // Selain itu -> koreksi lembut, naik bertahap
  if (millis() - waktuTerakhir >= INTERVAL_MS) {
    waktuTerakhir = millis();
    if (koreksi == 0) koreksi = KOREKSI_AWAL;
    else koreksi = min(koreksi + KOREKSI_NAIK, KOREKSI_MAKS);
  }

  int dalam = max(KECEPATAN_MAJU - koreksi, 0);  // roda dalam tidak mundur
  int luar  = KECEPATAN_MAJU;                    // roda luar tetap

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