// ============================================================
//  LINE FOLLOWER 3 SENSOR + PID  |  ESP32 + L298N
// ============================================================

// --- Pin sensor ---
const int SENSOR_KIRI   = 22;
const int SENSOR_TENGAH = 18;
const int SENSOR_KANAN  = 23;

// --- Pin motor driver L298N ---
const int ENA = 13; // PWM kecepatan motor kiri
const int IN1 = 14; // Arah motor kiri
const int IN2 = 27; // Arah motor kiri
const int IN3 = 26; // Arah motor kanan
const int IN4 = 25; // Arah motor kanan
const int ENB = 33; // PWM kecepatan motor kanan

// --- Konfigurasi sensor ---
// Isi dengan HIGH jika sensor mengeluarkan HIGH saat mendeteksi garis.
// Ubah ke LOW jika modul sensor kamu kebalikannya.
#define LINE_TERDETEKSI HIGH

// --- Parameter PWM ---
const int PWM_FREQ = 5000;  // Hz
const int PWM_RES  = 8;     // 8 bit -> 0..255

// --- Parameter PID (SESUAIKAN / TUNING) ---
float Kp = 35.0;
float Ki = 0.0;     // biasanya 0 untuk line follower sederhana
float Kd = 20.0;

// --- Kecepatan (0..255) ---
int baseSpeed   = 150;   // kecepatan dasar saat lurus
int maxSpeed    = 255;
int minSpeed    = -120;  // negatif = motor mundur (membantu tikungan tajam)
int searchSpeed = 130;   // kecepatan berputar saat garis hilang

// --- Timing loop ---
const unsigned long LOOP_MS = 5;

// --- Variabel PID ---
float lastError = 0;
float integral  = 0;
unsigned long lastTime = 0;

// ============================================================
//  PWM (kompatibel ESP32 Arduino core 2.x dan 3.x)
// ============================================================
void pwmSetup() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
#endif
}

void pwmWrite(int channelIndex, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(channelIndex == 0 ? ENA : ENB, duty);
#else
  ledcWrite(channelIndex, duty);
#endif
}

// ============================================================
//  KONTROL MOTOR
//  speed: -255..255 (positif = maju, negatif = mundur)
// ============================================================
void setMotor(int kiri, int kanan) {
  kiri  = constrain(kiri,  minSpeed, maxSpeed);
  kanan = constrain(kanan, minSpeed, maxSpeed);

  // Motor kiri
  if (kiri >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  pwmWrite(0, abs(kiri));

  // Motor kanan
  if (kanan >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  pwmWrite(1, abs(kanan));
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  pwmWrite(0, 0);
  pwmWrite(1, 0);
}

// ============================================================
//  BACA SENSOR -> ERROR
//  Negatif = garis di kiri, positif = garis di kanan
//  Return false jika garis tidak terdeteksi sama sekali
//
//   Kiri Tengah Kanan | Error
//     1     0     0   |  -2
//     1     1     0   |  -1
//     0     1     0   |   0
//     0     1     1   |  +1
//     0     0     1   |  +2
//     1     1     1   |   0  (persimpangan / garis lebar)
//     0     0     0   |  garis hilang
// ============================================================
bool bacaError(float &error) {
  bool kiri   = (digitalRead(SENSOR_KIRI)   == LINE_TERDETEKSI);
  bool tengah = (digitalRead(SENSOR_TENGAH) == LINE_TERDETEKSI);
  bool kanan  = (digitalRead(SENSOR_KANAN)  == LINE_TERDETEKSI);

  if (!kiri && !tengah && !kanan) return false;

  if      ( kiri && !tengah && !kanan) error = -2;
  else if ( kiri &&  tengah && !kanan) error = -1;
  else if (!kiri &&  tengah && !kanan) error =  0;
  else if (!kiri &&  tengah &&  kanan) error =  1;
  else if (!kiri && !tengah &&  kanan) error =  2;
  else                                  error =  0; // 111 atau 101

  return true;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_KIRI,   INPUT);
  pinMode(SENSOR_TENGAH, INPUT);
  pinMode(SENSOR_KANAN,  INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pwmSetup();
  stopMotor();

  delay(2000); // waktu untuk menaruh robot di lintasan
  lastTime = millis();
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  unsigned long now = millis();
  if (now - lastTime < LOOP_MS) return;

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float error;

  if (bacaError(error)) {
    // --- Hitung PID ---
    integral += error * dt;
    integral = constrain(integral, -50, 50);        // anti-windup

    float derivative = (error - lastError) / dt;
    float koreksi = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // error positif (garis di kanan) -> belok kanan
    // -> motor kiri lebih cepat, motor kanan lebih lambat
    int speedKiri  = baseSpeed + (int)koreksi;
    int speedKanan = baseSpeed - (int)koreksi;

    setMotor(speedKiri, speedKanan);

    // Debug (aktifkan saat tuning, matikan saat lomba agar loop cepat)
    // Serial.printf("err=%.1f koreksi=%.1f L=%d R=%d\n", error, koreksi, speedKiri, speedKanan);
  }
  else {
    // --- Garis hilang: berputar ke arah terakhir garis terlihat ---
    integral = 0;
    if (lastError < 0) {
      setMotor(-searchSpeed, searchSpeed);   // putar kiri
    } else {
      setMotor(searchSpeed, -searchSpeed);   // putar kanan
    }
  }
}