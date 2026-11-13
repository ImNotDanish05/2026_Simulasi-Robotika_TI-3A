// --- PIN MOTOR DRIVER L298N KE ESP32 ---
const int ENA = 13; // D13 - PWM Kecepatan Motor Kiri
const int IN1 = 14; // D14 - Arah Motor Kiri
const int IN2 = 27; // D27 - Arah Motor Kiri

const int IN3 = 26; // D26 - Arah Motor Kanan
const int IN4 = 25; // D25 - Arah Motor Kanan
const int ENB = 33; // D33 - PWM Kecepatan Motor Kanan

// --- PIN SENSOR GARIS ---
const int SENSOR_KIRI  = 22; // D22
const int SENSOR_KANAN = 23; // D23

// --- PENGATURAN SENSOR ---
// true  = sensor membaca HIGH saat di atas garis
// false = sensor membaca LOW saat di atas garis
const bool GARIS_TERBACA_HIGH = true;

// --- PENGATURAN KECEPATAN ---
const int BASE_SPEED = 150;  // Kecepatan lurus
const int MAX_SPEED  = 200;  // Batas atas PWM tiap motor
const int MIN_SPEED  = 0;    // Batas bawah (0 = motor dalam berhenti, tidak mundur)

// --- PARAMETER PID ---
float Kp = 100;   // Koreksi dasar
float Ki = 3;     // Koreksi bertambah selama garis masih menyimpang
float Kd = 30;    // Peredam overshoot

const float INTEGRAL_MAX = 30;       // Batas integral (anti-windup)
const unsigned long LOOP_MS = 10;    // Loop tetap 10 ms supaya I & D konsisten

const bool DEBUG = false; // true = tampilkan status di Serial Monitor

// --- VARIABEL PID ---
float error     = 0;
float lastError = 0;
float integral  = 0;
unsigned long lastTime = 0;

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

void loop() {
  // Jalankan tiap LOOP_MS saja
  if (millis() - lastTime < LOOP_MS) return;
  lastTime = millis();

  bool kiriGaris  = terdeteksiGaris(SENSOR_KIRI);
  bool kananGaris = terdeteksiGaris(SENSOR_KANAN);

  // Error:
  //  -1 = sensor kiri kena garis  -> belok kiri
  //   0 = tengah / dua-duanya kena (persimpangan) -> lurus
  //  +1 = sensor kanan kena garis -> belok kanan
  error = (float)kananGaris - (float)kiriGaris;

  // --- Integral (reset kalau sudah di tengah) ---
  if (error == 0) {
    integral = 0;
  } else {
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  // --- Derivative ---
  float derivative = error - lastError;
  lastError = error;

  // --- Output PID ---
  // output positif = belok kanan, negatif = belok kiri
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Belok kanan -> motor kiri lebih cepat, motor kanan lebih lambat
  int kecKiri  = constrain((int)(BASE_SPEED + output), MIN_SPEED, MAX_SPEED);
  int kecKanan = constrain((int)(BASE_SPEED - output), MIN_SPEED, MAX_SPEED);

  gerak(kecKiri, kecKanan);

  if (DEBUG) {
    Serial.printf("Kiri:%d Kanan:%d | err:%.1f int:%.1f out:%.1f | L:%d R:%d\n",
                  kiriGaris, kananGaris, error, integral, output, kecKiri, kecKanan);
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