// Pin sensor
const int SENSOR_KIRI   = 22;
const int SENSOR_TENGAH = 18;
const int SENSOR_KANAN  = 23;

// --- PIN MOTOR DRIVER L298N KE ESP32 ---
const int ENA = 13; // PWM Kecepatan Motor Kiri
const int IN1 = 14; // Arah Motor Kiri
const int IN2 = 27; // Arah Motor Kiri

const int IN3 = 26; // Arah Motor Kanan
const int IN4 = 25; // Arah Motor Kanan
const int ENB = 33; // PWM Kecepatan Motor Kanan

// --- PENGATURAN PWM ESP32 ---
const int PWM_FREQ = 1000;
const int PWM_RES  = 10;   // 0 - 1023
const int PWM_MAX  = 1023;

// --- KONSTANTA PID ---
float Kp = 250.0;
float Ki = 1;
float Kd = 600.0;

// --- PENGATURAN KECEPATAN (0 - 1023) ---
int baseSpeed = 560;
int maxSpeed  = 1023;
int turnSpeed = 800;  // Kecepatan saat putar balik

// Ubah ke LOW jika sensor mengeluarkan logika 0 saat mendeteksi garis hitam
const int GARIS = HIGH;

// Variabel perhitungan PID
int error = 0;
int lastError = 0;
int integral = 0;
int lastKnownDirection = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_KIRI, INPUT);
  pinMode(SENSOR_TENGAH, INPUT);
  pinMode(SENSOR_KANAN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  setMotor(0, 0);
  delay(2000);
}

void loop() {
  // 1. Baca sensor
  int sKiri   = (digitalRead(SENSOR_KIRI)   == GARIS) ? 1 : 0;
  int sTengah = (digitalRead(SENSOR_TENGAH) == GARIS) ? 1 : 0;
  int sKanan  = (digitalRead(SENSOR_KANAN)  == GARIS) ? 1 : 0;

  // --- KONDISI KHUSUS: KETIGA SENSOR DETEKSI HITAM ---
  if (sKiri == 1 && sTengah == 1 && sKanan == 1) {
    putarBalik();
    return; // Langsung ulangi loop() dari awal setelah putar balik selesai
  }

  // 2. Konversi sensor menjadi error
  if (sKiri == 1 && sTengah == 0 && sKanan == 0) {
    error = -2;
    lastKnownDirection = -1;
  } else if (sKiri == 1 && sTengah == 1 && sKanan == 0) {
    error = -1;
    lastKnownDirection = -1;
  } else if (sKiri == 0 && sTengah == 1 && sKanan == 0) {
    error = 0;
  } else if (sKiri == 0 && sTengah == 1 && sKanan == 1) {
    error = 1;
    lastKnownDirection = 1;
  } else if (sKiri == 0 && sTengah == 0 && sKanan == 1) {
    error = 2;
    lastKnownDirection = 1;
  } else if (sKiri == 0 && sTengah == 0 && sKanan == 0) {
    if (lastKnownDirection == -1)      error = -3;
    else if (lastKnownDirection == 1)  error = 3;
  }

  // 3. Kalkulasi PID
  integral += error;
  integral = constrain(integral, -100, 100);
  int derivative = error - lastError;

  float pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // 4. Kecepatan tiap motor
  int leftMotorSpeed  = baseSpeed + pidValue;
  int rightMotorSpeed = baseSpeed - pidValue;

  leftMotorSpeed  = constrain(leftMotorSpeed,  -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  // 5. Gerakkan motor
  setMotor(leftMotorSpeed, rightMotorSpeed);

  delay(10);
}

// Fungsi manuver berhenti dan putar balik 180 derajat
void putarBalik() {
  // 1. Berhenti sejenak
  setMotor(0, 0);
  delay(400);

  // 2. Mulai berputar di tempat (motor kiri maju, motor kanan mundur = putar kanan)
  // Jika ingin putar ke kiri, ubah menjadi setMotor(-turnSpeed, turnSpeed);
  setMotor(turnSpeed, -turnSpeed);

  // 3. Delay singkat agar sensor tengah keluar dari garis hitam yang sekarang
  // SESUAIKAN nilai delay ini jika robot langsung berhenti berputar sebelum berbalik arah
  delay(250);

  // 4. Terus berputar sampai sensor tengah kembali menemukan garis hitam
  while (digitalRead(SENSOR_TENGAH) != GARIS) {
    delay(5);
  }

  // 5. Rem sebentar untuk meredam momentum putaran agar tidak bablas
  setMotor(-turnSpeed / 2, turnSpeed / 2);
  delay(50);
  setMotor(0, 0);
  delay(200);

  // 6. Reset memori PID agar tidak kaget saat mulai jalan lagi
  error = 0;
  lastError = 0;
  integral = 0;
  lastKnownDirection = 0;
}

void setMotor(int speedKiri, int speedKanan) {
  // Motor Kiri
  if (speedKiri >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(ENA, constrain(abs(speedKiri), 0, PWM_MAX));

  // Motor Kanan
  if (speedKanan >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(ENB, constrain(abs(speedKanan), 0, PWM_MAX));
}