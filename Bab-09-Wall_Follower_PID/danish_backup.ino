// --- PIN SENSOR ULTRASONIK (HC-SR04) ---
const int TRIG_DEPAN = 22;
const int ECHO_DEPAN = 23;

const int TRIG_KIRI  = 18;
const int ECHO_KIRI  = 19;

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

// --- PILIHAN MODE BELOK KIRI ---
// true  = Belok kiri otomatis ditarik oleh PID (melengkung tanpa berhenti)
// false = Belok kiri menggunakan manuver putar 90 derajat (belok90Kiri)
const bool belokKiriPakaiPID = true;

// --- KONSTANTA PID SUSUR DINDING KIRI ---
float Kp = 30.0;
float Ki = 0.0;
float Kd = 150.0;

// --- PENGATURAN KECEPATAN & DURASI BELOK ---
int baseSpeed        = 560;
int maxSpeed         = 1023;

// PENGATURAN ANTI BERPUTAR-PUTAR (MODE PID):
// Kecepatan minimal roda kiri saat PID belok kiri (jangan 0 agar robot tetap maju masuk lorong!)
int minPidLeftSpeed  = 400; 
// Batas maksimal jarak kiri yang dihitung PID agar tarikan belok tidak terlalu ekstrem
const float MAX_JARAK_PID = 100.0; 

// PENGATURAN MODE 90 DERAJAT (jika belokKiriPakaiPID = false):
int preTurnSpeed     = 480;
int preTurnLeftMs    = 400;
int preTurnRightMs   = 0;

int turnLeftSpeed    = 500;
int turnLeftMs       = 350;
int postTurnLeftMs   = 550; // Dinaikkan agar badan robot masuk penuh ke lorong baru

int turnRightSpeed   = 800;
int turnRightMs      = 300;

// --- BATAS JARAK (CM) ---
const float BATAS_DEPAN       = 18.0;
const float BATAS_KIRI_ADA    = 30.0;
const float JARAK_TARGET_KIRI = 15.0;

// Variabel PID
float error = 0;
float lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_DEPAN, OUTPUT);
  pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);
  pinMode(ECHO_KIRI, INPUT);

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
  // 1. Baca jarak kedua sensor (dalam cm)
  float jarakDepan = bacaUltrasonik(TRIG_DEPAN, ECHO_DEPAN);
  delay(10);
  float jarakKiri  = bacaUltrasonik(TRIG_KIRI, ECHO_KIRI);

  // --- KONDISI 1: SENSOR DEPAN MENDETEKSI OBSTACLE ---
  if (jarakDepan < BATAS_DEPAN) {
    if (jarakKiri <= BATAS_KIRI_ADA) {
      belok90Kanan();
    } else {
      if (!belokKiriPakaiPID) {
        belok90Kiri();
      } else {
        setMotor(0, 0);
        delay(100);
        setMotor(-turnLeftSpeed, turnLeftSpeed);
        delay(turnLeftMs);
        setMotor(0, 0);
        delay(100);
        // Dorong maju singkat setelah putar agar sensor kiri menangkap dinding baru
        setMotor(baseSpeed, baseSpeed);
        delay(250);
        resetPID();
      }
    }
    return;
  }

  // --- KONDISI 2: SENSOR KIRI KOSONG MLOMPONG (HANYA MODE 90 DERAJAT) ---
  if (!belokKiriPakaiPID && jarakKiri > BATAS_KIRI_ADA) {
    belok90Kiri();
    return;
  }

  // --- KONDISI 3: SUSUR DINDING KIRI & BELOK KIRI VIA PID ---
  // Batasi pembacaan jarak kiri maksimal 45 cm di dalam rumus PID
  // supaya saat tikungan siku, error tidak melompat ke 85 dan membuat robot berputar di tempat
  float jarakKiriTerbatas = min(jarakKiri, MAX_JARAK_PID);
  error = JARAK_TARGET_KIRI - jarakKiriTerbatas;

  integral += error;
  integral = constrain(integral, -100, 100);
  float derivative = error - lastError;

  float pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Roda kiri dibatasi minimal minPidLeftSpeed (220), bukan 0, agar membentuk busur belok ke dalam lorong
  int leftMotorSpeed  = constrain(baseSpeed + pidValue, minPidLeftSpeed, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed - pidValue, 0, maxSpeed);

  setMotor(leftMotorSpeed, rightMotorSpeed);
  delay(15);
}

// --- FUNGSI BACA JARAK ULTRASONIK (CM) ---
float bacaUltrasonik(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long durasi = pulseIn(echoPin, HIGH, 25000);
  if (durasi == 0) {
    return 100.0;
  }
  return durasi * 0.0343 / 2.0;
}

// --- MANUVER BELOK KIRI 90 DERAJAT ---
void belok90Kiri() {
  setMotor(preTurnSpeed, preTurnSpeed);
  delay(preTurnLeftMs);

  setMotor(0, 0);
  delay(100);

  setMotor(-turnLeftSpeed, turnLeftSpeed);
  delay(turnLeftMs);

  setMotor(0, 0);
  delay(100);

  setMotor(baseSpeed, baseSpeed);
  delay(postTurnLeftMs);

  resetPID();
}

// --- MANUVER BELOK KANAN 90 DERAJAT ---
void belok90Kanan() {
  setMotor(preTurnSpeed, preTurnSpeed);
  delay(preTurnRightMs);

  setMotor(0, 0);
  delay(100);

  setMotor(turnRightSpeed, -turnRightSpeed);
  delay(turnRightMs);

  setMotor(0, 0);
  delay(100);

  resetPID();
}

void resetPID() {
  error = 0;
  lastError = 0;
  integral = 0;
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