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

// --- KONSTANTA PID SUSUR DINDING KIRI ---
float Kp = 25.0;
float Ki = 0.0;
float Kd = 40.0;

// --- PENGATURAN KECEPATAN & JARAK (CM) ---
int baseSpeed    = 560;
int maxSpeed     = 1023;
int turnSpeed    = 700;
int advanceSpeed = 480; // Kecepatan pelan saat maju memposisikan bodi sebelum berhenti

const float BATAS_DEPAN       = 18.0; // Dinaikkan sedikit (cm) agar aman saat maju sedikit sebelum stop
const float BATAS_KIRI_ADA    = 25.0; // Jarak maksimal (cm) dinding kiri dianggap masih ada
const float JARAK_TARGET_KIRI = 12.0; // Jarak ideal (cm) robot dari dinding kiri saat lurus

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
  delay(10); // Jeda singkat mencegah interferensi gelombang suara antar sensor
  float jarakKiri  = bacaUltrasonik(TRIG_KIRI, ECHO_KIRI);

  // --- KONDISI 1: SENSOR DEPAN MENDETEKSI OBSTACLE ---
  if (jarakDepan <= BATAS_DEPAN) {
    // Maju sedikit dengan pelan untuk memposisikan robot sebelum berhenti
    // Sesuaikan delay(120) dan BATAS_DEPAN agar hidung robot tidak menyentuh tembok depan
    setMotor(advanceSpeed, advanceSpeed);
    delay(120);

    // Berhenti sebelum belok
    setMotor(0, 0);
    delay(200);

    // Cek sensor kiri apakah ada obstacle
    if (jarakKiri <= BATAS_KIRI_ADA) {
      belokKanan();
    } else {
      belokKiri();
    }
    resetPID();
    return;
  }

  // --- KONDISI 2: SENSOR KIRI KOSONG MLOMPONG ---
  if (jarakKiri > BATAS_KIRI_ADA) {
    // Jalan maju sedikit agar badan/poros roda melewati sudut tembok kiri sebelum berhenti
    // Sesuaikan delay(220) dengan panjang bodi robot Anda
    setMotor(advanceSpeed, advanceSpeed);
    delay(220);

    // Berhenti sebelum belok
    setMotor(0, 0);
    delay(200);

    belokKiri();
    resetPID();
    return;
  }

  // --- KONDISI 3: SUSUR DINDING KIRI (PID) ---
  error = JARAK_TARGET_KIRI - jarakKiri;

  integral += error;
  integral = constrain(integral, -100, 100);
  float derivative = error - lastError;

  float pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftMotorSpeed  = constrain(baseSpeed + pidValue, -maxSpeed, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed - pidValue, -maxSpeed, maxSpeed);

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
    return 999.0;
  }
  return durasi * 0.0343 / 2.0;
}

// --- MANUVER BELOK ---
void belokKiri() {
  // Putar ke kiri di tempat (motor kiri mundur, motor kanan maju)
  setMotor(-turnSpeed, turnSpeed);
  delay(350); // Sesuaikan durasi agar pas 90 derajat

  setMotor(0, 0);
  delay(150);

  // Maju sedikit setelah belok agar sensor kiri langsung menangkap dinding baru
  setMotor(baseSpeed, baseSpeed);
  delay(250);
}

void belokKanan() {
  // Putar ke kanan di tempat (motor kiri maju, motor kanan mundur)
  setMotor(turnSpeed, -turnSpeed);
  delay(350); // Sesuaikan durasi agar pas 90 derajat

  setMotor(0, 0);
  delay(150);
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