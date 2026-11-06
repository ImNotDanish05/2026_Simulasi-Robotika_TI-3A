// --- PIN MOTOR DRIVER L298N KE ESP32 (tidak berubah) ---
  const int ENA = 13; // D13 - PWM Kecepatan Motor Kiri
  const int IN1 = 14; // D14 - Arah Motor Kiri
  const int IN2 = 27; // D27 - Arah Motor Kiri

  const int IN3 = 26; // D26 - Arah Motor Kanan
  const int IN4 = 25; // D25 - Arah Motor Kanan
  const int ENB = 33; // D33 - PWM Kecepatan Motor Kanan

  // --- PIN SENSOR ULTRASONIK (HC-SR04) ---
  // Semua pin ECHO pakai pembagi tegangan 1kΩ & 2kΩ

  // Sensor 1 (depan) - pin lama, tidak berubah
  const int TRIG_PIN_1 = 18; // D18
  const int ECHO_PIN_1 = 19; // D19

  // Sensor 2 (kanan) - TRIG pakai pin bekas servo
  const int TRIG_PIN_2 = 4;  // D4 (dulu SERVO_PIN)
  const int ECHO_PIN_2 = 34; // D34 (input only, pin baru)

  // Sensor 3 (kiri) - pin baru
  const int TRIG_PIN_3 = 32; // D32
  const int ECHO_PIN_3 = 35; // D35 (input only, pin baru)

  // --- PARAMETER PENGATURAN ---
  const int JARAK_AMAN = 25;        // Batas deteksi halangan (dalam cm)
  const int WAKTU_BELOK_90 = 500;   // Durasi (ms) agar robot berbelok ~90 derajat
  const int WAKTU_PUTAR_180 = 500;  // Durasi (ms) agar robot putar balik ~180 derajat
  const int JEDA_ANTAR_SENSOR = 30; // Jeda (ms) antar pembacaan sensor biar gema nggak saling ganggu

  // --- KECEPATAN MOTOR ---
  const int KECEPATAN_MAJU = 255; // Kecepatan motor saat maju (0-255)
  const int BELOK_KIRI = 180;     // Kecepatan motor saat berbelok (0-255)
  const int BELOK_KANAN = 180;    // Kecepatan motor saat berbelok (0-255)
  const int PUTAR_BALIK = 255;    // Kecepatan motor saat putar balik (0-255)

  const int KECEPATAN_MUNDUR = 200;     // Kecepatan motor saat mundur (0-255)

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
    pinMode(TRIG_PIN_1, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    pinMode(TRIG_PIN_2, OUTPUT);
    pinMode(ECHO_PIN_2, INPUT);
    pinMode(TRIG_PIN_3, OUTPUT);
    pinMode(ECHO_PIN_3, INPUT);

    berhenti();
    delay(2000); // Jeda awal 2 detik sebelum mobil bergerak
  }

void loop() {
  int jarakDepan = bacaJarak(TRIG_PIN_1, ECHO_PIN_1);
  Serial.print("Jarak Depan: ");
  Serial.print(jarakDepan);
  Serial.println(" cm");

  if (jarakDepan > JARAK_AMAN) {
    maju(KECEPATAN_MAJU);
  } else {
    // Depan terhalang -> MUNDUR sampai ada sisi yang bebas
    Serial.println("Depan terhalang, mundur...");

    int jarakKanan = 0;
    int jarakKiri = 0;
    bool kananBebas = false;
    bool kiriBebas = false;

    mundur(KECEPATAN_MUNDUR);

    while (true) {
      delay(JEDA_ANTAR_SENSOR);
      jarakKanan = bacaJarak(TRIG_PIN_2, ECHO_PIN_2);
      delay(JEDA_ANTAR_SENSOR);
      jarakKiri = bacaJarak(TRIG_PIN_3, ECHO_PIN_3);

      kananBebas = (jarakKanan > JARAK_AMAN);
      kiriBebas  = (jarakKiri  > JARAK_AMAN);

      Serial.print("Mundur - Kanan: ");
      Serial.print(jarakKanan);
      Serial.print(" cm | Kiri: ");
      Serial.print(jarakKiri);
      Serial.println(" cm");

      if (kananBebas || kiriBebas) break;
    }

    berhenti();
    delay(300);

    if (kananBebas && kiriBebas) {
      Serial.println("Kanan & kiri bebas -> Putar balik");
      putarBalik(PUTAR_BALIK);
      delay(WAKTU_PUTAR_180);
    } else if (kananBebas) {
      Serial.println("Tembok kiri ada -> Belok kanan");
      belokKanan(BELOK_KANAN);
      delay(WAKTU_BELOK_90);
    } else {
      Serial.println("Tembok kanan ada -> Belok kiri");
      belokKiri(BELOK_KIRI);
      delay(WAKTU_BELOK_90);
    }

    berhenti();
    delay(300);
  }

  delay(JEDA_ANTAR_SENSOR);
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

    // Menerima pantulan gelombang (timeout 30ms ~ 5 meter)
    long durasi = pulseIn(echoPin, HIGH, 30000);

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