// --- PIN SENSOR ULTRASONIK (HC-SR04) ---
const int TRIG_DEPAN = 22;
const int ECHO_DEPAN = 23;

const int TRIG_KIRI  = 18;
const int ECHO_KIRI  = 19;

// --- AMBANG BATAS JARAK (CM) ---
const float BATAS_DEPAN    = 15.0; // Dianggap ada halangan di depan
const float BATAS_KIRI_ADA = 25.0; // Dianggap masih ada dinding di kiri

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_DEPAN, OUTPUT);
  pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);
  pinMode(ECHO_KIRI, INPUT);

  Serial.println("=== MULAI DEBUG SENSOR ULTRASONIK ===");
  delay(1000);
}

void loop() {
  // 1. Baca jarak sensor depan
  float jarakDepan = bacaUltrasonik(TRIG_DEPAN, ECHO_DEPAN);
  delay(15); // Jeda agar gelombang suara sensor depan tidak mengganggu sensor kiri

  // 2. Baca jarak sensor kiri
  float jarakKiri = bacaUltrasonik(TRIG_KIRI, ECHO_KIRI);

  // 3. Print hasil pembacaan jarak
  Serial.print("Depan: ");
  if (jarakDepan >= 900) {
    Serial.print("KOSONG (>400 cm)");
  } else {
    Serial.print(jarakDepan, 1);
    Serial.print(" cm");
  }

  Serial.print("  |  Kiri: ");
  if (jarakKiri >= 900) {
    Serial.print("KOSONG (>400 cm)");
  } else {
    Serial.print(jarakKiri, 1);
    Serial.print(" cm");
  }

  // 4. Print simulasi keputusan robot berdasarkan kondisi
  Serial.print("  --> Status: ");
  if (jarakDepan <= BATAS_DEPAN) {
    if (jarakKiri <= BATAS_KIRI_ADA) {
      Serial.println("DEPAN TERHALANG & KIRI ADA -> BELOK KANAN");
    } else {
      Serial.println("DEPAN TERHALANG & KIRI KOSONG -> BELOK KIRI");
    }
  } else if (jarakKiri > BATAS_KIRI_ADA) {
    Serial.println("KIRI KOSONG MLOMPONG -> BERHENTI & BELOK KIRI");
  } else {
    Serial.println("SUSUR DINDING KIRI (PID AKTIF)");
  }

  delay(250); // Jeda pembacaan agar mudah dibaca di Serial Monitor
}

float bacaUltrasonik(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long durasi = pulseIn(echoPin, HIGH, 25000);
  if (durasi == 0) {
    return 999.0; // Timeout (tidak ada pantulan)
  }
  return durasi * 0.0343 / 2.0;
}