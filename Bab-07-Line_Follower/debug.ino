// --- PIN SENSOR GARIS ---
const int SENSOR_KIRI  = 22; // D23
const int SENSOR_KANAN = 23; // D22

int kiriTerakhir  = -1;
int kananTerakhir = -1;

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_KIRI, INPUT);
  pinMode(SENSOR_KANAN, INPUT);
  Serial.println("=== TES SENSOR GARIS ===");
  Serial.println("Gerakkan kertas hitam/putih di bawah sensor");
}

void loop() {
  int kiri  = digitalRead(SENSOR_KIRI);
  int kanan = digitalRead(SENSOR_KANAN);

  // Cetak hanya kalau ada perubahan, biar Serial Monitor tidak penuh
  if (kiri != kiriTerakhir || kanan != kananTerakhir) {
    Serial.printf("Kiri: %d | Kanan: %d\n", kiri, kanan);
    kiriTerakhir  = kiri;
    kananTerakhir = kanan;
  }

  delay(20);
}