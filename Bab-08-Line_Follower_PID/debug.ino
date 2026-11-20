```cpp
// ==========================================
//       DEBUG 3 SENSOR GARIS ESP32
// ==========================================

// Pin sensor
const int SENSOR_KIRI   = 22;
const int SENSOR_TENGAH = 21;
const int SENSOR_KANAN  = 23;

// true  = garis terdeteksi saat HIGH
// false = garis terdeteksi saat LOW
const bool GARIS_TERBACA_HIGH = true;

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_KIRI, INPUT);
  pinMode(SENSOR_TENGAH, INPUT);
  pinMode(SENSOR_KANAN, INPUT);

  Serial.println("================================");
  Serial.println("     LINE SENSOR DEBUGGER");
  Serial.println("================================");
  Serial.println("KIRI | TENGAH | KANAN");
}

void loop() {
  int kiri = digitalRead(SENSOR_KIRI);
  int tengah = digitalRead(SENSOR_TENGAH);
  int kanan = digitalRead(SENSOR_KANAN);

  bool K = GARIS_TERBACA_HIGH ? (kiri == HIGH) : (kiri == LOW);
  bool T = GARIS_TERBACA_HIGH ? (tengah == HIGH) : (tengah == LOW);
  bool Ka = GARIS_TERBACA_HIGH ? (kanan == HIGH) : (kanan == LOW);

  Serial.print("KIRI: ");
  Serial.print(K ? "DETEKSI" : "TIDAK");
  
  Serial.print(" | TENGAH: ");
  Serial.print(T ? "DETEKSI" : "TIDAK");

  Serial.print(" | KANAN: ");
  Serial.print(Ka ? "DETEKSI" : "TIDAK");

  Serial.print(" | POLA: ");
  Serial.print(K);
  Serial.print(T);
  Serial.println(Ka);

  delay(200);
}
```