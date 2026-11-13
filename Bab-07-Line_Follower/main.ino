// --- PIN MOTOR DRIVER L298N KE ESP32 (tidak berubah) ---
const int ENA = 13; // D13 - PWM Kecepatan Motor Kiri
const int IN1 = 14; // D14 - Arah Motor Kiri
const int IN2 = 27; // D27 - Arah Motor Kiri

const int IN3 = 26; // D26 - Arah Motor Kanan
const int IN4 = 25; // D25 - Arah Motor Kanan
const int ENB = 33; // D33 - PWM Kecepatan Motor Kanan

// --- PIN SENSOR GARIS (3 sensor) ---
const int SENSOR_KIRI   = 34; // D34 - Sensor Kiri   (input only)
const int SENSOR_TENGAH = 35; // D35 - Sensor Tengah (input only)
const int SENSOR_KANAN  = 32; // D32 - Sensor Kanan

void setup() {
  // Motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor
  pinMode(SENSOR_KIRI, INPUT);
  pinMode(SENSOR_TENGAH, INPUT);
  pinMode(SENSOR_KANAN, INPUT);

  Serial.begin(115200);
}

void loop(){
    
}