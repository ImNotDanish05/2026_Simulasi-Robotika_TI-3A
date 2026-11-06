#include <WiFi.h>         // WAJIB pakai ini untuk ESP32 (menggantikan ESP8266WiFi.h)
#include <PubSubClient.h>
#include <DHT.h>
#include "config.h"       // Memanggil kredensial rahasia dari config.h yang aman

// ==========================================
// KONFIGURASI TOPIK MQTT (NIM: 4.33.24.0.09)
// ==========================================
const char* TOPIC_SUHU        = "UAS25-IOT/4.33.24.0.09/SUHU";
const char* TOPIC_KELEMBAPAN  = "UAS25-IOT/4.33.24.0.09/KELEMBAPAN";
const char* TOPIC_LUMEN       = "UAS25-IOT/4.33.24.0.09/LUMEN";
const char* TOPIC_STATUS      = "UAS25-IOT/Status";

// ==========================================
// KONFIGURASI PIN HARDWARE (ESP32 DEVKIT V1)
// ==========================================
#define DHTPIN  26     // Dicolok ke pin GPIO 26 (D26)
#define DHTTYPE DHT11  // Jenis Sensor DHT
#define LDRPIN  34     // Dicolok ke pin GPIO 34 (Pin ADC murni, aman saat WiFi nyala)
#define LEDPIN  2      // GPIO 2 (Biasanya tersambung ke LED Biru internal ESP32)

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// Variabel Kontrol Pengiriman Data
bool isPublishing = false; 
unsigned long lastMsg = 0;

void setup_wifi() {
  delay(10);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID_WIFI);

  WiFi.begin(SSID_WIFI, PASSWORD_WIFI);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Fungsi Callback untuk memproses perintah START / STOP
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Pesan masuk di topik [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Implementasi Mekanisme Kontrol
  if (String(topic) == TOPIC_STATUS) {
    if (message == "START") {             
      isPublishing = true;
      digitalWrite(LEDPIN, HIGH);         // LED Status Menyala
      Serial.println("Status: START -> Memulai publikasi data sensor.");
    } else if (message == "STOP") {       
      isPublishing = false;
      digitalWrite(LEDPIN, LOW);          // LED Status Mati
      Serial.println("Status: STOP -> Menghentikan publikasi data sensor.");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT Broker...");
    // Menggunakan Kredensial User 1 (Mahasiswa)
    if (client.connect("ESP32_Danish", MQTT_USER, MQTT_PASS)) {
      Serial.println("Terhubung!");
      // Mengambil data dari topik kontrol
      client.subscribe(TOPIC_STATUS);
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" Mencoba kembali dalam 5 detik...");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW); // Default LED mati sebelum ada perintah START
  
  setup_wifi();
  dht.begin();
  
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Logika Publikasi Data Sensor: Hanya jika isPublishing bernilai true (START)
  if (isPublishing) {
    unsigned long now = millis();
    if (now - lastMsg > 2000) { // Pengiriman data berkala setiap 2 detik
      lastMsg = now;

      // Membaca Nilai Sensor DHT11 dan LDR
      float suhu = dht.readTemperature();   
      float kelembapan = dht.readHumidity(); 
      int lumen = analogRead(LDRPIN);       

      // Memastikan pembacaan sensor valid
      if (!isnan(suhu) && !isnan(kelembapan)) {
        Serial.print("Publish Data -> Suhu: ");
        Serial.print(suhu);
        Serial.print(" °C | Kelembapan: ");
        Serial.print(kelembapan);
        Serial.print(" % | Lumen: ");
        Serial.println(lumen);

        // Mempublikasikan data ke topik masing-masing
        client.publish(TOPIC_SUHU, String(suhu).c_str());
        client.publish(TOPIC_KELEMBAPAN, String(kelembapan).c_str());
        client.publish(TOPIC_LUMEN, String(lumen).c_str());
      } else {
        Serial.println("Peringatan: Gagal membaca data dari sensor DHT11!");
      }
    }
  }
}