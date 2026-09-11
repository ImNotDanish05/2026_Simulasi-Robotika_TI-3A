/*
  Program 09 - Multi LED dengan Potensiometer dan Push Button

  Program ini dibuat oleh ImNotDanish05

  Potensiometer pada A0 digunakan untuk mengatur
  tingkat kecerahan LED yang sedang dipilih.

  Tombol pada pin 7 digunakan untuk berpindah
  ke LED berikutnya.

  Urutan LED:
  10 -> 11 -> 12 -> 13 -> kembali ke 10

  Setiap LED menyimpan tingkat kecerahan terakhirnya,
  sehingga saat pindah LED, brightness sebelumnya tetap tersimpan.

  LED menggunakan konfigurasi active-low:
  LOW  = menyala
  HIGH = mati
*/

// =========================
// Konfigurasi LED
// =========================

const int ledPins[] = {10, 11, 12, 13};
const bool isHardwarePWM[] = {
  true,   // Pin 10
  true,   // Pin 11
  false,  // Pin 12
  false   // Pin 13
};

const int numLeds = 4;

// Menyimpan brightness masing-masing LED
// Nilai 0 - 255
int brightness[numLeds] = {0, 0, 0, 0};

// =========================
// Konfigurasi input
// =========================

const int potPin = A0;
const int buttonPin = 7;

// LED yang sedang dipilih
int currentLed = 0;

// =========================
// Debounce button
// =========================

int lastButtonReading = HIGH;
int stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// =========================
// Serial Monitor
// =========================

int lastPrintedLed = -1;
int lastPrintedBrightness = -1;

// =========================
// Software PWM
// =========================

const unsigned long pwmPeriod = 2000; // 500 Hz
unsigned long pwmCycleStart = 0;


// =====================================================
// SETUP
// =====================================================

void setup()
{
  // Atur semua pin LED sebagai output
  for (int i = 0; i < numLeds; i++)
  {
    pinMode(ledPins[i], OUTPUT);

    // LED active-low → HIGH = mati
    digitalWrite(ledPins[i], HIGH);
  }

  // Button menggunakan internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);

  printStatus();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // 1. Baca potensiometer
  // ---------------------------------------------------

  int potValue = analogRead(potPin);

  // Ubah nilai 0-1023 menjadi 0-255
  brightness[currentLed] = map(
    potValue,
    0,
    1023,
    0,
    255
  );


  // ---------------------------------------------------
  // 2. Baca tombol
  // ---------------------------------------------------

  int reading = digitalRead(buttonPin);

  // Jika pembacaan berubah, mulai timer debounce
  if (reading != lastButtonReading)
  {
    lastDebounceTime = millis();
  }

  // Jika keadaan sudah stabil selama debounceDelay
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != stableButtonState)
    {
      stableButtonState = reading;

      // Tombol ditekan
      if (stableButtonState == LOW)
      {
        currentLed++;

        // Jika melewati LED terakhir, kembali ke LED pertama
        if (currentLed >= numLeds)
        {
          currentLed = 0;
        }

        printStatus();
      }
    }
  }

  // Simpan pembacaan terakhir
  lastButtonReading = reading;


  // ---------------------------------------------------
  // 3. Update semua LED
  // ---------------------------------------------------

  updateOutputs();


  // ---------------------------------------------------
  // 4. Serial Monitor
  // ---------------------------------------------------

  if (
    currentLed != lastPrintedLed ||
    brightness[currentLed] != lastPrintedBrightness
  )
  {
    printStatus();
  }
}


// =====================================================
// UPDATE OUTPUT LED
// =====================================================

void updateOutputs()
{
  unsigned long now = micros();

  unsigned long elapsed = now - pwmCycleStart;

  if (elapsed >= pwmPeriod)
  {
    pwmCycleStart = now;
    elapsed = 0;
  }

  for (int i = 0; i < numLeds; i++)
  {
    // Pin 10 dan 11 → Hardware PWM
    if (isHardwarePWM[i])
    {
      // Karena active-low, brightness dibalik
      analogWrite(
        ledPins[i],
        255 - brightness[i]
      );
    }

    // Pin 12 dan 13 → Software PWM
    else
    {
      unsigned long onTime = map(
        brightness[i],
        0,
        255,
        0,
        pwmPeriod
      );

      if (elapsed < onTime)
      {
        // Active-low → LOW = nyala
        digitalWrite(ledPins[i], LOW);
      }
      else
      {
        // Active-low → HIGH = mati
        digitalWrite(ledPins[i], HIGH);
      }
    }
  }
}


// =====================================================
// SERIAL STATUS
// =====================================================

void printStatus()
{
  Serial.print("LED Pin ");
  Serial.print(ledPins[currentLed]);

  Serial.print(" | Brightness: ");
  Serial.print(brightness[currentLed]);

  Serial.print(" | Pot: ");
  Serial.println(analogRead(potPin));

  lastPrintedLed = currentLed;
  lastPrintedBrightness = brightness[currentLed];
}