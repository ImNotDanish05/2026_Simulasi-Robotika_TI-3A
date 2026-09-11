/*
  Program 08 - Multi LED dengan Threshold Potensiometer

  Program ini dibuat oleh ImNotDanish05

  Jika nilai potensiometer lebih dari threshold,
  maka 4 LED menyala.

  Jika nilai potensiometer sama dengan atau di bawah threshold,
  maka 4 LED mati.
*/

const int potPin = A0;

// Ubah angka ini untuk mengatur threshold
const int threshold = 500;

const int led1 = 10;
const int led2 = 11;
const int led3 = 12;
const int led4 = 13;

void setup()
{
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  int nilaiPot = analogRead(potPin);

  Serial.print("Nilai Potensiometer: ");
  Serial.print(nilaiPot);

  if (nilaiPot < threshold)
  {
    // Potensiometer di atas threshold → LED menyala
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);

    Serial.println(" | LED: NYALA");
  }
  else
  {
    // Potensiometer di bawah atau sama dengan threshold → LED mati
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);

    Serial.println(" | LED: MATI");
  }

  delay(100);
}