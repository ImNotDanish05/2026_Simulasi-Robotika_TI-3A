/*
  Program 07 - Multi LED dengan Potensiometer

  Program ini dibuat oleh ImNotDanish05

  Potensiometer digunakan untuk mengatur
  tingkat kecerahan 4 LED.

  Potensiometer:
  - 5V
  - A0
  - GND

  LED:
  - Pin 10
  - Pin 11
  - Pin 12
  - Pin 13
*/

const int potPin = A0;

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
}

void loop()
{
  // Baca nilai potensiometer (0-1023)
  int nilaiPot = analogRead(potPin);

  // Ubah menjadi nilai kecerahan (0-255)
  int brightness = map(nilaiPot, 0, 1023, 0, 255);

  // Software PWM untuk 4 LED
  for (int i = 0; i < 255; i++)
  {
    if (i < brightness)
    {
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
    }
    else
    {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);
      digitalWrite(led4, HIGH);
    }
  }
}