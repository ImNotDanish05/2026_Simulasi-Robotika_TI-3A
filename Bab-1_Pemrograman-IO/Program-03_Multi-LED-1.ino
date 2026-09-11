/*
  Program 4 LED dengan 2 Pola Kelap-Kelip
  Program ini dibuat oleh ImNotDanish05

  Pola 1: Keempat LED menyala dan padam secara bergantian.
  Pola 2: Dua LED menyala bergantian dengan dua LED lainnya.
*/

void setup()
{
  // Atur pin 10, 11, 12, dan 13 sebagai output LED
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void pola1()
{
  // Matikan semua LED
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(1000);

  // Nyalakan semua LED
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  delay(1000);
}

void pola2()
{
  // Nyalakan LED 12 dan 13, matikan LED 10 dan 11
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  delay(1000);

  // Nyalakan LED 10 dan 11, matikan LED 12 dan 13
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(1000);
}

void loop()
{
  // Jalankan pola 1 sebanyak 3 kali
  pola1();
  pola1();
  pola1();

  // Jalankan pola 2 sebanyak 3 kali
  pola2();
  pola2();
  pola2();
}
