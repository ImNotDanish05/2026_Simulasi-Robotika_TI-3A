/*
  Program 4 LED dengan Perulangan FOR
  Program ini dibuat oleh ImNotDanish05

  Program menggunakan dua pola LED dengan jumlah pengulangan
  yang berbeda menggunakan perintah for.
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
  // Ulangi pola 1 sebanyak 7 kali
  for (int i = 0; i < 7; i++)
  {
    // Matikan semua LED
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    delay(100);

    // Nyalakan semua LED
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    delay(100);
  }
}

void pola2()
{
  // Ulangi pola 2 sebanyak 5 kali
  for (int i = 0; i < 5; i++)
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
}

void loop()
{
  // Jalankan pola 1 lalu pola 2 secara berurutan
  pola1();
  pola2();
}
