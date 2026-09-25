/*
  Program 4 LED dengan 2 Pola dan Push Button
  Program ini dibuat oleh ImNotDanish05

  Pola LED bergantung pada keadaan push button S.
  Saat tombol ditekan, program menjalankan pola 1.
  Saat tombol tidak ditekan, program menjalankan pola 2.
*/

void setup()
{
  // Atur pin 10, 11, 12, dan 13 sebagai output LED
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // Atur pin 7 sebagai input push button
  pinMode(7, INPUT);
}

void pola1()
{
  // Ulangi pola 1 sebanyak 5 kali
  for (int i = 0; i < 5; i++)
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
  // Ulangi pola 2 sebanyak 3 kali
  for (int i = 0; i < 3; i++)
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
  // Baca kondisi push button pada pin 7
  int tombol = digitalRead(7);

  if (tombol == HIGH)
  {
    // Tombol ditekan: jalankan pola 1
    pola1();
  }
  else
  {
    // Tombol tidak ditekan: jalankan pola 2
    pola2();
  }
}
