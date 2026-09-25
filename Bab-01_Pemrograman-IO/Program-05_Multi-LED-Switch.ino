/*
  Program 4 LED dengan Push Button
  Program ini dibuat oleh ImNotDanish05

  Kondisi LED bergantung pada keadaan push button S.
  Saat tombol ditekan, dua LED menyala dan dua LED padam.
  Saat tombol tidak ditekan, kondisi LED menjadi kebalikannya.
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

void loop()
{
  // Baca kondisi push button pada pin 7
  int tombol = digitalRead(7);

  if (tombol == HIGH)
  {
    // Tombol ditekan: LED 12 dan 13 menyala
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    delay(1000);
  }
  else
  {
    // Tombol tidak ditekan: LED 10 dan 11 menyala
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    delay(1000);
  }
}
