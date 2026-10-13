# Rencana Solusi "Return Home" (Kembali ke Posisi Start)
Dokumentasi perbandingan 3 metode navigasi untuk mengembalikan robot E-puck dari posisi target merah kembali ke titik awal (*home/start*) pada arena labirin UTS Trinity / KRPAI Webots.

---

## 1. Opsi A — Dual Wall-Following (Symmetric Reverse Wall Follower)
> **Status:** Sedang Diuji (Implementasi Utama Saat Ini)  
> **Karakteristik:** Murni berbasis sensor jarak (`ps0` - `ps7`), 100% bebas dari ketergantungan koordinat odometri $(x, y)$.

### Prinsip Kerja
1. **Symmetric Hand Rule (Teorema Labirin):**
   * Saat pergi menjelajah (*EXPLORE*), robot menelusuri dinding kanan (**Right-Wall Follower**).
   * Begitu melihat balok merah di dalam ruangan, robot melakukan putar balik $180^\circ$ (*U-Turn*).
   * Dinding yang tadinya berada di sisi kanan robot saat berangkat, kini secara geometri berada di sisi **kiri** robot saat berbalik arah.
   * Maka, untuk pulang robot beralih $100\%$ menjadi penelusur dinding kiri (**Left-Wall Follower**).
2. **Mesin Keadaan (FSM) Kepulangan:**
   * `M_FOLLOW`: Menjaga jarak ke dinding kiri pada target $150.0$ ADC dengan PD controller.
   * `M_LOST_FWD` & `M_LOST_ARC`: Saat dinding kiri berbelok keluar atau melewati pintu/sudut, robot memajukan roda lalu membelok tajam ke kiri (*inward arc*) untuk mengunci dinding kembali.
   * `M_TURN`: Saat ada jalan buntu di depan, robot berputar di tempat menjauhi dinding depan.
3. **Kriteria Selesai (Stop di Start):**
   * Setelah berjalan pulang sejauh jarak minimum tertentu, robot mendeteksi area awal ketika koordinat relatif mendekati titik asal ($\text{hypot}(x, y) < 0.20\text{ m}$).
   * Karena odometri hanya dipakai untuk *trigger* radius stop (bukan untuk mengemudi), toleransi $15 - 20\text{ cm}$ sangat aman dari efek selip roda.

### Kelebihan & Kekurangan
* **Kelebihan:**
  * Tidak akan pernah "keluar jalur" karena robot terus mengunci dinding fisik lorong.
  * Kebal terhadap akumulasi selip roda (*wheel slip/drift*).
  * Sangat andal di lorong standar $23\text{ cm}$.
* **Kekurangan:**
  * Jalur pulang mengikuti kontur dinding yang sama (bukan rute terpendek garis lurus).

---

## 2. Opsi B — GPS Ground-Truth (Sensor GPS Bawaan Webots)
> **Karakteristik:** Menggunakan fitur simulasi Webots bawaan untuk membaca koordinat absolut tanpa drift sedikit pun.

### Prinsip Kerja
1. **Pemasangan Sensor GPS:**
   * Di dalam file `test_01.wbt` pada node `E-puck`, tambahkan node `GPS { }` pada slot `groundSensorsSlot` atau `turretSlot`.
   * Pada controller C, aktifkan device:
     ```c
     WbDeviceTag gps = wb_robot_get_device("gps");
     wb_gps_enable(gps, TIME_STEP);
     ```
2. **Pembacaan Koordinat 100% Bebas Selip:**
   * Alih-alih mengintegralkan enkoder roda:
     ```c
     const double *gps_pos = wb_gps_get_values(gps);
     double x = gps_pos[0];
     double y = gps_pos[1];
     ```
   * Nilai $(x, y)$ ini adalah posisi fisik nyata di dunia simulasi dengan toleransi 0 mm.
3. **Navigasi Pulang (Breadcrumbs Reversal):**
   * Remah jalur (*path crumbs*) yang direkam saat berangkat 100% presisi dan tepat di tengah lorong yang dilalui.
   * Saat pulang, robot melacak kembali remah tersebut secara terbalik menggunakan algoritma *Pure Pursuit* (lookahead 9 cm).

### Kelebihan & Kekurangan
* **Kelebihan:**
  * Jalur pulang adalah rute optimal/tercepat (remah loop sudah dibuang).
  * Tidak ada error akumulasi jarak.
* **Kekurangan:**
  * Di dunia fisik nyata (Arduino Uno / robot KRPAI fisik), tidak ada GPS di dalam ruangan (*indoor*). Ini adalah solusi khusus simulasi Webots.

---

## 3. Opsi C — Odometry-Corrected Breadcrumb Pure Pursuit
> **Karakteristik:** Menggunakan enkoder roda murni + perbaikan kontrol pelacakan remah tanpa tabrakan repulsi.

### Prinsip Kerja
1. **Pemisahan Kontrol Kemudi:**
   * Pada pengujian sebelumnya, gaya tolak dinding (*wall repulsion*) bertabrakan langsung dengan setir remah (*steer pursuit*), menyebabkan robot terlempar keluar jalur.
   * Pada Opsi C, gaya repulsi diubah menjadi batas keamanan darurat (*hard safety boundary*), bukan kemudi kontinu:
     - Jika robot berjarak aman dari dinding, kemudi $100\%$ dipandu oleh remah jalur (*pure pursuit*).
     - Hanya jika sensor depan/samping sangat dekat ($> 450$ ADC), koreksi keselamatan darurat aktif untuk memperlambat robot.
2. **Dead-Reckoning Rescaling:**
   * Menggunakan radius lookahead adaptif ($L = 0.12\text{ m}$) agar belokan robot lebih lembut dan tidak memotong sudut tajam dinding.
3. **Pemberhentian di Titik Awal:**
   * Berhenti ketika remah mencapai index `0` dan jarak ke koordinat awal $< 0.08\text{ m}$.

### Kelebihan & Kekurangan
* **Kelebihan:**
  * Jalur pulang lebih cepat daripada menyusuri seluruh dinding.
  * Dapat diimplementasikan pada mikrokontroler fisik tanpa GPS.
* **Kekurangan:**
  * Jika robot sempat menabrak dinding keras saat eksplorasi awal, odometri tetap rentan memiliki deviasi beberapa sentimeter.

---

## Rekomendasi Urutan Pengujian
1. **Langkah 1 (Sekarang):** Uji coba **Opsi A** (Dual Wall-Following) karena paling andal, reaktif, dan tidak bergantung pada drift odometri.
2. **Langkah 2:** Jika rute Opsi A dirasa terlalu panjang atau ingin rute terpendek, uji coba **Opsi B** dengan menambahkan sensor GPS di Webots.
3. **Langkah 3:** Gunakan **Opsi C** jika modul kuliah mewajibkan navigasi berbasis enkoder murni tanpa GPS dan tanpa *wall-following* simetris.

---

## 4. Rencana Solusi: Memastikan Robot Masuk Sepenuhnya ke Ruangan Start
> **Latar Belakang:** Pada pengujian Opsi A, robot berhasil menelusuri dinding kembali sampai ke pintu masuk, namun berhenti terlalu dini di ambang pintu karena ambang batas jarak penghentian default ($22\text{ cm}$) tercapai persis di kusen/pintu masuk ruangan.

### Tiga Alternatif Solusi:
1. **Plan 1 — Precision Radius Threshold:**
   * Menurunkan ambang batas jarak penghentian dari $0.22\text{ m}$ menjadi $0.06 - 0.08\text{ m}$ ($6-8\text{ cm}$).
   * Robot tidak akan berhenti di pintu, melainkan terus mengikuti dinding masuk ke dalam ruangan sampai mendekati titik $(0,0)$.
2. **Plan 2 — Dedicated Two-Stage State (`RETURN_ENTER_ROOM`):** *(Direkomendasikan & Diimplementasikan)*
   * **Stage 1 (Deteksi Pintu Masuk):** Saat $d_{\text{start}} < 0.22\text{ m}$, robot tidak langsung berhenti, melainkan berganti status ke `RETURN_ENTER_ROOM`.
   * **Stage 2 (Drive-In Commit):** Robot terus meluncur masuk ke dalam ruangan sejauh $20-25\text{ cm}$ (sekitar 40 steps) atau sampai $d_{\text{start}} < 0.07\text{ m}$, atau sampai sensor depan mendeteksi dinding dalam ruangan ($v[0] > 250$).
   * Memberikan jaminan bodi robot 100% melintasi ambang pintu dan berada di dalam ruangan.
3. **Plan 3 — Reverse Breadcrumb Docking (`path[0..5]`):**
   * Saat berada di pintu masuk, robot beralih ke docking remah awal (`path[0]` s.d. `path[5]`) yang direkam saat awal simulasi di dalam ruangan.


