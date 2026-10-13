# Simulasi Robotika — TI-3A (Coursework & UTS Project)

Repository ini berisi seluruh tugas praktikum laboratorium mikrokontroler (Arduino Uno) dan simulasi robotika mobile (*E-puck*) menggunakan simulator **Webots R2025a** untuk mata kuliah **Simulasi Robotika (Kelas TI-3A)**.

---

## 🏆 Project UTS — Simulasi Robotika (`Bab-04_WeBots/test_01`)

Folder `Bab-04_WeBots/test_01/` merupakan implementasi proyek Ujian Tengah Semester (UTS), yang mengadopsi model labirin standar **Trinity College International Fire-Fighting Home Robot Contest / Kontes Robot Pemadam Api Indonesia (KRPAI)** skala **1:2 (50%)**.

### 1. Map & Arena Labirin UTS (`test_01.wbt`)
* **Ukuran Arena:** $1.20\text{ m} \times 1.22\text{ m}$ dengan lebar lorong dan pintu standar $23\text{ cm}$.
* **Pembagian Ruangan:** Terdiri dari 4 ruangan (Room 1 s.d. Room 4) dengan Room 4 sebagai ruang pulau (*island room*) di bagian tengah atas.
* **Titik Start:** Robot E-puck diletakkan pada koordinat $(x=-0.42, y=0.04)$ menghadap Utara.
* **Target Misi:** Balok kubus merah (*Red Target Goal*) berada di dalam Room 4 pada koordinat $(x=0.20, y=0.25)$.
* **Spesifikasi Lengkap:** Lihat dokumentasi teknis di [`Bab-04_WeBots/catatan/arena_uts.md`](Bab-04_WeBots/catatan/arena_uts.md).

### 2. Program Controller Robot UTS (`global_robot_controller_sensor.c`)
Controller otonom lengkap untuk robot mobile E-puck dengan kapabilitas:
1. **Eksplorasi Global (`EXPLORE`):**
   * *Reactive Right-Wall Follower* dengan PD controller pada sensor jarak inframerah (`ps0` - `ps7`).
   * Pencegah kebuntuan / jalan berputar (*Anti-Looping Grid Cell Explorer*).
2. **Deteksi Target Merah:**
   * Pengolahan citra kamera robot (`camera` 64x64) dengan filter warna merah strictly-defined (`r > 140, g < 65, b < 65, r - g > 80`).
3. **Muter Balik 180° (`RETURN_UTURN`):**
   * Berputar di tempat (*pivot in-place*) secara penuh dan tuntas ($\ge 3.20\text{ rad}$) ke arah yang paling lapang untuk mengompensasi selip roda.
4. **Navigasi Pulang Bebas Drift (`RETURN_HOME`):**
   * Menggunakan prinsip *Dual Wall-Following (Symmetric Inverse Rule)* dengan menyusuri dinding sebaliknya (**Left-Wall Follower**) murni berbasis sensor jarak inframerah tanpa terpengaruh akumulasi selip odometri.
5. **Masuk Ruangan Start Penuh (`RETURN_ENTER_ROOM`):**
   * Mendeteksi pintu masuk ruangan start, lalu meluncur maju menembus kusen pintu sejauh $\approx 25\text{ cm}$ hingga seluruh bodi robot berada di dalam ruangan sebelum berhenti sempurna (`DONE`).
* **Dokumentasi Solusi Navigasi Pulang:** Lihat [`Bab-04_WeBots/catatan/rencana_solusi_return_home.md`](Bab-04_WeBots/catatan/rencana_solusi_return_home.md).

---

## 📂 Struktur Repositori

```
2026_Simulasi-Robotika_TI-3A/
├── README.md                                # Dokumentasi utama repositori
├── DANISH_AI_AGENT.md                       # Context file untuk AI agent (SOP single-file context)
├── Bab-01_Pemrograman-IO/                   # Lab 1: LED pattern active-low, push button, potentiometer, software PWM
├── Bab-02_Servo/                            # Lab 2: Kontrol servo 3-posisi & kemudi jarak sensor ultrasonik
├── Bab-03_DC/                               # Lab 3: Motor DC arah & kecepatan PWM via driver L293D
└── Bab-04_WeBots/                           # Lab 4 & Project UTS: Simulasi Webots E-puck
    ├── catatan/                             # Dokumentasi blueprint arena, perhitungan koordinat, & solusi navigasi
    │   ├── arena_uts.md                     # Blueprint arena 1:2 Trinity/KRPAI & mapping koordinat Webots
    │   ├── arena uts.docx                   # Dokumen Word acuan awal spesifikasi arena UTS
    │   └── rencana_solusi_return_home.md    # Perbandingan 3 solusi kembali ke start (Dual Wall, GPS, Pure Pursuit)
    ├── latihan_01/                          # Latihan awal dasar gerak, sensor jarak, dan kamera E-puck
    │   ├── worlds/latihan_01.wbt            # World latihan awal
    │   └── controllers/                     # Controller uji coba (sensor 1 s.d. 4)
    └── test_01/                             # === PROJECT UTS SIMULASI ROBOTIKA ===
        ├── worlds/test_01.wbt               # World resmi UTS (Arena Trinity/KRPAI 1:2 + target merah)
        └── controllers/
            └── global_robot_controller_sensor/  # Controller utama UTS (Explore, Detect, U-Turn, Return Home)
                ├── global_robot_controller_sensor.c
                └── Makefile
```

---

## 🛠️ Cara Menjalankan Simulasi UTS

1. **Buka Webots R2025a:**
   ```bash
   webots Bab-04_WeBots/test_01/worlds/test_01.wbt
   ```
2. **Build Controller (jika mengedit kode C):**
   * Melalui Webots GUI: Tekan ikon **Build** (ikon roda gigi / palu) pada Text Editor Webots, atau
   * Melalui Terminal:
     ```bash
     make -C Bab-04_WeBots/test_01/controllers/global_robot_controller_sensor
     ```
3. **Jalankan Simulasi:**
   * Tekan tombol **Play** / **Fast-Forward** pada toolbar kontrol simulasi Webots.
   * Amati robot menjelajahi labirin, menemukan balok merah di Room 4, putar balik 180°, lalu pulang menyusuri dinding kiri hingga masuk sepenuhnya ke ruangan start.