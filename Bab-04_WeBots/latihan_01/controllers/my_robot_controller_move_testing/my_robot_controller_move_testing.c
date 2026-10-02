/*
 * File:          my_robot_controller_move_testing.c
 * Description:   Controller pengujian gerakan robot E-puck:
 *                Maju 5s -> Berhenti 5s -> Mundur 5s -> Berhenti 5s -> Belok kiri -> Ulang
 */

#include <stdio.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Kecepatan standar (setengah kecepatan maksimal agar stabil)
#define SPEED (0.5 * MAX_SPEED)

// Durasi belok kiri dalam detik untuk putaran ~90 derajat
// (Perhitungan: 2.0 detik menghasilkan 225 derajat, maka 90/225 * 2.0 = 0.8 detik)
#define TURN_LEFT_DURATION 0.8 

/*
 * Fungsi pembantu untuk mengatur kecepatan motor kiri dan kanan
 */
void set_speeds(WbDeviceTag left_motor, WbDeviceTag right_motor, double left_speed, double right_speed) {
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}

/*
 * Fungsi pembantu untuk menunggu selama sejumlah detik (dalam waktu simulasi Webots).
 * Mengembalikan false jika simulasi dihentikan/selesai oleh pengguna.
 */
bool wait_seconds(double duration) {
  double start_time = wb_robot_get_time();
  while (wb_robot_get_time() - start_time < duration) {
    if (wb_robot_step(TIME_STEP) == -1) {
      return false; // Simulasi selesai atau jendela ditutup
    }
  }
  return true;
}

int main(int argc, char **argv) {
  /* Inisialisasi API Webots */
  wb_robot_init();

  /* Mengambil device motor E-puck */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  /*
   * Atur target posisi ke INFINITY agar motor bergerak dalam mode kecepatan (Velocity Control)
   */
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  /* Awal mulai: motor dalam keadaan diam */
  set_speeds(left_motor, right_motor, 0.0, 0.0);

  printf("[Controller] Dimulai: Siklus gerakan robot E-puck.\n");

  /*
   * Loop utama siklus pergerakan robot:
   * Maju 5s -> Diam 5s -> Mundur 5s -> Diam 5s -> Belok kiri -> Ulangi
   */
  while (true) {
    // 1. Maju selama 5 detik
    printf("[Gerak] Maju selama 5 detik...\n");
    set_speeds(left_motor, right_motor, SPEED, SPEED);
    if (!wait_seconds(5.0)) break;

    // 2. Berhenti selama 5 detik
    printf("[Gerak] Berhenti selama 5 detik...\n");
    set_speeds(left_motor, right_motor, 0.0, 0.0);
    if (!wait_seconds(5.0)) break;

    // 3. Mundur selama 5 detik
    printf("[Gerak] Mundur selama 5 detik...\n");
    set_speeds(left_motor, right_motor, -SPEED, -SPEED);
    if (!wait_seconds(5.0)) break;

    // 4. Berhenti selama 5 detik
    printf("[Gerak] Berhenti selama 5 detik...\n");
    set_speeds(left_motor, right_motor, 0.0, 0.0);
    if (!wait_seconds(5.0)) break;

    // 5. Belok kiri (motor kiri mundur / pelan, motor kanan maju)
    printf("[Gerak] Belok kiri...\n");
    set_speeds(left_motor, right_motor, -SPEED, SPEED);
    if (!wait_seconds(TURN_LEFT_DURATION)) break;

    // Setelah belok kiri selesai, siklus otomatis berulang kembali ke langkah 1
  }

  /* Bersihkan resource Webots sebelum program ditutup */
  wb_robot_cleanup();

  return 0;
}
