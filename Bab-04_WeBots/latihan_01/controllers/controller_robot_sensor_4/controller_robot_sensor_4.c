/*
 * File:          controller_robot_sensor_4.c
 * Description:   Controller Robot Sensor 4:
 *                - Mode Default: LURUS BLAS (MAX_SPEED, MAX_SPEED), kedua motor sama persis tanpa ngelengkung.
 *                - Koreksi Posisi:
 *                  * Jika ps2 terlalu dekat tembok kanan (> 165) -> serong kiri sebentar sampai aman, lalu LURUS BLAS.
 *                  * Jika ps2 terlalu jauh dari tembok kanan (< 68) -> serong kanan mendekat tembok sampai pas, lalu LURUS BLAS.
 *                - Belok Kanan: Jika tembok kanan sudah habis (ps2 < 45) -> belok kanan mengitari sudut.
 *                - Belok Kiri: Jika ada tembok di depan (ps0 / ps7 > 90) -> belok kiri menghindar.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

/*
 * Status / Mode Robot
 */
typedef enum {
  MODE_LURUS,          // LURUS BLAS (kiri = MAX_SPEED, kanan = MAX_SPEED)
  MODE_KOREKSI_KIRI,   // Terlalu nempel tembok kanan -> geser kiri sebentar
  MODE_KOREKSI_KANAN,  // Terlalu jauh dari tembok kanan -> geser kanan mendekat
  MODE_BELOK_KANAN,    // Tembok kanan habis -> belok kanan masuk lorong baru
  MODE_BELOK_KIRI      // Tembok di depan mentok -> belok kiri menghindar
} ModeRobot;

/*
 * Fungsi pembantu mengatur kecepatan kedua motor
 */
void set_speeds(WbDeviceTag left_motor, WbDeviceTag right_motor, double left_speed, double right_speed) {
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}

int main(int argc, char **argv) {
  wb_robot_init();

  /* Device motor roda E-puck */
  WbDeviceTag kanan_actuator = wb_robot_get_device("right wheel motor");
  WbDeviceTag kiri_actuator = wb_robot_get_device("left wheel motor");

  /* Device sensor jarak */
  WbDeviceTag ps0 = wb_robot_get_device("ps0"); // Depan kanan
  WbDeviceTag ps2 = wb_robot_get_device("ps2"); // Samping kanan (90°)
  WbDeviceTag ps7 = wb_robot_get_device("ps7"); // Depan kiri

  /* Aktifkan sensor jarak */
  wb_distance_sensor_enable(ps0, TIME_STEP);
  wb_distance_sensor_enable(ps2, TIME_STEP);
  wb_distance_sensor_enable(ps7, TIME_STEP);

  /* Mode kontrol kecepatan */
  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);

  set_speeds(kiri_actuator, kanan_actuator, 0.0, 0.0);

  ModeRobot mode = MODE_LURUS;
  bool pernah_lihat_tembok_kanan = false;

  printf("[Controller] Controller 4 Siap: LURUS BLAS!\n");

  /* Simulation loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* Baca nilai sensor */
    double ps0_val = wb_distance_sensor_get_value(ps0);
    double ps2_val = wb_distance_sensor_get_value(ps2);
    double ps7_val = wb_distance_sensor_get_value(ps7);

    // Tandai jika robot sudah pernah berada di samping tembok kanan
    if (ps2_val > 55.0) {
      pernah_lihat_tembok_kanan = true;
    }

    /* -------------------------------------------------------------
     * 1. CEK PRIORITAS UTAMA (TEMBOK DEPAN / TEMBOK KANAN HABIS)
     * ------------------------------------------------------------- */
    if (ps0_val > 90.0 || ps7_val > 90.0) {
      mode = MODE_BELOK_KIRI;
    } 
    else if (pernah_lihat_tembok_kanan && ps2_val < 45.0 && mode != MODE_BELOK_KIRI) {
      // Tembok kanan sudah habis -> Belok kanan
      mode = MODE_BELOK_KANAN;
    }

    /* -------------------------------------------------------------
     * 2. EKSEKUSI BERDASARKAN MODE
     * ------------------------------------------------------------- */
    switch (mode) {
      /* --- KASUS A: BELOK KIRI (Mentok di depan) --- */
      case MODE_BELOK_KIRI:
        set_speeds(kiri_actuator, kanan_actuator, -0.4 * MAX_SPEED, 0.4 * MAX_SPEED);
        if (ps0_val < 65.0 && ps7_val < 65.0) {
          printf("[Mode] Tembok depan bebas -> Gaspol LURUS BLAS!\n");
          mode = MODE_LURUS;
        }
        break;

      /* --- KASUS B: BELOK KANAN (Tembok kanan habis) --- */
      case MODE_BELOK_KANAN:
        // Belok kanan mengitari sudut (roda kiri laju, roda kanan lambat)
        set_speeds(kiri_actuator, kanan_actuator, MAX_SPEED, 0.15 * MAX_SPEED);
        // Selesai belok kanan saat ps2 kembali menangkap tembok baru di kanan
        if (ps2_val >= 70.0) {
          printf("[Mode] Tembok baru terdeteksi di kanan (ps2: %.1f) -> Gaspol LURUS BLAS!\n", ps2_val);
          mode = MODE_LURUS;
        }
        break;

      /* --- KASUS C: KOREKSI KIRI (Terlalu mepet tembok kanan) --- */
      case MODE_KOREKSI_KIRI:
        // Serong kiri sebentar untuk menjauh
        set_speeds(kiri_actuator, kanan_actuator, 0.6 * MAX_SPEED, MAX_SPEED);
        if (ps2_val <= 130.0) {
          printf("[Koreksi Selesai] Jarak aman (ps2: %.1f) -> Kembali LURUS BLAS!\n", ps2_val);
          mode = MODE_LURUS;
        }
        break;

      /* --- KASUS D: KOREKSI KANAN (Terlalu jauh dari tembok kanan) --- */
      case MODE_KOREKSI_KANAN:
        // Serong kanan sebentar untuk mendekat
        set_speeds(kiri_actuator, kanan_actuator, MAX_SPEED, 0.6 * MAX_SPEED);
        if (ps2_val >= 85.0) {
          printf("[Koreksi Selesai] Sudah pas (ps2: %.1f) -> Kembali LURUS BLAS!\n", ps2_val);
          mode = MODE_LURUS;
        }
        break;

      /* --- KASUS E: MODE LURUS (LURUS BLAS, NO BELOK-BELOK, NO NGELENGKUNG) --- */
      case MODE_LURUS:
      default:
        if (ps2_val > 165.0) {
          // Terlalu mepet ke tembok kanan -> Masuk mode koreksi kiri
          printf("[Koreksi] Mepet kanan (ps2: %.1f) -> Serong kiri sebentar...\n", ps2_val);
          mode = MODE_KOREKSI_KIRI;
        } 
        else if (pernah_lihat_tembok_kanan && ps2_val < 68.0 && ps2_val >= 45.0) {
          // Terlalu jauh dari tembok kanan -> Masuk mode koreksi kanan
          printf("[Koreksi] Terlalu jauh (ps2: %.1f) -> Serong kanan mendekat...\n", ps2_val);
          mode = MODE_KOREKSI_KANAN;
        } 
        else {
          // 100% LURUS BLAS! Kedua roda kecepatannya SAMA PERSIS!
          set_speeds(kiri_actuator, kanan_actuator, MAX_SPEED, MAX_SPEED);
        }
        break;
    }
  }

  wb_robot_cleanup();
  return 0;
}
