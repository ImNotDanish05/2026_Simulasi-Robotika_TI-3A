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
  // Sebelah kanan
  WbDeviceTag ps0 = wb_robot_get_device("ps0"); // Depan kanan (0°)
  WbDeviceTag ps1 = wb_robot_get_device("ps1"); // Samping kanan (45°)
  WbDeviceTag ps2 = wb_robot_get_device("ps2"); // Samping kanan (90°)
  WbDeviceTag ps3 = wb_robot_get_device("ps3"); // Belakang kanan (135°)

  // Sebelah kiri
  WbDeviceTag ps7 = wb_robot_get_device("ps7"); // Depan kiri (0°)
  WbDeviceTag ps6 = wb_robot_get_device("ps6"); // Samping kiri (45°)
  WbDeviceTag ps5 = wb_robot_get_device("ps5"); // Samping kiri (90°)
  WbDeviceTag ps4 = wb_robot_get_device("ps4"); // Belakang kiri (135°)

  /* Aktifkan sensor jarak */
  wb_distance_sensor_enable(ps0, TIME_STEP);
  wb_distance_sensor_enable(ps1, TIME_STEP);
  wb_distance_sensor_enable(ps2, TIME_STEP);
  wb_distance_sensor_enable(ps3, TIME_STEP);
  wb_distance_sensor_enable(ps4, TIME_STEP);
  wb_distance_sensor_enable(ps5, TIME_STEP);
  wb_distance_sensor_enable(ps6, TIME_STEP);
  wb_distance_sensor_enable(ps7, TIME_STEP);

  /* Mode kontrol kecepatan */
  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);

  set_speeds(kiri_actuator, kanan_actuator, 0.0, 0.0);

  /* Simulation loop */
  while (wb_robot_step(TIME_STEP) != -1) {
  }

  wb_robot_cleanup();
  return 0;
}
