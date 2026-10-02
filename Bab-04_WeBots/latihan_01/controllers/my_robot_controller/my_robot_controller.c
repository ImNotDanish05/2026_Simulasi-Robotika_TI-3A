/*
 * File:          my_robot_controller.c
 * Description:   Controller dasar robot E-puck di Webots
 */

#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
  /* Inisialisasi library Webots */
  wb_robot_init();

  /* Inisialisasi aktuator motor E-puck */
  WbDeviceTag kanan_actuator = wb_robot_get_device("right wheel motor");
  WbDeviceTag kiri_actuator = wb_robot_get_device("left wheel motor");

  /*
   * CARA 1: ROBOT BERJALAN MAJU TERUS (Velocity Control)
   * Set target posisi ke INFINITY agar motor bisa dikontrol kecepatannya
   */
  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);

  /* Set kecepatan putaran roda (misal: setengah dari kecepatan maksimal) */
  wb_motor_set_velocity(kanan_actuator, 0.5 * MAX_SPEED);
  wb_motor_set_velocity(kiri_actuator, 0.5 * MAX_SPEED);

  /*
   * (Opsional) CARA 2: JIKA MODUL MEMINTA TARGET POSISI TERTENTU (Position Control):
   * Jika di modul praktikum kamu diminta berputar beberapa radian saja lalu berhenti,
   * komentari baris di atas dan aktifkan dua baris di bawah ini:
   *
   * wb_motor_set_position(kanan_actuator, 5.0);
   * wb_motor_set_position(kiri_actuator, 5.0);
   */

  /* Simulation loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* Tempat membaca sensor atau mengubah kecepatan saat simulasi berjalan */
  }

  /* Bersihkan resource Webots */
  wb_robot_cleanup();

  return 0;
}
