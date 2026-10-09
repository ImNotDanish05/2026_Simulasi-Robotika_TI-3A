/*
 * File:          controller_robot_sensor_3.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdbool.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define OBSTACLE_THRESHOLD 80.0 // Nilai sensor jarak saat mendekati tembok (standar Webots E-puck)

#define SPEED_TURN (0.5 * MAX_SPEED)
#define SPEED_FORWARD (MAX_SPEED)

// Durasi belok kiri dalam detik untuk putaran ~90 derajat
// (Perhitungan: 2.0 detik menghasilkan 225 derajat, maka 90/225 * 2.0 = 0.8 detik)
#define TURN_LEFT_DURATION 0.71

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


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag kanan_actuator = wb_robot_get_device("right wheel motor");
  WbDeviceTag kiri_actuator = wb_robot_get_device("left wheel motor");

  /* Inisialisasi sensor jarak ps0 (depan kanan), ps2 (samping kanan), dan ps7 (depan kiri) */
  WbDeviceTag ps0 = wb_robot_get_device("ps0");
  WbDeviceTag ps2 = wb_robot_get_device("ps2");
  WbDeviceTag ps7 = wb_robot_get_device("ps7");

  /* Aktifkan sensor jarak dengan sampling period TIME_STEP */
  wb_distance_sensor_enable(ps0, TIME_STEP);
  wb_distance_sensor_enable(ps2, TIME_STEP);
  wb_distance_sensor_enable(ps7, TIME_STEP);

  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);

  /* Awal mulai: robot diam */
  wb_motor_set_velocity(kanan_actuator, 0.0);
  wb_motor_set_velocity(kiri_actuator, 0.0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * 1. Read the sensors :
     */
    double ps0_value = wb_distance_sensor_get_value(ps0);
    double ps2_value = wb_distance_sensor_get_value(ps2);
    double ps7_value = wb_distance_sensor_get_value(ps7);

    /*
     * 2. Process sensor data:
     * Nilai sensor E-puck semakin besar saat mendekati objek.
     * Jika sensor depan (ps0, ps7) atau sensor sebelah kanan (ps2) > OBSTACLE_THRESHOLD,
     * robot mendeteksi halangan dan akan belok ke kiri untuk menghindar.
     */
    bool halangan_terdeteksi = (ps0_value > OBSTACLE_THRESHOLD) ||
                               (ps7_value > OBSTACLE_THRESHOLD) ||
                               (ps2_value > OBSTACLE_THRESHOLD);

    /*
     * 3. Send actuator commands:
     */
    if (halangan_terdeteksi) {
      printf("[Sensor] Halangan terdeteksi (ps0: %.1f, ps2_kanan: %.1f, ps7: %.1f) -> Belok Kiri\n",
             ps0_value, ps2_value, ps7_value);
      // Belok kiri (roda kiri mundur, roda kanan maju)
      set_speeds(kiri_actuator, kanan_actuator, -SPEED_TURN, SPEED_TURN);
      if (!wait_seconds(TURN_LEFT_DURATION)) break;
    } else {
      /* Tidak ada halangan: Maju lurus */
      wb_motor_set_velocity(kanan_actuator, SPEED_FORWARD);
      wb_motor_set_velocity(kiri_actuator, SPEED_FORWARD);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
