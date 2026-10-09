/*
 * File:          controller_robot_sensor.c
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

  /* Inisialisasi sensor jarak ps0 (depan kanan) dan ps7 (depan kiri) */
  WbDeviceTag ps0 = wb_robot_get_device("ps0");
  WbDeviceTag ps7 = wb_robot_get_device("ps7");

  /* Aktifkan sensor jarak dengan sampling period TIME_STEP */
  wb_distance_sensor_enable(ps0, TIME_STEP);
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
    double ps7_value = wb_distance_sensor_get_value(ps7);

    /*
     * 2. Process sensor data:
     * Nilai sensor E-puck semakin besar saat mendekati objek.
     * Jika ps0 atau ps7 > OBSTACLE_THRESHOLD, berarti ada tembok di depan.
     */
    bool dekat_tembok = (ps0_value > OBSTACLE_THRESHOLD) || (ps7_value > OBSTACLE_THRESHOLD);

    /*
     * 3. Send actuator commands:
     */
    if (dekat_tembok) {
      /* Dekat dengan tembok: Berhenti */
      wb_motor_set_velocity(kanan_actuator, 0.0);
      wb_motor_set_velocity(kiri_actuator, 0.0);
    } else {
      /* Tidak ada tembok di depan: Maju */
      wb_motor_set_velocity(kanan_actuator, 0.5 * MAX_SPEED);
      wb_motor_set_velocity(kiri_actuator, 0.5 * MAX_SPEED);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
