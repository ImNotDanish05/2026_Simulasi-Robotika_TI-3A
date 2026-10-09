/*
 * File:          controller_robot_sensor_4.c
 * Description:   Controller Robot Sensor 4 - Right Wall Follower
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define OBSTACLE_THRESHOLD 80.0

void set_speeds(WbDeviceTag left_motor, WbDeviceTag right_motor, double left_speed, double right_speed) {
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag kanan_actuator = wb_robot_get_device("right wheel motor");
  WbDeviceTag kiri_actuator = wb_robot_get_device("left wheel motor");

  WbDeviceTag ps[8];
  char ps_name[4];
  for (int i = 0; i < 8; i++) {
    sprintf(ps_name, "ps%d", i);
    ps[i] = wb_robot_get_device(ps_name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);
  set_speeds(kiri_actuator, kanan_actuator, 0.0, 0.0);

  wb_robot_step(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (int i = 0; i < 8; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    /* Deteksi dinding */
    bool front_wall   = (ps_values[0] > OBSTACLE_THRESHOLD) || (ps_values[7] > OBSTACLE_THRESHOLD);
    bool right_wall   = ps_values[2] > OBSTACLE_THRESHOLD;  /* samping kanan 90° */
    bool right_corner = ps_values[1] > OBSTACLE_THRESHOLD;  /* serong kanan 45° */

    double left_speed  = MAX_SPEED;
    double right_speed = MAX_SPEED;

    if (front_wall) {
      /* Buntu di depan -> putar kiri di tempat */
      left_speed  = -MAX_SPEED;
      right_speed =  MAX_SPEED;
    } else {
      if (right_wall) {
        /* Dinding kanan ada -> jalan lurus */
        left_speed  = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        /* Dinding kanan hilang -> belok kanan pelan buat cari dinding lagi */
        left_speed  = MAX_SPEED;
        right_speed = MAX_SPEED / 8.0;
      }
      if (right_corner) {
        /* Terlalu dekat dinding kanan -> geser sedikit ke kiri */
        left_speed  = MAX_SPEED / 8.0;
        right_speed = MAX_SPEED;
      }
    }

    set_speeds(kiri_actuator, kanan_actuator, left_speed, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}