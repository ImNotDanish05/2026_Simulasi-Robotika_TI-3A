/*
 * File:          controller_robot_sensor_4.c
 * Description:   Right Wall Followear + berhenti saat merah menyentuh tepi bawah kamera
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define OBSTACLE_THRESHOLD 80.0

#define BOTTOM_ROWS 2          // Jumlah baris paling bawah yang dicek
#define RED_PIXEL_THRESHOLD 5  // Minimal piksel merah di baris bawah itu

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

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  int cam_width = wb_camera_get_width(camera);
  int cam_height = wb_camera_get_height(camera);

  wb_motor_set_position(kanan_actuator, INFINITY);
  wb_motor_set_position(kiri_actuator, INFINITY);
  set_speeds(kiri_actuator, kanan_actuator, 0.0, 0.0);

  wb_robot_step(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (int i = 0; i < 8; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    /* Hitung piksel merah HANYA di baris paling bawah kamera */
    const unsigned char *image = wb_camera_get_image(camera);
    int red_bottom_pixels = 0;
    if (image) {
      for (int x = 0; x < cam_width; x++) {
        for (int y = cam_height - BOTTOM_ROWS; y < cam_height; y++) {
          int r = wb_camera_image_get_red(image, cam_width, x, y);
          int g = wb_camera_image_get_green(image, cam_width, x, y);
          int b = wb_camera_image_get_blue(image, cam_width, x, y);

          if (r > 140 && g < 65 && b < 65 && (r - g > 80) && (r - b > 80)) {
            red_bottom_pixels++;
          }
        }
      }
    }

    bool red_detected = (red_bottom_pixels > RED_PIXEL_THRESHOLD);

    bool front_wall   = (ps_values[0] > OBSTACLE_THRESHOLD) || (ps_values[7] > OBSTACLE_THRESHOLD);
    bool right_wall   = ps_values[2] > OBSTACLE_THRESHOLD;
    bool right_corner = ps_values[1] > OBSTACLE_THRESHOLD;

    double left_speed  = MAX_SPEED;
    double right_speed = MAX_SPEED;

    if (red_detected) {
      if (!wait_seconds(1.5)) break;
      left_speed  = 0.0;
      right_speed = 0.0;
      printf("[Kamera] Merah menyentuh tepi bawah (%d piksel) -> BERHENTI!\n", red_bottom_pixels);
    } else if (front_wall) {
      left_speed  = -MAX_SPEED;
      right_speed =  MAX_SPEED;
    } else {
      if (right_wall) {
        left_speed  = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        left_speed  = MAX_SPEED;
        right_speed = MAX_SPEED / 8.0;
      }
      if (right_corner) {
        left_speed  = MAX_SPEED / 8.0;
        right_speed = MAX_SPEED;
      }
    }

    set_speeds(kiri_actuator, kanan_actuator, left_speed, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}