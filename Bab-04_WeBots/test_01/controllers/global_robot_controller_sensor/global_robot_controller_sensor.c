/*
 * File:          global_robot_controller_sensor.c
 * Description:   Global maze explorer & return home controller
 *
 *  EXPLORE (pergi):
 *   OPEN      : tidak ada tembok -> lurus sampai nemu tembok
 *   FOLLOW    : jalan lurus sambil pantau (jarak + arah perubahan jarak)
 *   LOST_FWD  : tembok tumpuan hilang -> maju sedikit biar badan lewat sudut
 *   LOST_ARC  : pivot ketat ke arah tembok sampai tembok ketemu lagi
 *   TURN      : depan buntu -> putar di tempat
 *   Tiap remah jalur dicatat dan loop otomatis dipangkas.
 *
 *  RETURN_UTURN (saat lihat balok merah):
 *   Putar balik 180 derajat di tempat ke arah yang lebih lapang secara tuntas.
 *
 *  RETURN_HOME (pulang):
 *   Dual Wall-Following simetris (Left-Wall Follower): menyusuri dinding sebaliknya
 *   murni berbasis sensor jarak (ps0..ps7), kebal terhadap selip roda / odometri,
 *   dan otomatis berhenti saat mendeteksi kembali area start.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define CRUISE_SPEED 5.0
#define RETURN_SPEED 4.5
#define TURN_SPEED 3.0

/* Geometri e-puck (kalau odometry meleset, kalibrasi dua angka ini) */
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052

/* ---------- Sensor jarak ---------- */
#define WALL_DETECT 80.0
#define FRONT_ARC_DETECT 250.0
#define CORNER_NEAR 400.0
#define OPP_NEAR 400.0
#define TURN_EXTRA_STEPS 1

/* ---------- FOLLOW ---------- */
#define SIDE_TARGET 150.0
#define KP_WALL 0.012
#define KD_WALL 0.03
#define E_DEADBAND 30.0
#define D_DEADBAND 8.0
#define STEER_LIMIT 3.0

/* ---------- Tembok tumpuan hilang ---------- */
#define LOST_FWD_STEPS 3
#define ARC_OUT CRUISE_SPEED
#define ARC_IN (CRUISE_SPEED * 0.12)
#define ARC_MAX_TURN 2.0

/* ---------- Kamera / merah ---------- */
#define RED_PIXEL_THRESHOLD 15
#define RED_CONFIRM_FRAMES 3

/* ---------- Breadcrumb path ---------- */
#define MAX_PATH 4000
#define CRUMB_DIST 0.03
#define LOOP_RADIUS 0.04
#define LOOP_MIN_GAP 20
#define LOOKAHEAD_DIST 0.09   /* Jarak pandang ke depan (9 cm) untuk tracking jalur pulang */

/* ---------- Anti-loop eksplorasi ---------- */
#define GRID_N 160
#define CELL 0.05
#define NO_NEW_CELL_LIMIT 400
#define FORCE_STRAIGHT_STEPS 40

typedef enum { SIDE_NONE, SIDE_LEFT, SIDE_RIGHT } Side;
typedef struct { double x, y; Side side; } Crumb; /* side = tembok tumpuan saat remah ini direkam */
typedef enum { EXPLORE, RETURN_UTURN, RETURN_HOME, RETURN_ENTER_ROOM, DONE } State;
typedef enum { M_OPEN, M_TURN, M_FOLLOW, M_LOST_FWD, M_LOST_ARC } Mode;

static Crumb path[MAX_PATH];
static int path_len = 0;
static bool visited[GRID_N][GRID_N];

static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static void set_speeds(WbDeviceTag lm, WbDeviceTag rm, double l, double r) {
  wb_motor_set_velocity(lm, clampd(l, -MAX_SPEED, MAX_SPEED));
  wb_motor_set_velocity(rm, clampd(r, -MAX_SPEED, MAX_SPEED));
}

static bool mark_visited(double x, double y) {
  int ix = (int)floor(x / CELL) + GRID_N / 2;
  int iy = (int)floor(y / CELL) + GRID_N / 2;
  if (ix < 0 || iy < 0 || ix >= GRID_N || iy >= GRID_N) return false;
  if (visited[ix][iy]) return false;
  visited[ix][iy] = true;
  return true;
}

static void path_record(double x, double y, Side side) {
  if (path_len == 0) {
    path[0].x = x;
    path[0].y = y;
    path[0].side = SIDE_NONE;
    path_len = 1;
    return;
  }
  if (hypot(x - path[path_len - 1].x, y - path[path_len - 1].y) < CRUMB_DIST) return;

  for (int i = 0; i < path_len - LOOP_MIN_GAP; i++) {
    if (hypot(x - path[i].x, y - path[i].y) < LOOP_RADIUS) {
      path_len = i + 1; /* buang loop / dead end */
      return;
    }
  }
  if (path_len < MAX_PATH) {
    path[path_len].x = x;
    path[path_len].y = y;
    path[path_len].side = side;
    path_len++;
  }
}

static double dist_to(int idx, double x, double y) {
  return hypot(path[idx].x - x, path[idx].y - y);
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_enc = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_enc = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_enc, TIME_STEP);
  wb_position_sensor_enable(right_enc, TIME_STEP);

  WbDeviceTag ps[8];
  char ps_name[4];
  for (int i = 0; i < 8; i++) {
    sprintf(ps_name, "ps%d", i);
    ps[i] = wb_robot_get_device(ps_name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  int cam_w = wb_camera_get_width(camera);
  int cam_h = wb_camera_get_height(camera);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  set_speeds(left_motor, right_motor, 0.0, 0.0);

  wb_robot_step(TIME_STEP);

  double x = 0.0, y = 0.0, theta = 0.0; /* odometry: CCW positif */
  double prev_l = wb_position_sensor_get_value(left_enc);
  double prev_r = wb_position_sensor_get_value(right_enc);

  State state = EXPLORE;
  Mode mode = M_OPEN;
  Side follow_side = SIDE_NONE;

  double follow_prev = 0.0;
  bool follow_first = true;
  int lost_cnt = 0;
  double arc_turned = 0.0;
  int turn_dir = 0; /* 1 = kanan, -1 = kiri */
  int turn_extra = 0;

  int red_frames = 0;
  int ret_idx = 0;
  int no_new_cell = 0;
  int force_straight = 0;
  double uturn_angle = 0.0;
  int uturn_dir = 1; /* 1 = kanan, -1 = kiri */
  Side return_side = SIDE_LEFT;
  int return_steps = 0;
  int enter_room_steps = 0;

  path_record(0.0, 0.0, SIDE_NONE);
  mark_visited(0.0, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /* ===== 1. Odometry ===== */
    double cur_l = wb_position_sensor_get_value(left_enc);
    double cur_r = wb_position_sensor_get_value(right_enc);
    double dl = (cur_l - prev_l) * WHEEL_RADIUS;
    double dr = (cur_r - prev_r) * WHEEL_RADIUS;
    prev_l = cur_l;
    prev_r = cur_r;
    double ds = (dl + dr) / 2.0;
    double dth = (dr - dl) / AXLE_LENGTH;
    x += ds * cos(theta + dth / 2.0);
    y += ds * sin(theta + dth / 2.0);
    theta = normalize_angle(theta + dth);

    /* ===== 2. Sensor jarak ===== */
    double v[8];
    for (int i = 0; i < 8; i++) v[i] = wb_distance_sensor_get_value(ps[i]);

    double ls = 0.0, rs = 0.0;

    if (state == EXPLORE) {
      /* ===== 3. Deteksi balok merah ===== */
      const unsigned char *image = wb_camera_get_image(camera);
      int red_pixels = 0;
      if (image) {
        for (int cx = 0; cx < cam_w; cx++) {
          for (int cy = 0; cy < cam_h; cy++) {
            int r = wb_camera_image_get_red(image, cam_w, cx, cy);
            int g = wb_camera_image_get_green(image, cam_w, cx, cy);
            int b = wb_camera_image_get_blue(image, cam_w, cx, cy);
            if (r > 140 && g < 65 && b < 65 && (r - g > 80) && (r - b > 80)) red_pixels++;
          }
        }
      }
      if (red_pixels > RED_PIXEL_THRESHOLD) red_frames++;
      else red_frames = 0;

      if (red_frames >= RED_CONFIRM_FRAMES) {
        state = RETURN_UTURN;
        uturn_angle = 0.0;
        /* Dinding pulang: kebalikan dari dinding penelusuran pergi */
        return_side = (follow_side == SIDE_RIGHT) ? SIDE_LEFT : SIDE_RIGHT;
        if (follow_side == SIDE_NONE) return_side = SIDE_LEFT;

        /* Putar menjauhi rintangan samping yang paling dekat */
        double left_obst = v[5] + v[6];
        double right_obst = v[2] + v[1];
        uturn_dir = (left_obst > right_obst) ? 1 : -1; /* 1 = kanan (CW), -1 = kiri (CCW) */
        printf("[MERAH] Target ditemukan! Memulai U-Turn untuk menyusuri dinding %s saat pulang...\n",
               (return_side == SIDE_LEFT) ? "KIRI" : "KANAN");
        set_speeds(left_motor, right_motor, 0.0, 0.0);
        continue;
      }

      /* ===== 4. Rekam jalur (+ label tembok) dan anti-loop ===== */
      path_record(x, y, (mode == M_OPEN) ? SIDE_NONE : follow_side);
      if (mark_visited(x, y)) no_new_cell = 0;
      else no_new_cell++;
      if (no_new_cell > NO_NEW_CELL_LIMIT) {
        force_straight = FORCE_STRAIGHT_STEPS;
        no_new_cell = 0;
        if (mode != M_TURN) mode = M_OPEN;
        printf("[ANTI-LOOP] Muter-muter, paksa lurus\n");
      }

    } else if (state == RETURN_UTURN) {
      /* ===== 3a. Putar balik 180 derajat di tempat secara tuntas ===== */
      uturn_angle += fabs(dth);

      if (uturn_dir == 1) {
        /* Putar ke kanan (CW): roda kiri maju, roda kanan mundur */
        ls = TURN_SPEED;
        rs = -TURN_SPEED;
      } else {
        /* Putar ke kiri (CCW): roda kiri mundur, roda kanan maju */
        ls = -TURN_SPEED;
        rs = TURN_SPEED;
      }

      /* Sensor dinding pulang yang harus dideteksi */
      double target_wall = (return_side == SIDE_LEFT) ? v[5] : v[2];
      bool front_clear = (v[0] < FRONT_ARC_DETECT && v[7] < FRONT_ARC_DETECT);

      /* Putar sampai minimal 180 derajat fisik (~3.20 rad) dan dinding pulang terdeteksi, atau batas 3.60 rad */
      if ((uturn_angle >= 3.20 && target_wall > WALL_DETECT && front_clear) || uturn_angle >= 3.60) {
        state = RETURN_HOME;
        follow_side = return_side;
        follow_first = true;
        mode = M_FOLLOW;
        return_steps = 0;
        printf("[U-TURN] Selesai (%.2f rad)! Beralih ke RETURN_HOME (Dual Wall-Following dinding %s).\n",
               uturn_angle, (follow_side == SIDE_LEFT) ? "KIRI" : "KANAN");
      }
      set_speeds(left_motor, right_motor, ls, rs);
      continue;

    } else if (state == RETURN_HOME) {
      /* ===== 3b. Monitor kepulangan ke titik start ===== */
      return_steps++;
      double dist_start = hypot(x, y);

      /* Saat robot mendekati pintu masuk ruangan start (< 22 cm) */
      if (return_steps > 120 && dist_start < 0.22) {
        state = RETURN_ENTER_ROOM;
        enter_room_steps = 0;
        printf("[PINTU MASUK] Terdeteksi di pintu masuk (d=%.2f m). Melanjutkan masuk ke dalam ruangan...\n", dist_start);
      }
      /* Mengalir ke FSM penelusur dinding (section 5, 6, 7) */

    } else if (state == RETURN_ENTER_ROOM) {
      /* ===== 3c. Masuk sepenuhnya ke dalam ruangan start ===== */
      enter_room_steps++;
      double dist_start = hypot(x, y);
      double front_max = fmax(v[0], v[7]);

      /* Berhenti jika sudah berada di dalam ruangan:
         1. Jarak ke titik asal sangat dekat (d < 0.08 m)
         2. Atau sudah maju cukup dalam melewati kusen pintu (enter_room_steps >= 40 atau ~25 cm)
         3. Atau sensor depan mendeteksi dinding ruangan (front_max > 250.0) */
      if (dist_start < 0.08 || enter_room_steps >= 40 || front_max > 250.0) {
        state = DONE;
        set_speeds(left_motor, right_motor, 0.0, 0.0);
        printf("[SELESAI] Robot SUDAH MASUK SEPENUHNYA ke dalam ruangan start (d=%.2f m, step_masuk=%d)! Berhenti sempurna.\n",
               dist_start, enter_room_steps);
        continue;
      }
      /* Mengalir ke FSM penelusur dinding agar bodi tetap aman dari benturan */

    } else if (state == DONE) {
      set_speeds(left_motor, right_motor, 0.0, 0.0);
      continue;
    }

    if (state == EXPLORE || state == RETURN_HOME || state == RETURN_ENTER_ROOM) {
      /* ===== 5. Deteksi tembok ===== */
      double front_thr = (mode == M_LOST_ARC) ? FRONT_ARC_DETECT : WALL_DETECT;
      bool front = (v[0] > front_thr || v[7] > front_thr);

      double sl_any = fmax(v[5], v[6]);
      double sr_any = fmax(v[2], v[1]);
      bool left_wall = v[5] > WALL_DETECT;
      bool right_wall = v[2] > WALL_DETECT;

      /* ===== 6. Depan buntu -> TURN ===== */
      if (front) {
        if (mode != M_TURN) {
          bool lb = sl_any > WALL_DETECT;
          bool rb = sr_any > WALL_DETECT;
          if (lb && !rb) turn_dir = 1;
          else if (rb && !lb) turn_dir = -1;
          else if (lb && rb) turn_dir = (sl_any > sr_any) ? 1 : -1;
          else turn_dir = (v[7] > v[0]) ? 1 : -1;
          mode = M_TURN;
        }
        turn_extra = TURN_EXTRA_STEPS;
      } else if (mode == M_TURN) {
        if (turn_extra > 0) {
          turn_extra--;
        } else {
          mode = M_OPEN;
          turn_dir = 0;
        }
      }

      /* ===== 7. Mesin mode ===== */
      switch (mode) {
        case M_TURN: {
          ls = turn_dir * TURN_SPEED;
          rs = -turn_dir * TURN_SPEED;
          break;
        }

        case M_OPEN: {
          if (force_straight > 0) {
            force_straight--;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          } else if (left_wall || right_wall) {
            Side s;
            if (state == RETURN_HOME) {
              /* Saat pulang, prioritaskan mengunci dinding return_side */
              s = (return_side == SIDE_LEFT && left_wall) ? SIDE_LEFT :
                  ((return_side == SIDE_RIGHT && right_wall) ? SIDE_RIGHT :
                  (left_wall ? SIDE_LEFT : SIDE_RIGHT));
            } else if (left_wall && right_wall) {
              if (follow_side != SIDE_NONE) s = follow_side;
              else s = (v[5] > v[2]) ? SIDE_LEFT : SIDE_RIGHT;
            } else {
              s = left_wall ? SIDE_LEFT : SIDE_RIGHT;
            }
            follow_side = s;
            follow_first = true;
            mode = M_FOLLOW;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          } else {
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          }
          break;
        }

        case M_FOLLOW: {
          bool right = (follow_side == SIDE_RIGHT);
          double sv = right ? v[2] : v[5];
          double dv = right ? v[1] : v[6];
          double ov = right ? v[5] : v[2];
          double wall_sign = right ? 1.0 : -1.0;

          if (sv < WALL_DETECT) {
            mode = M_LOST_FWD;
            lost_cnt = 0;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
            break;
          }

          if (follow_first) {
            follow_prev = sv;
            follow_first = false;
          }

          double e = sv - SIDE_TARGET;
          if (dv > CORNER_NEAR) e += (dv - CORNER_NEAR);
          if (ov > OPP_NEAR) e -= 0.5 * (ov - OPP_NEAR);
          double d = sv - follow_prev;
          follow_prev = sv;

          if (fabs(e) < E_DEADBAND) e = 0.0;
          if (fabs(d) < D_DEADBAND) d = 0.0;

          double steer = -wall_sign * (KP_WALL * e + KD_WALL * d);
          steer = clampd(steer, -STEER_LIMIT, STEER_LIMIT);
          ls = CRUISE_SPEED + steer;
          rs = CRUISE_SPEED - steer;
          break;
        }

        case M_LOST_FWD: {
          bool right = (follow_side == SIDE_RIGHT);
          double sv = right ? v[2] : v[5];
          if (sv > WALL_DETECT) {
            mode = M_FOLLOW;
            follow_first = true;
          } else if (++lost_cnt > LOST_FWD_STEPS) {
            mode = M_LOST_ARC;
            arc_turned = 0.0;
          }
          ls = CRUISE_SPEED;
          rs = CRUISE_SPEED;
          break;
        }

        case M_LOST_ARC: {
          bool right = (follow_side == SIDE_RIGHT);
          double sv = right ? v[2] : v[5];
          arc_turned += fabs(dth);

          if (sv > WALL_DETECT) {
            mode = M_FOLLOW;
            follow_first = true;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          } else if (arc_turned > ARC_MAX_TURN) {
            mode = M_OPEN;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          } else if (right) {
            ls = ARC_OUT;
            rs = ARC_IN;
          } else {
            ls = ARC_IN;
            rs = ARC_OUT;
          }
          break;
        }
      }
    }

    set_speeds(left_motor, right_motor, ls, rs);
  }

  wb_robot_cleanup();
  return 0;
}