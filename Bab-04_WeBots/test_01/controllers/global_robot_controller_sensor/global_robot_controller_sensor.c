/*
 * File:          controller_robot_sensor_7.c
 * Description:   Random maze explorer v3 (tanpa berhenti-nengok)
 *
 *  Mode eksplorasi:
 *   OPEN      : tidak ada tembok -> lurus sampai nemu tembok
 *   FOLLOW    : jalan lurus sambil pantau (jarak + arah perubahan jarak)
 *               dekat/mau nabrak -> geser menjauh, menjauh -> geser mendekat, pas -> lurus
 *   LOST_FWD  : tembok tumpuan hilang -> maju sedikit biar badan lewat sudut
 *   LOST_ARC  : pivot ketat ke arah tembok sampai tembok ketemu lagi, lalu LANGSUNG lanjut
 *   TURN      : depan buntu -> putar di tempat
 *
 *  Merah terdeteksi -> pulang ke titik start lewat breadcrumb path, lalu berhenti.
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

/* Geometri e-puck */
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052

/* ---------- Sensor jarak ---------- */
#define WALL_DETECT 80.0        /* di atas ini = ada tembok */
#define FRONT_ARC_DETECT 250.0  /* ambang depan saat tikungan keluar */
#define CORNER_NEAR 400.0       /* sensor 45 derajat di atas ini = terlalu dekat */
#define OPP_NEAR 400.0          /* tembok sisi seberang terlalu dekat */
#define TURN_EXTRA_STEPS 1      /* muter sebentar setelah depan bebas */

/* ---------- FOLLOW (jalan lurus sambil pantau) ---------- */
#define SIDE_TARGET 150.0       /* nilai sensor samping yang diinginkan */
#define KP_WALL 0.012           /* koreksi karena jarak */
#define KD_WALL 0.03            /* koreksi karena arah perubahan jarak (ngelurusin sambil jalan) */
#define E_DEADBAND 30.0         /* selama selisih jarak kecil -> dianggap pas */
#define D_DEADBAND 8.0          /* selama perubahan jarak kecil -> dianggap lurus */
#define STEER_LIMIT 3.0

/* ---------- Tembok tumpuan hilang (belokan keluar) ---------- */
#define LOST_FWD_STEPS 3
#define ARC_OUT CRUISE_SPEED
#define ARC_IN (CRUISE_SPEED * 0.12)
#define ARC_MAX_TURN 2.0        /* rad (~115 derajat); lebih dari ini dianggap area kosong */

/* ---------- Kamera / merah ---------- */
#define RED_PIXEL_THRESHOLD 15
#define RED_CONFIRM_FRAMES 3

/* ---------- Breadcrumb path ---------- */
#define MAX_PATH 4000
#define CRUMB_DIST 0.03
#define LOOP_RADIUS 0.04
#define LOOP_MIN_GAP 20
#define WP_REACH 0.025
#define RET_AVOID 600.0

/* ---------- Anti-loop eksplorasi ---------- */
#define GRID_N 160
#define CELL 0.05
#define NO_NEW_CELL_LIMIT 400
#define FORCE_STRAIGHT_STEPS 40

typedef struct { double x, y; } Point;
typedef enum { EXPLORE, RETURN_HOME, DONE } State;
typedef enum { SIDE_NONE, SIDE_LEFT, SIDE_RIGHT } Side;
typedef enum { M_OPEN, M_TURN, M_FOLLOW, M_LOST_FWD, M_LOST_ARC } Mode;

static Point path[MAX_PATH];
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

static void path_record(double x, double y) {
  if (path_len == 0) {
    path[0].x = x;
    path[0].y = y;
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
    path_len++;
  }
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

  path_record(0.0, 0.0);
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
        if (hypot(x - path[path_len - 1].x, y - path[path_len - 1].y) > 0.005 && path_len < MAX_PATH) {
          path[path_len].x = x;
          path[path_len].y = y;
          path_len++;
        }
        ret_idx = path_len - 1;
        state = RETURN_HOME;
        printf("[MERAH] Ketemu! Jalur pulang: %d waypoint\n", path_len);
        continue;
      }

      /* ===== 4. Rekam jalur + anti-loop ===== */
      path_record(x, y);
      if (mark_visited(x, y)) no_new_cell = 0;
      else no_new_cell++;
      if (no_new_cell > NO_NEW_CELL_LIMIT) {
        force_straight = FORCE_STRAIGHT_STEPS;
        no_new_cell = 0;
        if (mode != M_TURN) mode = M_OPEN;
        printf("[ANTI-LOOP] Muter-muter, paksa lurus\n");
      }

      /* ===== 5. Deteksi tembok ===== */
      double front_thr = (mode == M_LOST_ARC) ? FRONT_ARC_DETECT : WALL_DETECT;
      bool front = (v[0] > front_thr || v[7] > front_thr);

      double sl_any = fmax(v[5], v[6]);
      double sr_any = fmax(v[2], v[1]);
      bool left_wall = v[5] > WALL_DETECT;  /* sensor samping murni */
      bool right_wall = v[2] > WALL_DETECT;

      /* ===== 6. Depan buntu -> TURN ===== */
      if (front) {
        if (mode != M_TURN) {
          bool lb = sl_any > WALL_DETECT;
          bool rb = sr_any > WALL_DETECT;
          if (lb && !rb) turn_dir = 1;            /* kiri tertutup -> belok kanan */
          else if (rb && !lb) turn_dir = -1;      /* kanan tertutup -> belok kiri */
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
            /* Nemu tembok -> langsung FOLLOW (tanpa berhenti) */
            Side s;
            if (left_wall && right_wall) {
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
            /* Tidak ada tembok -> lurus sampai nemu tembok */
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          }
          break;
        }

        case M_FOLLOW: {
          bool right = (follow_side == SIDE_RIGHT);
          double sv = right ? v[2] : v[5];  /* sensor samping tembok tumpuan */
          double dv = right ? v[1] : v[6];  /* sensor serong tembok tumpuan */
          double ov = right ? v[5] : v[2];  /* sensor samping seberang */
          double wall_sign = right ? 1.0 : -1.0;

          if (sv < WALL_DETECT) {
            /* Tembok tumpuan hilang (mis. sudut luar) */
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

          /* e > 0: terlalu dekat tembok tumpuan. d > 0: sedang mendekat */
          double e = sv - SIDE_TARGET;
          if (dv > CORNER_NEAR) e += (dv - CORNER_NEAR);      /* sudut mau nabrak */
          if (ov > OPP_NEAR) e -= 0.5 * (ov - OPP_NEAR);      /* seberang terlalu dekat */
          double d = sv - follow_prev;
          follow_prev = sv;

          if (fabs(e) < E_DEADBAND) e = 0.0;
          if (fabs(d) < D_DEADBAND) d = 0.0;

          /* steer positif = belok kanan. Tembok kanan & terlalu dekat -> belok kiri */
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
            mode = M_FOLLOW; /* cuma celah kecil, tembok lanjut */
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
            /* Tembok ketemu lagi -> langsung lanjut FOLLOW */
            mode = M_FOLLOW;
            follow_first = true;
            ls = CRUISE_SPEED;
            rs = CRUISE_SPEED;
          } else if (arc_turned > ARC_MAX_TURN) {
            /* Tidak ada tembok -> area kosong, lurus */
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

    } else if (state == RETURN_HOME) {
      /* ===== Pulang lewat breadcrumb ===== */
      while (ret_idx >= 0 && hypot(path[ret_idx].x - x, path[ret_idx].y - y) < WP_REACH) ret_idx--;

      if (ret_idx < 0) {
        state = DONE;
        ls = 0.0;
        rs = 0.0;
        printf("[SELESAI] Sudah di titik start. Berhenti.\n");
      } else {
        double dx = path[ret_idx].x - x;
        double dy = path[ret_idx].y - y;
        double err = normalize_angle(atan2(dy, dx) - theta); /* + = target di kiri */

        if (fabs(err) > 0.6) {
          double s = (err > 0) ? TURN_SPEED : -TURN_SPEED;
          ls = -s;
          rs = s;
        } else {
          double push = 0.0;
          if (v[5] > RET_AVOID || v[6] > RET_AVOID) push += 1.5;
          if (v[2] > RET_AVOID || v[1] > RET_AVOID) push -= 1.5;
          ls = RETURN_SPEED - 3.0 * err + push;
          rs = RETURN_SPEED + 3.0 * err - push;
        }
      }
    } else {
      ls = 0.0;
      rs = 0.0;
    }

    set_speeds(left_motor, right_motor, ls, rs);
  }

  wb_robot_cleanup();
  return 0;
}