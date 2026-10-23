/*
 * global_robot_controller_sensor.c
 *
 * E-puck maze challenge (Webots R2025a, world: test_01.wbt)
 *
 *  1. Navigasi odometri lewat waypoint menuju pintu Room 2
 *  2. SCAN   : putar di tempat sampai kamera melihat balok HIJAU
 *  3. APPROACH: visual servoing (kamera) + ps0/ps7 untuk berhenti
 *  4. STOP   : diam sebentar
 *  5. BLINK  : led5 nyala 5 kali
 *  6. BACK   : kembali lewat waypoint yang sama (dibalik)
 *  7. PARK   : masuk Room 1, posisi & heading sama persis dengan start
 *
 * Odometri: posisi dari encoder roda, HEADING dari gyro (kebal slip roda).
 *   Kalau gyro terdeteksi tidak konsisten dengan encoder -> otomatis fallback ke encoder.
 *
 * Parameter robot diambil dari E-puck.proto:
 *   radius roda = 0.02 m, jarak antar roda = 0.052 m, max wheel = 6.28 rad/s
 *
 * Koordinat Webots (R2025a, ENU): x = timur, y = utara, heading 0 = hadap +x.
 */

#include <math.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/gyro.h>

#define PI 3.14159265358979323846

/* ------------------------------------------------------------------ */
/*  Parameter robot                                                    */
/* ------------------------------------------------------------------ */
#define WHEEL_R   0.02     /* m   */
#define AXLE_L    0.052    /* m   */
#define MAX_WHEEL 6.28     /* rad/s (e-puck v1) */

/* Pose awal = persis dari test_01.wbt */
#define START_X   0.081162
#define START_Y  -0.546865
#define START_TH  0.0

/* ------------------------------------------------------------------ */
/*  Parameter tuning (ubah di sini kalau perlu)                        */
/* ------------------------------------------------------------------ */
#define V_CRUISE      0.095   /* m/s  (~4.75 rad/s di roda)            */
#define V_APPROACH    0.075   /* m/s  kecepatan maju saat menuju balok */
#define W_TURN_MAX    3.0     /* rad/s putar di tempat                 */
#define W_TURN_MIN    0.40
#define TOL_WP        0.03    /* m   toleransi waypoint biasa          */
#define TOL_PARK      0.002   /* m   toleransi posisi akhir            */
#define TOL_HEAD_PARK 0.008   /* rad toleransi heading akhir           */

/* Sensor jarak (nilai mentah; baseline "tidak ada apa-apa" = ~67)     */
#define PS_SAFE       320.0   /* di atas ini robot mulai menjauh dari tembok */
#define PS_STOP_GREEN 200.0   /* rata2 ps0/ps7 -> berhenti di depan balok    */

/* Deteksi hijau */
#define MIN_BLOB_PX   6       /* piksel minimum supaya dianggap target */
#define GREEN_MIN_G   50

/* Kedip LED */
#define BLINK_COUNT   5
#define BLINK_HALF    0.25    /* s nyala, s mati */
#define STOP_PAUSE    0.5     /* s diam sebelum kedip */

/* ------------------------------------------------------------------ */
/*  Waypoint (dari analisis peta test_01.wbt)                          */
/* ------------------------------------------------------------------ */
typedef struct { double x, y; } Pt;

/* P1..P7 : dari Room 1 -> koridor timur -> tengah -> celah -> pintu Room 2 */
static const Pt OUT_WP[] = {
  { 0.480, -0.250 },   /* P1 keluar Room 1 ke koridor timur       */
  { 0.470, -0.040 },   /* P2 naik koridor timur                   */
  { 0.220,  0.050 },   /* P3 lewat atas balok navy                */
  { 0.080,  0.075 },   /* P4                                      */
  {-0.115, -0.180 },   /* P5 masuk celah (antara Room 1 & Room 2) */
  {-0.115, -0.440 },   /* P6 turun celah                          */
  {-0.420, -0.460 }    /* P7 pintu selatan Room 2 (titik SCAN)    */
};
#define N_OUT ((int)(sizeof(OUT_WP) / sizeof(OUT_WP[0])))

/* Jalur pulang: P7 -> ... -> P1 -> masuk Room 1 dari timur */
static const Pt BACK_WP[] = {
  {-0.420, -0.460 },
  {-0.115, -0.440 },
  {-0.115, -0.180 },
  { 0.080,  0.075 },
  { 0.220,  0.050 },
  { 0.470, -0.040 },
  { 0.480, -0.250 },
  { 0.480, -0.380 },   /* B1 */
  { START_X, -0.400 }  /* B2 : di dalam Room 1, sejajar x start */
};
#define N_BACK ((int)(sizeof(BACK_WP) / sizeof(BACK_WP[0])))

/* ------------------------------------------------------------------ */
/*  State                                                              */
/* ------------------------------------------------------------------ */
typedef enum {
  S_OUT, S_SCAN, S_APPROACH, S_STOP, S_BLINK, S_BACK,
  S_PARK_TURN1, S_PARK_DRIVE, S_PARK_TURN2, S_DONE
} State;

static const char *STATE_NAME[] = {
  "OUT", "SCAN", "APPROACH", "STOP", "BLINK", "BACK",
  "PARK_TURN1", "PARK_DRIVE", "PARK_TURN2", "DONE"
};

/* ------------------------------------------------------------------ */
/*  Global                                                             */
/* ------------------------------------------------------------------ */
static int TS;
static WbDeviceTag lm, rm, lps, rps, cam, led5, gyro, ps[8];
static int cam_w, cam_h;

static double px = START_X, py = START_Y, pth = START_TH;   /* pose odometri */
static double prev_l = 0.0, prev_r = 0.0;
static double psv[8];                                         /* nilai ps      */
static int    use_gyro = 1;                                   /* fusi gyro aktif */
static double acc_enc = 0.0, acc_gyr = 0.0;                   /* untuk cek konsistensi */
static double th_enc = START_TH;                              /* heading murni encoder (cadangan) */

/* gyro e-puck: lookupTable [-13.315805 -100000 ... 13.315805 100000] -> raw ke rad/s */
#define GYRO_RAW2RADS (13.315805 / 100000.0)

/* arah sensor ps0..ps7 terhadap hadap robot (rad), untuk gaya tolak */
static const double PS_ANGLE[8] = { -0.30, -0.785, -1.571, -2.62, 2.62, 1.571, 0.785, 0.30 };

static State  state = S_OUT;
static double t_state = 0.0;
static int    wp_idx = 0;
static int    align = 1;           /* 1 = putar di tempat dulu sebelum maju */
static double scan_dir = -1.0;     /* -1 = CW (target diperkirakan di kanan) */
static double scan_accum = 0.0;
static int    lost_cnt = 0;
static double last_e = 0.0;

/* hasil deteksi kamera */
static int    g_cnt = 0, g_x0 = 0, g_x1 = 0;
static double g_cx = 0.0;

/* ------------------------------------------------------------------ */
/*  Utility                                                            */
/* ------------------------------------------------------------------ */
static double wrap(double a) {
  while (a >  PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}
static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* v [m/s], w [rad/s, + = putar kiri/CCW] -> kecepatan roda */
static void set_vw(double v, double w) {
  double l = (v - w * AXLE_L * 0.5) / WHEEL_R;
  double r = (v + w * AXLE_L * 0.5) / WHEEL_R;
  double m = fmax(fabs(l), fabs(r));
  if (m > MAX_WHEEL) {
    l *= MAX_WHEEL / m;
    r *= MAX_WHEEL / m;
  }
  wb_motor_set_velocity(lm, l);
  wb_motor_set_velocity(rm, r);
}

static void change_state(State s) {
  printf("[%6.2fs] %-10s -> %-10s pose=(%.3f, %.3f, %.1f deg)\n",
         wb_robot_get_time(), STATE_NAME[state], STATE_NAME[s],
         px, py, pth * 180.0 / PI);
  state = s;
  t_state = wb_robot_get_time();
  align = 1;
}

/* ------------------------------------------------------------------ */
/*  Odometri (encoder roda, midpoint integration)                      */
/* ------------------------------------------------------------------ */
static void update_odometry(void) {
  double l = wb_position_sensor_get_value(lps);
  double r = wb_position_sensor_get_value(rps);
  double dl = (l - prev_l) * WHEEL_R;
  double dr = (r - prev_r) * WHEEL_R;
  prev_l = l;
  prev_r = r;
  double ds  = 0.5 * (dl + dr);
  double dth = (dr - dl) / AXLE_L;                 /* heading dari encoder */

  th_enc = wrap(th_enc + dth);                     /* selalu dihitung sebagai cadangan */

  if (use_gyro) {
    double wz  = wb_gyro_get_values(gyro)[2] * GYRO_RAW2RADS;   /* yaw rate (rad/s), + = CCW */
    double dthg = wz * TS / 1000.0;
    acc_enc += dth;
    acc_gyr += dthg;
    /* sanity check cepat: setelah ~23 derajat berputar, gyro harus searah & sekitar skala yang sama */
    if (fabs(acc_enc) > 0.4) {
      double ratio = acc_gyr / acc_enc;
      if (ratio < 0.6 || ratio > 1.6) {
        use_gyro = 0;
        pth = th_enc;                              /* buang heading gyro, balik ke encoder */
        printf("WARNING: gyro tidak konsisten dengan encoder (rasio %.2f) -> pakai encoder saja\n", ratio);
      } else {
        acc_enc = acc_gyr = 0.0;
      }
    }
    if (use_gyro) dth = dthg;
  }

  px  += ds * cos(pth + 0.5 * dth);
  py  += ds * sin(pth + 0.5 * dth);
  pth  = wrap(pth + dth);
}

static void read_ps(void) {
  for (int i = 0; i < 8; i++)
    psv[i] = wb_distance_sensor_get_value(ps[i]);
}

/* ------------------------------------------------------------------ */
/*  Kamera: cari piksel hijau                                          */
/* ------------------------------------------------------------------ */
static void detect_green(void) {
  const unsigned char *img = wb_camera_get_image(cam);
  g_cnt = 0;
  g_x0 = cam_w;
  g_x1 = -1;
  double sx = 0.0;
  if (!img) return;
  for (int y = 0; y < cam_h; y++) {
    for (int x = 0; x < cam_w; x++) {
      int r = wb_camera_image_get_red(img, cam_w, x, y);
      int g = wb_camera_image_get_green(img, cam_w, x, y);
      int b = wb_camera_image_get_blue(img, cam_w, x, y);
      if (g > GREEN_MIN_G && g > 1.5 * r && g > 1.5 * b) {
        g_cnt++;
        sx += x;
        if (x < g_x0) g_x0 = x;
        if (x > g_x1) g_x1 = x;
      }
    }
  }
  g_cx = g_cnt ? sx / g_cnt : 0.0;
}

/* ------------------------------------------------------------------ */
/*  Safety layer: pakai ps untuk menjauh dari tembok & melambat        */
/* ------------------------------------------------------------------ */
static void apply_safety(double *v, double *w) {
  /* gaya tolak: tiap sensor di atas PS_SAFE mendorong robot menjauhi arahnya */
  double rx = 0.0, ry = 0.0, front_ex = 0.0;
  for (int i = 0; i < 8; i++) {
    double e = psv[i] - PS_SAFE;
    if (e <= 0.0) continue;
    rx -= e * cos(PS_ANGLE[i]);
    ry -= e * sin(PS_ANGLE[i]);
    if (cos(PS_ANGLE[i]) > 0.5 && e > front_ex) front_ex = e;
  }
  *w += clampd(0.003 * ry, -1.5, 1.5);               /* + = belok kiri (menjauhi sisi kanan) */
  if (front_ex > 0.0)                                 /* melambat, lalu mundur kalau sangat dekat */
    *v = *v * clampd(1.0 - front_ex / 500.0, 0.0, 1.0) + clampd(0.00006 * rx, -0.05, 0.0);
}

/* ------------------------------------------------------------------ */
/*  Go-to-point: putar di tempat -> maju lurus dengan koreksi heading  */
/*  return 1 kalau sudah sampai                                        */
/* ------------------------------------------------------------------ */
static int go_to(double tx, double ty, double vmax, double tol, int safety) {
  double dx = tx - px, dy = ty - py;
  double dist = hypot(dx, dy);
  if (dist < tol) {
    set_vw(0.0, 0.0);
    align = 1;
    return 1;
  }
  double err = wrap(atan2(dy, dx) - pth);

  if (align) {
    if (fabs(err) < 0.04) {
      align = 0;
    } else {
      double w = clampd(3.0 * err, -W_TURN_MAX, W_TURN_MAX);
      if (fabs(w) < W_TURN_MIN) w = (w < 0 ? -W_TURN_MIN : W_TURN_MIN);
      set_vw(0.0, w);
      return 0;
    }
  } else if (fabs(err) > 0.6) {
    align = 1;
    set_vw(0.0, 0.0);
    return 0;
  }

  double v = vmax;
  if (dist < 0.08) v = fmax(0.03, vmax * dist / 0.08);
  double w = clampd(3.0 * err, -1.5, 1.5);
  if (safety) apply_safety(&v, &w);
  set_vw(v, w);
  return 0;
}

/* putar di tempat ke heading absolut */
static int turn_to(double target, double tol) {
  double err = wrap(target - pth);
  if (fabs(err) < tol) {
    set_vw(0.0, 0.0);
    return 1;
  }
  double w = clampd(3.0 * err, -W_TURN_MAX, W_TURN_MAX);
  double wmin = (fabs(err) < 0.15) ? 0.25 : W_TURN_MIN;
  if (fabs(w) < wmin) w = (w < 0 ? -wmin : wmin);
  set_vw(0.0, w);
  return 0;
}

/* ------------------------------------------------------------------ */
/*  main                                                               */
/* ------------------------------------------------------------------ */
int main(void) {
  wb_robot_init();
  TS = (int)wb_robot_get_basic_time_step();

  lm = wb_robot_get_device("left wheel motor");
  rm = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(lm, INFINITY);
  wb_motor_set_position(rm, INFINITY);
  wb_motor_set_velocity(lm, 0.0);
  wb_motor_set_velocity(rm, 0.0);

  lps = wb_robot_get_device("left wheel sensor");
  rps = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(lps, TS);
  wb_position_sensor_enable(rps, TS);

  char name[8];
  for (int i = 0; i < 8; i++) {
    snprintf(name, sizeof(name), "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TS);
  }

  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TS);
  cam_w = wb_camera_get_width(cam);
  cam_h = wb_camera_get_height(cam);

  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, TS);

  led5 = wb_robot_get_device("led5");
  wb_led_set(led5, 0);

  /* satu step supaya sensor punya nilai valid, lalu simpan encoder awal */
  wb_robot_step(TS);
  prev_l = wb_position_sensor_get_value(lps);
  prev_r = wb_position_sensor_get_value(rps);

  printf("Controller mulai. Kamera %dx%d, TS=%d ms\n", cam_w, cam_h, TS);
  t_state = wb_robot_get_time();

  int blink_done = 0;
  double next_log = 0.0;

  while (wb_robot_step(TS) != -1) {
    double t = wb_robot_get_time();
    update_odometry();
    read_ps();
    detect_green();

    if (t >= next_log) {              /* log tiap 5 detik */
      next_log += 5.0;
      printf("[%6.2fs] %-10s pose=(%.3f, %.3f, %.1f deg) ps0=%.0f ps2=%.0f ps5=%.0f ps7=%.0f green_px=%d\n",
             t, STATE_NAME[state], px, py, pth * 180.0 / PI,
             psv[0], psv[2], psv[5], psv[7], g_cnt);
    }

    switch (state) {

    /* ---------------- 1. menuju pintu Room 2 -------------------- */
    case S_OUT: {
      /* di Room 1 dekat tembok selatan, safety dimatikan untuk leg pertama */
      int use_safety = (wp_idx > 0);
      if (go_to(OUT_WP[wp_idx].x, OUT_WP[wp_idx].y, V_CRUISE, TOL_WP, use_safety)) {
        wp_idx++;
        if (wp_idx >= N_OUT) {
          wp_idx = 0;
          scan_accum = 0.0;
          change_state(S_SCAN);
        }
      }
      break;
    }

    /* ---------------- 2. deteksi balok hijau -------------------- */
    case S_SCAN:
      if (g_cnt >= MIN_BLOB_PX) {
        set_vw(0.0, 0.0);
        lost_cnt = 0;
        change_state(S_APPROACH);
      } else {
        set_vw(0.0, scan_dir * 1.0);
        scan_accum += 1.0 * TS / 1000.0;
        if (scan_accum > 2.0 * PI + 0.3) {   /* sudah satu putaran penuh -> balik arah */
          scan_dir = -scan_dir;
          scan_accum = 0.0;
        }
      }
      break;

    /* ---------------- 3. mendekat ------------------------------- */
    case S_APPROACH: {
      double front = 0.5 * (psv[0] + psv[7]);
      if (g_cnt >= 3) {
        lost_cnt = 0;
        double e = (g_cx - (cam_w - 1) * 0.5) / (cam_w * 0.5);   /* -1..1, + = target di kanan */
        last_e = e;
        double w = -1.4 * e;
        double v = V_APPROACH * clampd(1.0 - fabs(e) / 0.5, 0.0, 1.0);
        if (fabs(e) < 0.5 && v < 0.015) v = 0.015;
        if (front > 130.0) v *= 0.6;                              /* sudah dekat -> pelan */
        int blob_w = g_x1 - g_x0 + 1;
        if (front >= PS_STOP_GREEN || blob_w >= cam_w - 4) {
          set_vw(0.0, 0.0);
          change_state(S_STOP);
        } else {
          set_vw(v, w);
        }
      } else {
        /* target hilang dari frame: putar ke arah terakhir terlihat */
        lost_cnt++;
        set_vw(0.0, (last_e > 0 ? -0.6 : 0.6));
        if (lost_cnt > 60) change_state(S_SCAN);
      }
      break;
    }

    /* ---------------- 4. berhenti ------------------------------- */
    case S_STOP:
      set_vw(0.0, 0.0);
      if (t - t_state >= STOP_PAUSE) change_state(S_BLINK);
      break;

    /* ---------------- 5. led5 nyala 5 kali ---------------------- */
    case S_BLINK: {
      set_vw(0.0, 0.0);
      int phase = (int)floor((t - t_state) / BLINK_HALF);   /* 0=on 1=off 2=on ... */
      if (phase >= 2 * BLINK_COUNT) {
        wb_led_set(led5, 0);
        blink_done = 1;
      } else {
        wb_led_set(led5, (phase % 2 == 0) ? 1 : 0);
      }
      if (blink_done) {
        wp_idx = 0;
        change_state(S_BACK);
      }
      break;
    }

    /* ---------------- 6. pulang --------------------------------- */
    case S_BACK:
      /* waypoint terakhir (sejajar x start) harus presisi, sisanya longgar */
      double tol = (wp_idx == N_BACK - 1) ? TOL_PARK : TOL_WP;
      if (go_to(BACK_WP[wp_idx].x, BACK_WP[wp_idx].y, V_CRUISE, tol, 1)) {
        wp_idx++;
        if (wp_idx >= N_BACK) change_state(S_PARK_TURN1);
      }
      break;

    /* ---------------- 7. parkir di pose awal -------------------- */
    case S_PARK_TURN1:                       /* hadap selatan (-y) */
      if (turn_to(-PI / 2.0, 0.01)) change_state(S_PARK_DRIVE);
      break;

    case S_PARK_DRIVE: {                     /* turun ke y start, tanpa safety (dekat tembok) */
      double err = START_Y - py;
      double v = clampd(1.5 * fabs(err), 0.02, 0.05);
      if (fabs(err) < TOL_PARK) {
        set_vw(0.0, 0.0);
        change_state(S_PARK_TURN2);
      } else {
        double w = clampd(3.0 * wrap(-PI / 2.0 - pth), -1.0, 1.0);
        set_vw(err < 0 ? v : -v, err < 0 ? w : -w);
      }
      break;
    }

    case S_PARK_TURN2:                       /* heading akhir = rotasi awal (0) */
      if (turn_to(START_TH, TOL_HEAD_PARK)) {
        change_state(S_DONE);
        printf("SELESAI. Waktu total = %.2f s | pose odometri = (%.4f, %.4f, %.2f deg)\n",
               t, px, py, pth * 180.0 / PI);
        printf("Target pose          = (%.4f, %.4f, %.2f deg)\n",
               START_X, START_Y, START_TH * 180.0 / PI);
      }
      break;

    case S_DONE:
    default:
      set_vw(0.0, 0.0);
      break;
    }
  }

  wb_robot_cleanup();
  return 0;
}