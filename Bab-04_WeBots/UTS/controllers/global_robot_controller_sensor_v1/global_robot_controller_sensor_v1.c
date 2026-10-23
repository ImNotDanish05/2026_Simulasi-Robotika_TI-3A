/*
 * global_robot_controller_sensor.c   (versi 2: outbound FULL SENSOR)
 *
 * E-puck maze challenge (Webots R2025a, world: test_01.wbt)
 *
 *  OUTBOUND (dilarang pakai koordinat):
 *    1. WALL   : wall following (default: tembok di KIRI) pakai 8 sensor ps0..ps7
 *                sambil kamera terus mencari balok HIJAU
 *    2. APPROACH: visual servoing (kamera) + ps0/ps7 untuk berhenti
 *    3. STOP, BLINK: berhenti, led5 nyala 5 kali
 *  PULANG (boleh pakai koordinat rumah):
 *    4. BACK   : waypoint odometri (jalur tidak harus sama dengan berangkat)
 *    5. PARK   : masuk Room 1, posisi & heading sama dengan start
 *
 * Odometri (encoder + gyro) tetap dihitung di background sejak awal supaya posisi
 * rumah diketahui, tapi TIDAK dipakai untuk keputusan apa pun selama outbound.
 *
 * Parameter robot dari E-puck.proto: radius roda 0.02 m, jarak roda 0.052 m,
 * max wheel 6.28 rad/s. Koordinat Webots (ENU): heading 0 = hadap +x.
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
#define V_CRUISE      0.11    /* m/s  kecepatan jelajah pulang (~5.5 rad/s di roda) */
#define V_APPROACH    0.075   /* m/s  kecepatan maju saat menuju balok */
#define W_TURN_MAX    3.0     /* rad/s putar di tempat                 */
#define W_TURN_MIN    0.40
#define TOL_WP_MID    0.05    /* m   toleransi waypoint antara (pulang) */
#define TOL_PARK      0.002   /* m   toleransi posisi akhir            */
#define TOL_HEAD_PARK 0.008   /* rad toleransi heading akhir           */

/* Sensor jarak (nilai mentah; baseline "tidak ada apa-apa" = ~67)     */
#define PS_SAFE       320.0   /* di atas ini gaya tolak aktif (mode follow)  */
#define PS_STOP_GREEN 200.0   /* rata2 ps0/ps7 -> berhenti di depan balok    */

/* Wall following */
#define WALL_HAND     1       /* +1 = tembok di KIRI (left-hand), -1 = di KANAN */
#define V_SEEK        0.10    /* m/s cari tembok pertama (lurus)       */
#define V_WF          0.09    /* m/s menyusuri tembok                  */
#define L_REF         0.072   /* m  jarak pusat robot ke tembok (celah tepi ~3 cm) */
#define KP_WALL       45.0    /* rad/s per meter error lateral         */
#define KH_WALL       12.0    /* peredam heading (selisih ps depan vs samping) */
#define SEEN_RAW      95.0    /* ps > ini = ada tembok (baseline ~67)  */
#define FRONT_T       160.0   /* rata ps0/ps7 > ini = tembok di depan  */
#define FRONT_CLEAR   120.0   /* di bawah ini = depan sudah lapang     */
#define W_TURN_WF     2.2     /* rad/s putar di tempat di sudut dalam  */
#define V_ARC         0.055   /* m/s saat membelok mengitari ujung tembok */
#define W_ARC         1.0     /* rad/s                                 */
#define ARC_PRE_STEPS 12       /* langkah lurus dulu sebelum membelok (lewati ujung) */
#define ARC_TIMEOUT   6.0     /* s, kalau tembok tidak ketemu lagi -> SEEK */

/* Deteksi hijau */
#define MIN_BLOB_PX   6       /* piksel minimum supaya dianggap target */
#define GREEN_MIN_G   50
#define GREEN_ROW_MIN 30      /* abaikan baris atas (langit/background), balok ada di bawah horizon */

/* Kedip LED */
#define BLINK_COUNT   5
#define BLINK_HALF    0.25    /* s nyala, s mati */
#define STOP_PAUSE    0.5     /* s diam sebelum kedip */

/* ------------------------------------------------------------------ */
/*  Waypoint PULANG (dari analisis peta test_01.wbt)                   */
/*  Titik pertama = pintu selatan Room 2, lalu celah -> tengah ->      */
/*  koridor timur -> masuk Room 1 dari timur.                          */
/* ------------------------------------------------------------------ */
typedef struct { double x, y; } Pt;

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
  S_WALL, S_APPROACH, S_STOP, S_BLINK, S_BACK,
  S_PARK_TURN1, S_PARK_DRIVE, S_PARK_TURN2, S_DONE
} State;

static const char *STATE_NAME[] = {
  "WALL", "APPROACH", "STOP", "BLINK", "BACK",
  "PARK_TURN1", "PARK_DRIVE", "PARK_TURN2", "DONE"
};

typedef enum { WF_SEEK, WF_FOLLOW, WF_TURN, WF_ARC } WfMode;
static const char *WF_NAME[] = { "SEEK", "FOLLOW", "TURN", "ARC" };

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

static State  state = S_WALL;
static double t_state = 0.0;
static int    wp_idx = 0;
static int    align = 1;           /* 1 = putar di tempat dulu sebelum maju */
static WfMode wf_mode = WF_SEEK;
static int    wf_steps = 0;        /* penghitung langkah di mode WF saat ini */
static double wf_t0 = 0.0;
static int    green_frames = 0;    /* frame beruntun yang melihat hijau      */
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
  for (int y = GREEN_ROW_MIN; y < cam_h; y++) {
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
static int go_to(double tx, double ty, double vmax, double tol, int safety, int final_wp) {
  double dx = tx - px, dy = ty - py;
  double dist = hypot(dx, dy);
  if (dist < tol) {
    if (final_wp) set_vw(0.0, 0.0);
    align = 1;
    return 1;
  }
  double err = wrap(atan2(dy, dx) - pth);
  double align_enter = final_wp ? 0.04 : 0.5;     /* waypoint antara: cuma putar di tempat kalau belokannya tajam */

  if (align) {
    if (fabs(err) < (final_wp ? 0.04 : 0.15)) {
      align = 0;
    } else {
      double w = clampd(3.0 * err, -W_TURN_MAX, W_TURN_MAX);
      if (fabs(w) < W_TURN_MIN) w = (w < 0 ? -W_TURN_MIN : W_TURN_MIN);
      set_vw(0.0, w);
      return 0;
    }
  } else if (fabs(err) > align_enter + 0.2) {
    align = 1;
    set_vw(0.0, 0.0);
    return 0;
  }

  double v = vmax;
  if (final_wp && dist < 0.08) v = fmax(0.03, vmax * dist / 0.08);   /* melambat hanya di titik akhir */
  double w = clampd(3.0 * err, -1.5, 1.5);
  if (fabs(err) > 0.3) v *= 0.6;                                      /* belokan agak tajam -> pelan */
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
/*  Konversi pembacaan ps -> jarak (m), invers lookupTable E-puckDistanceSensor */
/* ------------------------------------------------------------------ */
static const double LUT_V[10] = { 4095.0, 2133.33, 1465.73, 601.46, 383.84, 234.93, 158.03, 120.0, 104.09, 67.19 };
static const double LUT_D[10] = { 0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07 };

static double ps_dist(double raw) {
  if (raw <= 72.0) return 0.08;                     /* baseline -> tidak ada tembok dalam 7 cm */
  if (raw >= LUT_V[0]) return 0.0;
  for (int i = 0; i < 9; i++) {
    if (raw <= LUT_V[i] && raw >= LUT_V[i + 1]) {
      double f = (LUT_V[i] - raw) / (LUT_V[i] - LUT_V[i + 1]);
      return LUT_D[i] + f * (LUT_D[i + 1] - LUT_D[i]);
    }
  }
  return 0.08;
}

/* ------------------------------------------------------------------ */
/*  WALL FOLLOWING (hanya sensor ps, tanpa koordinat)                  */
/*  WALL_HAND=+1: tembok di kiri -> sensor samping ps5, depan-samping ps6, belakang-samping ps4 */
/*  WALL_HAND=-1: cermin -> ps2, ps1, ps3                              */
/* ------------------------------------------------------------------ */
#define IDX_SIDE   (WALL_HAND > 0 ? 5 : 2)
#define IDX_SFRONT (WALL_HAND > 0 ? 6 : 1)
#define IDX_SREAR  (WALL_HAND > 0 ? 4 : 3)

static void wf_set(WfMode m) {
  wf_mode = m;
  wf_steps = 0;
  wf_t0 = wb_robot_get_time();
}

static void wall_follow(void) {
  double front = 0.5 * (psv[0] + psv[7]);
  double fmx   = fmax(psv[0], psv[7]);
  double sd = psv[IDX_SIDE], sf = psv[IDX_SFRONT], sr = psv[IDX_SREAR];
  int seen_side = (sd > SEEN_RAW || sf > SEEN_RAW);
  int seen_any  = (seen_side || sr > SEEN_RAW);
  int blocked   = (front > FRONT_T || fmx > FRONT_T * 1.4);
  double t = wb_robot_get_time();
  wf_steps++;

  switch (wf_mode) {

  /* belum ada tembok di sisi yang diikuti: maju lurus sampai ketemu */
  case WF_SEEK:
    if (blocked)        { wf_set(WF_TURN); set_vw(0.0, 0.0); }
    else if (seen_side) { wf_set(WF_FOLLOW); }
    else                { set_vw(V_SEEK, 0.0); }
    break;

  /* sudut dalam: putar di tempat menjauhi tembok yang dihadapi */
  case WF_TURN: {
    set_vw(0.0, -WALL_HAND * W_TURN_WF);
    if (front < FRONT_CLEAR && fmx < FRONT_CLEAR * 1.4 && sd > SEEN_RAW && sd >= 0.8 * sf) {
      wf_set(WF_FOLLOW);
    } else if (t - wf_t0 > 4.0) {                    /* 4 s ~ lebih dari 360 derajat */
      wf_set(WF_SEEK);
    }
    break;
  }

  /* tembok hilang (ujung tembok / bukaan): maju sedikit lalu membelok mengitari ujungnya */
  case WF_ARC:
    if (blocked) { wf_set(WF_TURN); set_vw(0.0, 0.0); break; }
    if (wf_steps <= ARC_PRE_STEPS) {
      set_vw(V_ARC, 0.0);
    } else {
      set_vw(V_ARC, WALL_HAND * W_ARC);
      if (seen_side && wf_steps > ARC_PRE_STEPS + 8) wf_set(WF_FOLLOW);
      else if (t - wf_t0 > ARC_TIMEOUT)               wf_set(WF_SEEK);
    }
    break;

  /* menyusuri tembok: jaga jarak L_REF dari pusat robot */
  case WF_FOLLOW:
  default: {
    if (blocked) { wf_set(WF_TURN); set_vw(0.0, 0.0); break; }
    if (!seen_any) { wf_set(WF_ARC); set_vw(V_ARC, 0.0); break; }

    /* estimasi jarak lateral pusat-robot -> tembok dari tiap sensor yang melihat tembok */
    double L5 = (sd > SEEN_RAW) ? 0.031 + ps_dist(sd) : 9.0;
    double L6 = (sf > SEEN_RAW) ? 0.025 + 0.7071 * ps_dist(sf) : 9.0;
    double Lest = fmin(L5, L6);
    if (Lest > 8.0) Lest = L_REF + 0.02;             /* hanya sensor belakang yang lihat: lanjut pelan ke arah tembok */

    double e = Lest - L_REF;                          /* + = terlalu jauh */
    double damp = (L5 < 8.0 && L6 < 8.0) ? (L5 - L6) : 0.0;  /* + = menjauhi tembok? (L6 lebih dekat -> mendekat) */
    double w = WALL_HAND * (KP_WALL * e - KH_WALL * damp);
    w = clampd(w, -2.5, 2.5);
    double v = V_WF * clampd(1.0 - fabs(w) / 3.0, 0.45, 1.0);
    if (front > 130.0) v *= 0.7;
    apply_safety(&v, &w);
    set_vw(v, w);
    break;
  }
  }
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

    /* ---------------- 1+2. cari balok hijau: FULL SENSOR ----------- */
    case S_WALL:
      if (g_cnt >= MIN_BLOB_PX) green_frames++; else green_frames = 0;
      if (green_frames >= 2) {                       /* terlihat 2 frame beruntun -> target */
        set_vw(0.0, 0.0);
        lost_cnt = 0;
        change_state(S_APPROACH);
      } else {
        WfMode before = wf_mode;
        wall_follow();
        if (wf_mode != before)
          printf("[%6.2fs]   wall-follow: %-6s -> %-6s (ps: front=%.0f side=%.0f sf=%.0f)\n",
                 t, WF_NAME[before], WF_NAME[wf_mode], 0.5 * (psv[0] + psv[7]),
                 psv[WALL_HAND > 0 ? 5 : 2], psv[WALL_HAND > 0 ? 6 : 1]);
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
        if (lost_cnt > 60) { wf_set(WF_SEEK); green_frames = 0; change_state(S_WALL); }
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
      int last = (wp_idx == N_BACK - 1);
      double tol = last ? TOL_PARK : TOL_WP_MID;
      if (go_to(BACK_WP[wp_idx].x, BACK_WP[wp_idx].y, V_CRUISE, tol, 1, last)) {
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