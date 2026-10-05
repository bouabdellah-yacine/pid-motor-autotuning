/*
 * Controller tests on a PC (same code as on the ESP32):
 *   gcc -O2 -Wall -Wextra -o t test/test_pid.c src/pid.c src/motor_model.c src/tuning.c -lm && ./t
 */
#include <stdio.h>
#include <math.h>
#include "sim.h"
#include "../src/tuning.h"

static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("%s ", ok_ ? "[OK]  " : "[FAIL]"); \
  printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

static const float Ts = 0.01f;

/* Load that stalls the motor for 2 s and then disappears: integrator windup test */
static float stall_overshoot(int antiwindup) {
  pid_ctrl_t p; pid_init(&p, 0.0002f, 0.006f, 0, Ts); p.antiwindup = antiwindup;
  motor_t m; motor_init(&m); int32_t last = 0; float y = 0, peak = 0;
  for (int k = 0; k < 600; k++) {
    float t = k * Ts;
    m.load = (t >= 1.0f && t < 3.0f) ? 0.45f : 0.0f;    /* more than max torque → stall */
    float u = pid_update(&p, 1500, y);
    for (int s = 0; s < 10; s++) motor_step(&m, u, Ts / 10);
    y = measure_rpm(&last, &m, Ts);
    if (t >= 3.0f && y > peak) peak = y;
  }
  return (peak - 1500) / 1500 * 100;
}

int main(void) {
  /* 1. Motor identification with a 30% → 60% PWM step */
  motor_t m; motor_init(&m); int32_t last = 0; float y = 0, buf[300];
  for (int k = 0; k < 200; k++) { for (int s = 0; s < 10; s++) motor_step(&m, 0.3f, Ts / 10); y = measure_rpm(&last, &m, Ts); }
  buf[0] = y;
  for (int k = 1; k < 300; k++) { for (int s = 0; s < 10; s++) motor_step(&m, 0.6f, Ts / 10); buf[k] = y = measure_rpm(&last, &m, Ts); }
  fopdt_t f;
  int ok = tune_identify(buf, 300, Ts, 0.3f, &f);
  float K_th = 12.0f / 0.035f * 60.0f / (2 * (float)M_PI);       /* theoretical static gain */
  CHECK(ok && fabsf(f.K - K_th) / K_th < 0.05f, "identification: K = %.0f rpm per unit PWM (theory %.0f)", f.K, K_th);
  CHECK(f.tau > 0.25f && f.tau < 0.40f, "identification: time constant τ = %.3f s (theory ≈ 0.33 s)", f.tau);

  /* 2. Deliberately poor manual tuning (demo default values) */
  pid_ctrl_t p; pid_init(&p, 0.0002f, 0.006f, 0, Ts);
  step_metrics_t r0 = sim_step(&p, 500, 2000, 5, -1, 0, 0, 0);
  CHECK(r0.overshoot > 15, "manual tuning: overshoot %.1f%%, slow oscillations (poor, by design)", r0.overshoot);

  /* 3. Auto-tuned PI (SIMC) */
  float kp, ki; tune_simc(&f, 0.1f, &kp, &ki);
  pid_init(&p, kp, ki, 0, Ts);
  step_metrics_t r1 = sim_step(&p, 500, 2000, 3, -1, 0, 0, 0);
  CHECK(r1.overshoot < 5, "auto-tuning: overshoot %.1f%% (< 5%%)", r1.overshoot);
  CHECK(r1.t_settle > 0 && r1.t_settle < 0.6f, "auto-tuning: settling to ±5%% in %.2f s (< 0.6 s)", r1.t_settle);
  CHECK(fabsf(r1.err_final) < 0.5f, "auto-tuning: steady-state error %.2f%% (< 0.5%%)", r1.err_final);

  /* 4. Disturbance rejection: the shaft is braked */
  step_metrics_t r2 = sim_step(&p, 1500, 1500, 3, 0.5f, 0.08f, 0, 0);
  CHECK(fabsf(r2.err_final) < 0.5f, "0.08 N·m load: final error %.2f%% (the integrator compensates)", r2.err_final);

  /* 5. Anti-windup: motor stalled for 2 s, then released */
  float os_aw = stall_overshoot(1), os_no = stall_overshoot(0);
  CHECK(os_aw < os_no * 0.5f, "anti-windup: overshoot after stall %.0f%% (without anti-windup: %.0f%%)", os_aw, os_no);

  /* 6. The output stays within actuator limits */
  pid_init(&p, 1.0f, 1.0f, 0.1f, Ts);
  int in = 1;
  for (int k = 0; k < 100; k++) { float u = pid_update(&p, (k & 8) ? 3000 : 0, 1000); if (u < 0 || u > 1) in = 0; }
  CHECK(in, "output always between 0 and 100%% PWM");

  printf("\n%s: %d failure(s)\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
  return fails != 0;
}
