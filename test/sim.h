/* Closed loop simulated on a PC: same PID code as on the ESP32, virtual motor. */
#pragma once
#include "../src/motor_model.h"
#include "../src/pid.h"

typedef struct { float overshoot, t_settle, err_final, peak; } step_metrics_t;

/* Speed measurement as on the board: encoder count difference over Ts */
static inline float measure_rpm(int32_t *last, const motor_t *m, float Ts) {
  int32_t c = motor_encoder(m);
  float rpm = (float)(c - *last) / MOTOR_CPR / Ts * 60.0f;
  *last = c;
  return rpm;
}

/* Simulates a setpoint step sp0 → sp1; optional: load applied at t_load */
static inline step_metrics_t sim_step(pid_ctrl_t *p, float sp0, float sp1, float t_total,
                                      float t_load, float load, float *trace, int trace_n) {
  motor_t m; motor_init(&m);
  int32_t last = 0; float y = 0;
  pid_reset(p);
  /* warm-up at sp0 */
  for (int k = 0; k < 300; k++) {
    float u = pid_update(p, sp0, y);
    for (int s = 0; s < 10; s++) motor_step(&m, u, p->Ts / 10);
    y = measure_rpm(&last, &m, p->Ts);
  }
  step_metrics_t r = {0, -1, 0, 0};
  int n = (int)(t_total / p->Ts);
  float band = 0.05f * (sp1 - sp0 > 0 ? sp1 - sp0 : sp0 - sp1);
  if (band < 0.02f * sp1) band = 0.02f * sp1;
  float last_out = 0;
  for (int k = 0; k < n; k++) {
    float t = k * p->Ts;
    m.load = (t_load >= 0 && t >= t_load) ? load : 0;
    float u = pid_update(p, sp1, y);
    for (int s = 0; s < 10; s++) motor_step(&m, u, p->Ts / 10);
    y = measure_rpm(&last, &m, p->Ts);
    if (trace && k < trace_n) trace[k] = y;
    if (t_load < 0 || t < t_load) {
      if (y > r.peak) r.peak = y;
      float dev = y - sp1; if (dev < 0) dev = -dev;
      if (dev > band) { r.t_settle = -1; last_out = t; }
      else if (r.t_settle < 0) r.t_settle = t;
    }
  }
  (void)last_out;
  r.overshoot = sp1 > sp0 ? (r.peak - sp1) / (sp1 - sp0) * 100.0f : 0;
  if (r.overshoot < 0) r.overshoot = 0;
  /* final error: average over the last 0.5 s */
  float acc = 0; int cnt = 0;
  for (int k = 0; k < 50; k++) {
    float u = pid_update(p, sp1, y);
    for (int s = 0; s < 10; s++) motor_step(&m, u, p->Ts / 10);
    y = measure_rpm(&last, &m, p->Ts); acc += y; cnt++;
  }
  r.err_final = (acc / cnt - sp1) / sp1 * 100.0f;
  return r;
}
