#include "tuning.h"

int tune_identify(const float *y, int n, float Ts, float du, fopdt_t *m) {
  if (n < 10 || du == 0) return 0;
  float y0 = y[0], yend = 0;
  int tail = n / 10;                                    /* moyenne des 10 % finaux */
  for (int k = n - tail; k < n; k++) yend += y[k];
  yend /= tail;
  float dy = yend - y0;
  if (dy <= 0) return 0;
  float t28 = -1, t63 = -1;
  for (int k = 1; k < n; k++) {
    float r = (y[k] - y0) / dy;
    float rp = (y[k - 1] - y0) / dy;
    float t = (k - 1 + (0.283f - rp) / (r - rp + 1e-9f)) * Ts;   /* interpolation */
    if (t28 < 0 && r >= 0.283f) t28 = t;
    if (t63 < 0 && r >= 0.632f) { t63 = (k - 1 + (0.632f - rp) / (r - rp + 1e-9f)) * Ts; break; }
  }
  if (t28 < 0 || t63 < 0) return 0;
  m->K = dy / du;
  m->tau = 1.5f * (t63 - t28);
  m->theta = t63 - m->tau;
  if (m->theta < Ts) m->theta = Ts;
  return 1;
}

void tune_simc(const fopdt_t *m, float tauc, float *kp, float *ki) {
  float kc = m->tau / (m->K * (tauc + m->theta));
  float ti = 4.0f * (tauc + m->theta);
  if (m->tau < ti) ti = m->tau;
  *kp = kc;
  *ki = kc / ti;
}
