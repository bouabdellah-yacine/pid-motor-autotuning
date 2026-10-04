#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float Ts) {
  p->kp = kp; p->ki = ki; p->kd = kd; p->Ts = Ts;
  p->tf = 0.02f; p->umin = 0.0f; p->umax = 1.0f; p->antiwindup = 1;
  pid_reset(p);
}

void pid_reset(pid_ctrl_t *p) { p->integ = 0; p->deriv = 0; p->prev_meas = 0; p->first = 1; }

float pid_update(pid_ctrl_t *p, float sp, float y) {
  float e = sp - y;
  if (p->first) { p->prev_meas = y; p->first = 0; }

  /* dérivée sur la mesure, filtrée au 1er ordre */
  float d_raw = -p->kd * (y - p->prev_meas) / p->Ts;
  float a = p->Ts / (p->tf + p->Ts);
  p->deriv += a * (d_raw - p->deriv);
  p->prev_meas = y;

  float v = p->kp * e + p->integ + p->deriv;              /* commande non saturée */
  float u = v < p->umin ? p->umin : (v > p->umax ? p->umax : v);

  /* intégrale + anti-windup par recalcul : on « dégonfle » l'intégrale de
     l'excès de commande que l'actionneur ne peut pas fournir */
  p->integ += p->ki * p->Ts * e;
  if (p->antiwindup && p->ki > 0 && p->kp > 0) {
    float tt = 0.5f * p->kp / p->ki;                    /* Tt = Ti / 2 */
    float g = p->Ts / tt; if (g > 1.0f) g = 1.0f;
    p->integ += (u - v) * g;
  }
  return u;
}
