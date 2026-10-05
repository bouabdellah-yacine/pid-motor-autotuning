/*
 * Discrete PID controller, industrial-grade version:
 *  - derivative on MEASUREMENT (no derivative kick when the setpoint changes)
 *  - low-pass filter on the derivative (encoder noise is not amplified)
 *  - integrator windup protection (back-calculation anti-windup) when the PWM saturates
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp, ki, kd;        /* gains (output = duty cycle 0..1, error in rpm) */
  float Ts;                /* sampling period (s) */
  float tf;                /* derivative filter time constant (s) */
  float umin, umax;        /* output limits */
  int   antiwindup;        /* 1 = enabled */
  /* internal state */
  float integ, deriv, prev_meas;
  int   first;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float Ts);
void  pid_reset(pid_ctrl_t *p);
float pid_update(pid_ctrl_t *p, float setpoint, float measure);

#ifdef __cplusplus
}
#endif
