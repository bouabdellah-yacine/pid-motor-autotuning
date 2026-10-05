/*
 * Physical model of a DC motor (digital twin).
 * Acts as "simulated hardware": the controller only sees the PWM it outputs
 * and the encoder pulses it receives, exactly as on real hardware.
 *
 *   Electrical : L di/dt = V - R i - Ke w
 *   Mechanical : J dw/dt = Kt i - b w - friction - load
 *   Encoder    : 1024 pulses/rev, x4 decoding = 4096 counts/rev
 */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CPR 4096          /* encoder counts per revolution */

typedef struct {
  float R, L, Ke, Kt, J, b, friction, vbus;   /* parameters */
  float i, w, theta;                           /* state: current, speed (rad/s), angle (rad) */
  float load;                                  /* external load torque (N·m) */
} motor_t;

void    motor_init(motor_t *m);
void    motor_step(motor_t *m, float duty, float dt);   /* duty 0..1 */
int32_t motor_encoder(const motor_t *m);                 /* encoder count */
float   motor_rpm(const motor_t *m);                     /* true speed (for tests) */

#ifdef __cplusplus
}
#endif
