#include "motor_model.h"
#include <math.h>

void motor_init(motor_t *m) {
  m->R = 1.0f;          /* Ω   */
  m->L = 0.02f;         /* H   (motor + driver) */
  m->Ke = 0.035f;       /* V/(rad/s) */
  m->Kt = 0.035f;       /* N·m/A */
  m->J = 0.0004f;       /* kg·m² (rotor + flywheel) */
  m->b = 1e-5f;         /* viscous friction */
  m->friction = 0.002f; /* Coulomb friction (N·m) */
  m->vbus = 12.0f;      /* supply voltage (V) */
  m->i = m->w = m->theta = 0.0f;
  m->load = 0.0f;
}

void motor_step(motor_t *m, float duty, float dt) {
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  float v = duty * m->vbus;
  m->i += (v - m->R * m->i - m->Ke * m->w) / m->L * dt;
  if (m->i < 0) m->i = 0;                       /* unidirectional H-bridge */
  float torque = m->Kt * m->i - m->b * m->w;
  float resist = m->load + m->friction;
  if (m->w > 0.0f || torque > resist) torque -= resist;
  else torque = 0.0f;                           /* motor held still by friction */
  m->w += torque / m->J * dt;
  if (m->w < 0) m->w = 0;
  m->theta += m->w * dt;
}

int32_t motor_encoder(const motor_t *m) {
  return (int32_t)(m->theta * MOTOR_CPR / (2.0f * (float)M_PI));
}

float motor_rpm(const motor_t *m) { return m->w * 60.0f / (2.0f * (float)M_PI); }
