/*
 * Régulateur PID discret, version « industrielle » :
 *  - dérivée sur la MESURE (pas de coup de bélier quand la consigne change)
 *  - filtre passe-bas sur la dérivée (le bruit du codeur n'est pas amplifié)
 *  - anti-emballement de l'intégrale (anti-windup par recalcul) quand le PWM sature
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp, ki, kd;        /* gains (sortie = rapport cyclique 0..1, erreur en tr/min) */
  float Ts;                /* période d'échantillonnage (s) */
  float tf;                /* constante du filtre de dérivée (s) */
  float umin, umax;        /* limites de la commande */
  int   antiwindup;        /* 1 = actif */
  /* état interne */
  float integ, deriv, prev_meas;
  int   first;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float Ts);
void  pid_reset(pid_ctrl_t *p);
float pid_update(pid_ctrl_t *p, float setpoint, float measure);

#ifdef __cplusplus
}
#endif
