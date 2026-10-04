/*
 * Modèle physique d'un moteur à courant continu (jumeau numérique).
 * Sert de « matériel simulé » : le régulateur ne voit que le PWM qu'il envoie
 * et les impulsions du codeur qu'il reçoit, exactement comme sur une vraie carte.
 *
 *   Électrique : L di/dt = V - R i - Ke w
 *   Mécanique  : J dw/dt = Kt i - b w - frottement - charge
 *   Codeur     : 1024 impulsions/tour, décodage x4 = 4096 points/tour
 */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CPR 4096          /* points de codeur par tour */

typedef struct {
  float R, L, Ke, Kt, J, b, friction, vbus;   /* paramètres */
  float i, w, theta;                           /* état : courant, vitesse (rad/s), angle (rad) */
  float load;                                  /* couple de charge extérieur (N·m) */
} motor_t;

void    motor_init(motor_t *m);
void    motor_step(motor_t *m, float duty, float dt);   /* duty 0..1 */
int32_t motor_encoder(const motor_t *m);                 /* compteur du codeur */
float   motor_rpm(const motor_t *m);                     /* vitesse vraie (pour les tests) */

#ifdef __cplusplus
}
#endif
