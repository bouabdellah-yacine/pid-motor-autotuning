/*
 * Auto-réglage : identification du moteur par un essai indicielle puis réglage
 * du PI par la méthode SIMC de Skogestad (robuste, utilisée en industrie).
 *  1. on applique un échelon de PWM et on enregistre la vitesse
 *  2. on en déduit un modèle « 1er ordre + retard » : gain K, constante τ, retard θ
 *     (méthode des deux points : 28,3 % et 63,2 % de la réponse)
 *  3. Kp = τ / (K (τc + θ)),  Ti = min(τ, 4 (τc + θ))
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float K, tau, theta; } fopdt_t;

/* y : vitesses mesurées après l'échelon (y[0] = avant l'échelon), du = amplitude de l'échelon */
int  tune_identify(const float *y, int n, float Ts, float du, fopdt_t *model);
/* tauc : rapidité voulue en boucle fermée (s) ; renvoie kp, ki */
void tune_simc(const fopdt_t *m, float tauc, float *kp, float *ki);

#ifdef __cplusplus
}
#endif
