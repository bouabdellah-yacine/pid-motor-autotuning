/*
 * Auto-tuning: motor identification from a step test, then PI tuning
 * with Skogestad's SIMC method (robust, widely used in industry).
 *  1. apply a PWM step and record the speed
 *  2. derive a first-order-plus-dead-time model: gain K, time constant τ, dead time θ
 *     (two-point method: 28.3% and 63.2% of the response)
 *  3. Kp = τ / (K (τc + θ)),  Ti = min(τ, 4 (τc + θ))
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float K, tau, theta; } fopdt_t;

/* y: speeds measured after the step (y[0] = before the step), du = step amplitude */
int  tune_identify(const float *y, int n, float Ts, float du, fopdt_t *model);
/* tauc: desired closed-loop time constant (s); returns kp, ki */
void tune_simc(const fopdt_t *m, float tauc, float *kp, float *ki);

#ifdef __cplusplus
}
#endif
