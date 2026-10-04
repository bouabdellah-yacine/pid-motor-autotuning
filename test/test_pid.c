/*
 * Tests du régulateur sur PC (même code que sur l'ESP32) :
 *   gcc -O2 -Wall -Wextra -o t test/test_pid.c src/pid.c src/motor_model.c src/tuning.c -lm && ./t
 */
#include <stdio.h>
#include <math.h>
#include "sim.h"
#include "../src/tuning.h"

static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("%s ", ok_ ? "[OK]  " : "[ECHEC]"); \
  printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

static const float Ts = 0.01f;

/* Charge qui bloque le moteur pendant 2 s puis disparaît : test d'emballement de l'intégrale */
static float stall_overshoot(int antiwindup) {
  pid_ctrl_t p; pid_init(&p, 0.0002f, 0.006f, 0, Ts); p.antiwindup = antiwindup;
  motor_t m; motor_init(&m); int32_t last = 0; float y = 0, peak = 0;
  for (int k = 0; k < 600; k++) {
    float t = k * Ts;
    m.load = (t >= 1.0f && t < 3.0f) ? 0.45f : 0.0f;    /* plus que le couple max → blocage */
    float u = pid_update(&p, 1500, y);
    for (int s = 0; s < 10; s++) motor_step(&m, u, Ts / 10);
    y = measure_rpm(&last, &m, Ts);
    if (t >= 3.0f && y > peak) peak = y;
  }
  return (peak - 1500) / 1500 * 100;
}

int main(void) {
  /* 1. Identification du moteur par un échelon de PWM 30 % → 60 % */
  motor_t m; motor_init(&m); int32_t last = 0; float y = 0, buf[300];
  for (int k = 0; k < 200; k++) { for (int s = 0; s < 10; s++) motor_step(&m, 0.3f, Ts / 10); y = measure_rpm(&last, &m, Ts); }
  buf[0] = y;
  for (int k = 1; k < 300; k++) { for (int s = 0; s < 10; s++) motor_step(&m, 0.6f, Ts / 10); buf[k] = y = measure_rpm(&last, &m, Ts); }
  fopdt_t f;
  int ok = tune_identify(buf, 300, Ts, 0.3f, &f);
  float K_th = 12.0f / 0.035f * 60.0f / (2 * (float)M_PI);       /* gain statique théorique */
  CHECK(ok && fabsf(f.K - K_th) / K_th < 0.05f, "identification : K = %.0f tr/min par unité de PWM (théorie %.0f)", f.K, K_th);
  CHECK(f.tau > 0.25f && f.tau < 0.40f, "identification : constante de temps τ = %.3f s (théorie ≈ 0,33 s)", f.tau);

  /* 2. Réglage manuel médiocre (valeurs par défaut de la démo) */
  pid_ctrl_t p; pid_init(&p, 0.0002f, 0.006f, 0, Ts);
  step_metrics_t r0 = sim_step(&p, 500, 2000, 5, -1, 0, 0, 0);
  CHECK(r0.overshoot > 15, "réglage manuel : dépassement %.1f %%, oscillations lentes (mauvais, c'est voulu)", r0.overshoot);

  /* 3. PI auto-réglé (SIMC) */
  float kp, ki; tune_simc(&f, 0.1f, &kp, &ki);
  pid_init(&p, kp, ki, 0, Ts);
  step_metrics_t r1 = sim_step(&p, 500, 2000, 3, -1, 0, 0, 0);
  CHECK(r1.overshoot < 5, "auto-réglage : dépassement %.1f %% (< 5 %%)", r1.overshoot);
  CHECK(r1.t_settle > 0 && r1.t_settle < 0.6f, "auto-réglage : stabilisation à ±5 %% en %.2f s (< 0,6 s)", r1.t_settle);
  CHECK(fabsf(r1.err_final) < 0.5f, "auto-réglage : erreur statique %.2f %% (< 0,5 %%)", r1.err_final);

  /* 4. Rejet de perturbation : on freine l'arbre */
  step_metrics_t r2 = sim_step(&p, 1500, 1500, 3, 0.5f, 0.08f, 0, 0);
  CHECK(fabsf(r2.err_final) < 0.5f, "charge de 0,08 N·m : erreur finale %.2f %% (l'intégrale compense)", r2.err_final);

  /* 5. Anti-windup : moteur bloqué 2 s puis libéré */
  float os_aw = stall_overshoot(1), os_no = stall_overshoot(0);
  CHECK(os_aw < os_no * 0.5f, "anti-windup : dépassement après blocage %.0f %% (sans anti-windup : %.0f %%)", os_aw, os_no);

  /* 6. La commande reste dans les limites de l'actionneur */
  pid_init(&p, 1.0f, 1.0f, 0.1f, Ts);
  int in = 1;
  for (int k = 0; k < 100; k++) { float u = pid_update(&p, (k & 8) ? 3000 : 0, 1000); if (u < 0 || u > 1) in = 0; }
  CHECK(in, "commande toujours entre 0 et 100 %% de PWM");

  printf("\n%s : %d échec(s)\n", fails ? "ÉCHEC" : "TOUS LES TESTS PASSENT", fails);
  return fails != 0;
}
