# 🎛️ Régulateur PID de moteur avec auto-réglage (ESP32 + FreeRTOS)

Un ESP32 tient la vitesse d'un moteur à courant continu exactement à la consigne, même quand on freine
l'arbre. Un bouton lance l'**auto-réglage** : la carte identifie le moteur toute seule, puis calcule les
gains du régulateur par la méthode **SIMC** (Skogestad), utilisée dans l'industrie.

> ✅ Simulé sur **Wokwi** (VS Code). Le moteur est un **jumeau numérique** (modèle physique électrique +
> mécanique + codeur 4096 points/tour) exécuté à 1 kHz : le régulateur ne voit que le PWM et le codeur,
> comme sur une vraie carte.

## Résultats (mesurés par les tests automatiques)

| | Réglage manuel | **Après auto-réglage** |
|---|---|---|
| Dépassement (échelon 500 → 2000 tr/min) | 25 % | **0,1 %** |
| Temps de stabilisation (±5 %) | > 3 s, oscillations | **0,38 s** |
| Erreur statique | ≈ 0,5 % | **0,00 %** |
| Freinage de 0,08 N·m | — | **erreur finale 0,00 %** |

Anti-windup : après un blocage du moteur de 2 s, le dépassement tombe de **115 % à 35 %**.

## Ce que montre le projet

- **PID discret « industriel »** : dérivée sur la mesure (pas d'à-coup quand la consigne change),
  filtre de dérivée, **anti-windup** par recalcul quand le PWM sature.
- **Identification de système** : essai indiciel → modèle 1er ordre + retard (méthode 28 % / 63 %).
- **Auto-réglage SIMC** : `Kp = τ / (K (τc + θ))`, `Ti = min(τ, 4 (τc + θ))`.
- **Temps réel FreeRTOS** : 4 tâches à fréquences fixes (`vTaskDelayUntil`), deux cœurs, sections critiques.
- **Mesure automatique des performances** à chaque changement de consigne (dépassement, temps de réponse).
- **Code C portable testé sur PC** : le même `pid.c` tourne sur l'ESP32 et dans les tests (CI GitHub).

## Architecture

```
taskInput (20 Hz)   potentiomètre → consigne, boutons, commandes série
      │
taskControl (100 Hz) codeur → vitesse → PID → PWM        (cœur 1, priorité 4)
      │                          ▲
taskMotor (1 kHz)   moteur DC + codeur simulés ────┘     (cœur 1, priorité 5)
      │
taskDisplay (10 Hz) OLED : courbe consigne / vitesse      (cœur 0)
```

| Fichier | Rôle |
|---|---|
| `src/pid.c` | régulateur PID (anti-windup, dérivée filtrée) |
| `src/tuning.c` | identification du moteur + réglage SIMC |
| `src/motor_model.c` | jumeau numérique du moteur (R, L, Ke, Kt, J, frottements) |
| `src/main.cpp` | tâches FreeRTOS, PWM réel (LEDC), OLED, commandes |
| `test/test_pid.c` | 9 tests : identification, performances, perturbation, anti-windup |

## Lancer la démo (Wokwi dans VS Code)

1. Ouvre ce dossier dans VS Code → PlatformIO **Build**.
2. **F1 › Wokwi: Start Simulator**.
3. Tourne le **potentiomètre** : la consigne change, l'écran trace la consigne (pointillés) et la vitesse.
   Avec les gains de départ, la vitesse **oscille** (dépassement ≈ 25 %).
4. Appuie sur le bouton **bleu « Auto-reglage »** : 5 s d'identification, puis nouveaux gains.
   Tourne à nouveau le potentiomètre : la vitesse rejoint la consigne **sans dépassement**.
5. Appuie sur le bouton **rouge « Frein »** : la vitesse chute puis revient à la consigne.

Commandes dans le moniteur série : `kp 0.0005`, `ki 0.004`, `kd 0`, `aw off`, `tune`, `reset`, `csv on`.

## Tests

```bash
gcc -O2 -Wall -Wextra -o t test/test_pid.c src/pid.c src/motor_model.c src/tuning.c -lm && ./t
```
