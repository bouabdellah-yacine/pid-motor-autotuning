# 🎛️ PID Motor Speed Controller with Auto-Tuning (ESP32 + FreeRTOS)

[![Tests](https://github.com/bouabdallah-yacine/pid-motor-autotuning/actions/workflows/ci.yml/badge.svg)](https://github.com/bouabdallah-yacine/pid-motor-autotuning/actions/workflows/ci.yml)

An ESP32 holds the speed of a DC motor exactly at the setpoint, even when the shaft is braked.
A single button starts **auto-tuning**: the board identifies the motor on its own, then computes the
controller gains using the **SIMC** method (Skogestad), widely used in industry.

> ✅ Simulated on **Wokwi** (VS Code). The motor is a **digital twin** (electrical + mechanical physical
> model + 4096 counts/rev encoder) running at 1 kHz: the controller only sees the PWM output and the encoder,
> just like on real hardware.

## Results (measured by the automated tests)

| | Manual tuning | **After auto-tuning** |
|---|---|---|
| Overshoot (500 → 2000 rpm step) | 25% | **0.1%** |
| Settling time (±5%) | > 3 s, oscillating | **0.38 s** |
| Steady-state error | ≈ 0.5% | **0.00%** |
| 0.08 N·m braking load | — | **final error 0.00%** |

Anti-windup: after a 2 s motor stall, overshoot drops from **115% to 35%**.

## What the project demonstrates

- **Industrial-grade discrete PID**: derivative on measurement (no kick when the setpoint changes),
  derivative filter, back-calculation **anti-windup** when the PWM saturates.
- **System identification**: step test → first-order-plus-dead-time model (28% / 63% two-point method).
- **SIMC auto-tuning**: `Kp = τ / (K (τc + θ))`, `Ti = min(τ, 4 (τc + θ))`.
- **FreeRTOS real-time design**: 4 fixed-rate tasks (`vTaskDelayUntil`), two cores, critical sections.
- **Automatic performance measurement** on every setpoint change (overshoot, settling time).
- **Portable C code tested on a PC**: the same `pid.c` runs on the ESP32 and in the tests (GitHub CI).

## Architecture

```
taskInput (20 Hz)   potentiometer → setpoint, buttons, serial commands
      │
taskControl (100 Hz) encoder → speed → PID → PWM         (core 1, priority 4)
      │                          ▲
taskMotor (1 kHz)   simulated DC motor + encoder ───┘    (core 1, priority 5)
      │
taskDisplay (10 Hz) OLED: setpoint / speed plot           (core 0)
```

| File | Role |
|---|---|
| `src/pid.c` | PID controller (anti-windup, filtered derivative) |
| `src/tuning.c` | motor identification + SIMC tuning |
| `src/motor_model.c` | motor digital twin (R, L, Ke, Kt, J, friction) |
| `src/main.cpp` | FreeRTOS tasks, hardware PWM (LEDC), OLED, commands, web server |
| `src/web_page.h` | embedded web dashboard: live chart, gains, identified model (light / dark theme) |
| `test/test_pid.c` | 9 tests: identification, performance, disturbance rejection, anti-windup |

## Running the demo (Wokwi in VS Code)

1. Open this folder in VS Code → PlatformIO **Build**.
2. **F1 › Wokwi: Start Simulator**.
3. Open **http://localhost:8182**: the dashboard (live chart, gains, auto-tuning, brake, setpoint).
   Turn the **potentiometer** (or the slider on the page): the setpoint changes and the display plots the setpoint (dotted) and the speed.
   With the initial gains, the speed **oscillates** (overshoot ≈ 25%).
4. Press the **blue "Auto-tune"** button: 5 s of identification, then new gains are applied.
   Turn the potentiometer again: the speed now reaches the setpoint **without overshoot**.
5. Press the **red "Brake"** button: the speed drops, then recovers to the setpoint.

Serial monitor commands: `kp 0.0005`, `ki 0.004`, `kd 0`, `aw off`, `tune`, `reset`, `csv on`.

## Tests

```bash
gcc -O2 -Wall -Wextra -o t test/test_pid.c src/pid.c src/motor_model.c src/tuning.c -lm && ./t
```

## License

© 2026 Yacine — all rights reserved. Code published for viewing purposes only (see [`LICENSE`](LICENSE)).
