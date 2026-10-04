/*
 * ============================================================================
 *  Régulateur PID de vitesse moteur — ESP32 + FreeRTOS (simulé sur Wokwi)
 * ============================================================================
 *  Le potentiomètre règle la consigne (0 à 3000 tr/min). Le régulateur PID
 *  calcule le PWM à envoyer au moteur pour tenir cette vitesse, même quand on
 *  freine l'arbre (bouton rouge). Le bouton bleu lance l'AUTO-RÉGLAGE :
 *  la carte identifie le moteur toute seule puis calcule les meilleurs gains.
 *
 *  Tâches FreeRTOS (cœur 1 = temps réel, cœur 0 = interface) :
 *    taskMotor    1 kHz  prio 5  « matériel simulé » : moteur DC + codeur
 *    taskControl  100 Hz prio 4  mesure codeur → PID → PWM, auto-réglage, métriques
 *    taskInput    20 Hz  prio 2  potentiomètre, boutons, commandes série
 *    taskDisplay  10 Hz  prio 1  écran OLED : courbe consigne / vitesse
 *
 *  Dashboard web servi par l'ESP32 : http://localhost:8182 (redirection Wokwi).
 *
 *  Le code du PID (pid.c), du modèle (motor_model.c) et de l'auto-réglage
 *  (tuning.c) est en C portable, testé sur PC (test/test_pid.c).
 * ============================================================================
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_arduino_version.h"
#include "pid.h"
#include "motor_model.h"
#include "tuning.h"
#include "web_page.h"

#define PIN_POT       34
#define PIN_BTN_LOAD  26
#define PIN_BTN_TUNE  27
#define PIN_LED_PWM   2
#define SP_MAX        3000.0f
#define TS            0.01f          // période du régulateur : 10 ms
#define LOAD_TORQUE   0.08f          // couple du frein (N·m)

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// --- État partagé (protégé par une section critique) ------------------------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
motor_t motor;                 // le « moteur » (accédé seulement par taskMotor + sections critiques)
volatile float g_duty = 0;     // PWM appliqué (0..1)
volatile float g_load = 0;     // couple de charge demandé
volatile float g_setpoint = 1500;
volatile float g_rpm = 0;

enum Mode { MODE_PID, MODE_TUNE };
volatile Mode g_mode = MODE_PID;
volatile int  g_tuneProgress = 0;          // 0..100 %
volatile bool g_tuneRequest = false;
volatile bool g_csv = false;               // journal CSV (commande « csv on »)

pid_ctrl_t pid;
SemaphoreHandle_t pidMutex;                // gains modifiables depuis la liaison série

// Métriques de la dernière réponse à un échelon
struct Metrics { float overshoot, tSettle; bool valid, running; };
volatile Metrics g_metrics = {0, 0, false, false};

// Historique pour la courbe (128 points, un toutes les 50 ms ≈ 6,4 s)
#define HIST 128
float histSp[HIST], histY[HIST];
volatile int histHead = 0;

// Historique pour la page web (20 points/s, avec numéro de séquence)
struct Sample { uint32_t seq, t; float sp, y, u; };
#define WEB_HIST 256
Sample webHist[WEB_HIST];
volatile uint32_t webSeq = 0;
volatile float g_spWeb = -1;               // consigne réglée depuis la page web (-1 = potentiomètre)
fopdt_t g_model = {0, 0, 0};               // dernier modèle identifié
WebServer server(80);

// ---------------------------------------------------------------------------
//  PWM réel (LED dont la luminosité suit le rapport cyclique)
// ---------------------------------------------------------------------------
static void pwmInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LED_PWM, 20000, 10);
#else
  ledcSetup(0, 20000, 10);
  ledcAttachPin(PIN_LED_PWM, 0);
#endif
}
static void pwmWrite(float duty) {
  uint32_t v = (uint32_t)(duty * 1023.0f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED_PWM, v);
#else
  ledcWrite(0, v);
#endif
}

// ---------------------------------------------------------------------------
//  Matériel simulé : moteur + codeur, intégré à 1 kHz
// ---------------------------------------------------------------------------
void taskMotor(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1);                      // 1 tick = 1 ms
    portENTER_CRITICAL(&mux);
    motor.load = g_load;
    motor_step(&motor, g_duty, 0.001f);
    portEXIT_CRITICAL(&mux);
  }
}

static int32_t readEncoder() {
  portENTER_CRITICAL(&mux);
  int32_t c = motor_encoder(&motor);
  portEXIT_CRITICAL(&mux);
  return c;
}

// ---------------------------------------------------------------------------
//  Régulation 100 Hz + auto-réglage + mesure des performances
// ---------------------------------------------------------------------------
void taskControl(void *) {
  TickType_t last = xTaskGetTickCount();
  int32_t lastCount = readEncoder();
  float y = 0, u = 0, spRef = g_setpoint;
  bool forceStep = false;
  // métriques
  float stepFrom = 0, stepTo = 0, peak = 0, inBandSince = -1; uint32_t stepTick = 0;
  // auto-réglage
  static float tuneBuf[300];
  int tunePhase = 0, tuneK = 0;
  uint32_t logDiv = 0;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    int32_t c = readEncoder();
    y = (float)(c - lastCount) / MOTOR_CPR / TS * 60.0f;     // vitesse en tr/min
    lastCount = c;
    g_rpm = y;
    float sp = g_setpoint;

    if (g_tuneRequest) { g_tuneRequest = false; g_mode = MODE_TUNE; tunePhase = 0; tuneK = 0; }

    if (g_mode == MODE_TUNE) {
      // Phase 0 : PWM fixe 30 % pendant 2 s (régime établi)
      // Phase 1 : échelon à 60 % pendant 3 s, on enregistre la vitesse
      if (tunePhase == 0) {
        u = 0.3f;
        if (++tuneK >= 200) { tunePhase = 1; tuneK = 0; tuneBuf[0] = y; }
        g_tuneProgress = tuneK * 40 / 200;
      } else {
        u = 0.6f;
        tuneBuf[++tuneK] = y;
        g_tuneProgress = 40 + tuneK * 60 / 299;
        if (tuneK >= 299) {
          fopdt_t m;
          if (tune_identify(tuneBuf, 300, TS, 0.3f, &m)) {
            float kp, ki;
            tune_simc(&m, 0.1f, &kp, &ki);
            g_model = m;
            xSemaphoreTake(pidMutex, portMAX_DELAY);
            pid.kp = kp; pid.ki = ki; pid.kd = 0;
            pid_reset(&pid);
            pid.integ = u;                                  // transition sans à-coup
            xSemaphoreGive(pidMutex);
            Serial.printf("# AUTO-REGLAGE : modele K=%.0f tr/min, tau=%.3f s, retard=%.3f s\n", m.K, m.tau, m.theta);
            Serial.printf("# AUTO-REGLAGE : nouveaux gains Kp=%.6f Ki=%.6f (methode SIMC)\n", kp, ki);
          } else {
            Serial.println("# AUTO-REGLAGE : echec de l'identification, gains inchanges");
          }
          g_mode = MODE_PID;
          forceStep = true;                                // mesure la performance des nouveaux gains
        }
      }
    } else {
      xSemaphoreTake(pidMutex, portMAX_DELAY);
      u = pid_update(&pid, sp, y);
      xSemaphoreGive(pidMutex);
    }
    g_duty = u;
    pwmWrite(u);

    // --- Mesure automatique des performances à chaque changement de consigne
    //     (échelon d'au moins 200 tr/min ; si on tourne encore le bouton, on suit)
    if (g_mode == MODE_PID) {
      if (forceStep || fabsf(sp - spRef) >= 200) {
        stepFrom = y; stepTo = sp; spRef = sp; peak = y; inBandSince = -1;
        stepTick = xTaskGetTickCount(); forceStep = false;
        g_metrics.running = true; g_metrics.valid = false;
      } else if (g_metrics.running && sp != stepTo) {
        stepTo = sp; spRef = sp; peak = y; inBandSince = -1; stepTick = xTaskGetTickCount();
      }
    }
    float t = (xTaskGetTickCount() - stepTick) * portTICK_PERIOD_MS / 1000.0f;
    if (g_metrics.running) {
      float span = fabsf(stepTo - stepFrom);
      bool up = stepTo > stepFrom;
      if (up ? y > peak : y < peak) peak = y;
      bool inBand = fabsf(y - stepTo) <= fmaxf(0.05f * span, 20.0f);
      if (inBand) { if (inBandSince < 0) inBandSince = t; }
      else inBandSince = -1;
      if (inBandSince >= 0 && t - inBandSince >= 0.5f) {            // stable depuis 0,5 s
        g_metrics.overshoot = span > 0 ? fmaxf(0, (up ? peak - stepTo : stepTo - peak) / span * 100) : 0;
        g_metrics.tSettle = inBandSince;
        g_metrics.valid = true; g_metrics.running = false;
        Serial.printf("# PERFORMANCE : depassement %.1f %%, stabilisation a 5 %% en %.2f s\n",
                      g_metrics.overshoot, g_metrics.tSettle);
      } else if (t > 10) { g_metrics.running = false; }              // jamais stabilisé
    }

    // --- Historique (toutes les 50 ms) et journal CSV (toutes les 100 ms)
    if (++logDiv % 5 == 0) {
      int h = histHead;
      histSp[h] = sp; histY[h] = y;
      histHead = (h + 1) % HIST;
      uint32_t q = webSeq;
      webHist[q % WEB_HIST] = { q + 1, (uint32_t)millis(), sp, y, u };
      webSeq = q + 1;
    }
    if (g_csv && logDiv % 10 == 0)                                  // CSV pour tracer une courbe
      Serial.printf("%lu,%.0f,%.0f,%.3f\n", (unsigned long)millis(), sp, y, u);
    else if (!g_csv && logDiv % 100 == 0)                           // résumé chaque seconde
      Serial.printf("consigne %4.0f tr/min | vitesse %4.0f tr/min | PWM %3.0f %%\n", sp, y, u * 100);
  }
}

// ---------------------------------------------------------------------------
//  Entrées : potentiomètre, boutons, commandes série
// ---------------------------------------------------------------------------
static void handleCommand(String line) {
  line.trim();
  float v = line.substring(line.indexOf(' ') + 1).toFloat();
  xSemaphoreTake(pidMutex, portMAX_DELAY);
  if (line.startsWith("kp ")) pid.kp = v;
  else if (line.startsWith("ki ")) pid.ki = v;
  else if (line.startsWith("kd ")) pid.kd = v;
  else if (line == "aw on") pid.antiwindup = 1;
  else if (line == "aw off") pid.antiwindup = 0;
  else if (line == "reset") { pid.kp = 0.0002f; pid.ki = 0.006f; pid.kd = 0; pid_reset(&pid); }
  xSemaphoreGive(pidMutex);
  if (line == "tune") g_tuneRequest = true;
  if (line == "csv on") { g_csv = true; Serial.println("t_ms,consigne_tr_min,vitesse_tr_min,pwm"); }
  if (line == "csv off") g_csv = false;
  Serial.printf("# gains : Kp=%.6f Ki=%.6f Kd=%.6f anti-windup=%s\n", pid.kp, pid.ki, pid.kd,
                pid.antiwindup ? "oui" : "non");
}

void taskInput(void *) {
  float potFilt = analogRead(PIN_POT);
  bool lastLoad = HIGH, lastTune = HIGH;
  String line;
  for (;;) {
    potFilt += 0.3f * (analogRead(PIN_POT) - potFilt);
    float sp = roundf(potFilt / 4095.0f * SP_MAX / 50.0f) * 50.0f;   // pas de 50 tr/min
    g_setpoint = g_spWeb >= 0 ? g_spWeb : sp;

    bool l = digitalRead(PIN_BTN_LOAD), tb = digitalRead(PIN_BTN_TUNE);
    if (l == LOW && lastLoad == HIGH) {
      g_load = g_load > 0 ? 0 : LOAD_TORQUE;
      Serial.printf("# FREIN %s (%.2f N.m)\n", g_load > 0 ? "serre" : "relache", (float)g_load);
    }
    if (tb == LOW && lastTune == HIGH && g_mode == MODE_PID) {
      g_tuneRequest = true;
      Serial.println("# AUTO-REGLAGE demarre : identification du moteur (5 s)");
    }
    lastLoad = l; lastTune = tb;

    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') { if (line.length()) handleCommand(line); line = ""; }
      else if (line.length() < 32) line += ch;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------------------------------------------------------------------
//  Écran OLED : courbe consigne (pointillés) et vitesse (trait plein)
// ---------------------------------------------------------------------------
void taskDisplay(void *) {
  const int gTop = 10, gH = 42;
  auto yPix = [&](float rpm) { int p = gTop + gH - 1 - (int)(rpm / 3300.0f * (gH - 1)); return constrain(p, gTop, gTop + gH - 1); };
  for (;;) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.printf("C:%4.0f  V:%4.0f %3.0f%%", (float)g_setpoint, (float)g_rpm, g_duty * 100);

    int head = histHead;
    int prevY = -1;
    for (int x = 0; x < HIST; x++) {
      int i = (head + x) % HIST;
      if (x % 3 == 0) oled.drawPixel(x, yPix(histSp[i]), SSD1306_WHITE);  // consigne en pointillés
      int py = yPix(histY[i]);
      if (prevY >= 0) oled.drawLine(x - 1, prevY, x, py, SSD1306_WHITE);
      prevY = py;
    }

    oled.setCursor(0, 56);
    if (g_mode == MODE_TUNE) oled.printf("AUTO-REGLAGE %3d%%", (int)g_tuneProgress);
    else if (g_load > 0) oled.print("FREIN SERRE");
    else if (g_metrics.valid) oled.printf("Dep:%4.1f%% t5%%:%4.2fs", (float)g_metrics.overshoot, (float)g_metrics.tSettle);
    else if (g_metrics.running) oled.print("mesure en cours...");
    else oled.printf("Kp%.4f Ki%.4f", pid.kp, pid.ki);
    oled.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------------------------------------------------------------------------
//  Dashboard web
// ---------------------------------------------------------------------------
static void webState() {
  static char buf[12288];
  uint32_t since = server.arg("since").toInt(), seq = webSeq;
  uint32_t first = seq > WEB_HIST ? seq - WEB_HIST : 0;
  if (since > first) first = since < seq ? since : seq;
  Metrics m; memcpy(&m, (const void *)&g_metrics, sizeof m);
  int n = snprintf(buf, sizeof buf,
    "{\"sp\":%.0f,\"y\":%.0f,\"u\":%.3f,\"mode\":\"%s\",\"progress\":%d,\"brake\":%d,\"aw\":%d,\"kp\":%.6f,\"ki\":%.6f,"
    "\"spWeb\":%.0f,\"metrics\":{\"valid\":%d,\"running\":%d,\"os\":%.2f,\"ts\":%.3f},"
    "\"model\":{\"K\":%.1f,\"tau\":%.4f,\"theta\":%.4f},\"samples\":[",
    (float)g_setpoint, (float)g_rpm, (float)g_duty, g_mode == MODE_TUNE ? "tune" : "pid", (int)g_tuneProgress,
    g_load > 0, pid.antiwindup, pid.kp, pid.ki, (float)g_spWeb, m.valid, m.running, m.overshoot, m.tSettle,
    g_model.K, g_model.tau, g_model.theta);
  for (uint32_t q = first; q < seq && n < (int)sizeof buf - 100; q++) {
    const Sample &p = webHist[q % WEB_HIST];
    n += snprintf(buf + n, sizeof buf - n, "%s{\"seq\":%lu,\"t\":%lu,\"sp\":%.0f,\"y\":%.0f,\"u\":%.1f}", q == first ? "" : ",",
                  (unsigned long)p.seq, (unsigned long)p.t, p.sp, p.y, p.u * 100);
  }
  snprintf(buf + n, sizeof buf - n, "]}");
  server.send(200, "application/json", buf);
}

static void webCmd() {
  String c = server.arg("c"), v = server.arg("v");
  if (c == "sp") g_spWeb = constrain(v.toFloat(), -1.0f, SP_MAX);
  else if (c == "tune") { if (g_mode == MODE_PID) { g_tuneRequest = true; Serial.println("# AUTO-REGLAGE demarre (page web)"); } }
  else if (c == "brake") { g_load = g_load > 0 ? 0 : LOAD_TORQUE; Serial.printf("# FREIN %s (page web)\n", g_load > 0 ? "serre" : "relache"); }
  else if (c == "aw") handleCommand(pid.antiwindup ? "aw off" : "aw on");
  else if (c == "reset") handleCommand("reset");
  else if (c == "kp" || c == "ki") handleCommand(c + " " + v);
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_LOAD, INPUT_PULLUP);
  pinMode(PIN_BTN_TUNE, INPUT_PULLUP);
  pwmInit();
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("# ERREUR : ecran OLED introuvable");

  motor_init(&motor);
  pid_init(&pid, 0.0002f, 0.006f, 0.0f, TS);        // réglage manuel volontairement médiocre
  pidMutex = xSemaphoreCreateMutex();
  for (int i = 0; i < HIST; i++) { histSp[i] = 0; histY[i] = 0; }

  Serial.println("# Regulateur PID moteur pret. Potentiometre = consigne, rouge = frein, bleu = auto-reglage");
  Serial.println("# Commandes : kp <v>, ki <v>, kd <v>, aw on|off, tune, reset, csv on|off");

  xTaskCreatePinnedToCore(taskMotor,   "motor",   2048, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(taskControl, "control", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskInput,   "input",   4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskDisplay, "display", 4096, NULL, 1, NULL, 0);

  WiFi.begin("Wokwi-GUEST", "", 6);
  for (int k = 0; k < 40 && WiFi.status() != WL_CONNECTED; k++) delay(250);
  server.on("/", []() { server.send(200, "text/html; charset=utf-8", WEB_PAGE); });
  server.on("/api/state", webState);
  server.on("/api/cmd", webCmd);
  server.begin();
  Serial.printf("# Dashboard web : http://localhost:8182 (Wi-Fi %s)\n", WiFi.status() == WL_CONNECTED ? "OK" : "non connecte");
}

void loop() {
  server.handleClient();
  delay(2);
}
