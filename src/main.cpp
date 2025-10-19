#include <Arduino.h>
#ifndef DISABLE_WDT
#include <esp_task_wdt.h>
#endif

// ===== PWM (TC4426A je inverting) =====
static const int PWM_PIN        = 1;
static const int PWM_CHANNEL    = 0;
static const int PWM_FREQ       = 20000;         // 20 kHz
static const int PWM_RESOLUTION = 10;            // 10-bit
static const int MAX_PWM        = (1 << PWM_RESOLUTION) - 1;

// ===== TOUCH =====
static const int TOUCH_GPIO      = 9;            // tvoja páska
static const int TOUCH_SAMPLES   = 16;           // koľko meraní spriemerovať
static const int TOUCH_SAMPLE_US = 1200;         // odstup meraní (~19 ms okno)
static const int TRIG_PCT        = 15;           // dotyk pri |Δ| > 15 % baseline
static const int RELEASE_PCT     = 10;           // pustenie pri |Δ| < 10 %
static const int DEBOUNCE_MS     = 120;          // jednoduchý debounce
static const int BASE_ADAPT_PCT  = 5;            // adaptovať baseline len keď |Δ| <= 5%
static const int REDETECT_IDLE_MS = 15000;       // po dlhom pokoji znovu autodetekuj polaritu

// ===== RAMP (plynulý nábeh/zhasínanie) =====
static const int RAMP_MS         = 350;          // čas na prechod 0↔MAX (ms)
static const int RAMP_STEP_MS    = 16;           // čas medzi krokmi rampy (ms)

// ===== STAV =====
static uint32_t baseline = 0;
enum Polarity { POL_UNKNOWN=0, POL_NEG=-1, POL_POS=1 };
static Polarity pol = POL_UNKNOWN;

static bool pressed = false;
static uint32_t tEdge = 0;
static uint32_t tQuietStart = 0; // začiatok "pokojného" okna (pre re-detekciu polarity)

static bool ledOn = false; // cieľ: 0% / 100%

// PWM rampa: aktuálna a cieľová hodnota (v rozsahu 0..MAX_PWM)
static int currentPwm = 0;
static int targetPwm  = 0;
static uint32_t lastRampStep = 0;

// --- pomocné ---
static inline void writePwm(int pwm) {
  pwm = constrain(pwm, 0, MAX_PWM);
  int inv = MAX_PWM - pwm;           // TC4426A je inverting
  ledcWrite(PWM_CHANNEL, inv);
}

static inline void setLedImmediate(bool on) {
  ledOn = on;
  currentPwm = on ? MAX_PWM : 0;
  targetPwm  = currentPwm;
  writePwm(currentPwm);
}

static inline void setLedTarget(bool on) {
  ledOn = on;
  targetPwm = on ? MAX_PWM : 0;
}

static inline void updateRamp() {
  if (currentPwm == targetPwm) return;
  uint32_t now = millis();
  if (now - lastRampStep < (uint32_t)RAMP_STEP_MS) return;
  lastRampStep = now;

  int totalSteps = RAMP_MS / RAMP_STEP_MS;
  if (totalSteps < 1) totalSteps = 1;
  int stepMag = MAX_PWM / totalSteps;
  if (stepMag < 1) stepMag = 1;

  int delta = targetPwm - currentPwm;
  int step = (delta > 0) ? stepMag : -stepMag;
  if (abs(delta) <= stepMag) currentPwm = targetPwm;
  else                       currentPwm += step;

  writePwm(currentPwm);
}

static uint32_t touchAvg() {
  uint32_t s = 0;
  for (int i = 0; i < TOUCH_SAMPLES; ++i) {
    s += touchRead(TOUCH_GPIO);
    delayMicroseconds(TOUCH_SAMPLE_US);
  }
  return s / TOUCH_SAMPLES;
}

static void calibrateBaseline(uint32_t ms) {
  uint32_t t0 = millis();
  uint64_t acc = 0; uint32_t n = 0;
  while (millis() - t0 < ms) { acc += touchAvg(); n++; delay(5); }
  baseline = n ? (acc / n) : touchAvg();
}

void setup() {
  USBSerial.begin(115200);
  delay(200);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  setLedImmediate(false); // začni vypnuté (0%)

  calibrateBaseline(800);

  USBSerial.println("\n== Minimal Touch + PWM ==");
  USBSerial.printf("PWM: %d Hz, %d bit\n", PWM_FREQ, PWM_RESOLUTION);
  USBSerial.printf("Touch pin: GPIO%d\n", TOUCH_GPIO);
  USBSerial.printf("Baseline: %u\n", baseline);
  USBSerial.printf("Trig %d%%, Release %d%%\n", TRIG_PCT, RELEASE_PCT);

#ifndef DISABLE_WDT
  // Task Watchdog: 3 s timeout, trigger panic on timeout for useful backtrace
  esp_task_wdt_init(3, true);
  esp_task_wdt_add(NULL); // current task (Arduino loop task)
#endif
}

void loop() {
  // --- meranie ---
  uint32_t raw = touchAvg();
  int32_t  diff = (int32_t)raw - (int32_t)baseline;
  uint32_t absDiff = (diff >= 0) ? diff : -diff;

  // prahy podľa baseline
  uint32_t trig = (uint32_t)((uint64_t)baseline * TRIG_PCT    / 100);
  uint32_t rel  = (uint32_t)((uint64_t)baseline * RELEASE_PCT / 100);

  // autodetekcia polarity len ak sme výrazne nad prahom
  if (pol == POL_UNKNOWN && absDiff > trig) {
    pol = (diff > 0) ? POL_POS : POL_NEG;
    USBSerial.println(pol == POL_POS ? "POL=POS" : "POL=NEG");
  }

  // surový kandidát dotyku (ak ešte nevieme polaritu, nedotýkame sa)
  bool cand = false;
  if (pol == POL_POS) {
    bool trigPos = (diff >  (int32_t)trig);
    bool relPos  = (diff <  (int32_t)rel);
    cand = pressed ? !relPos : trigPos;
  } else if (pol == POL_NEG) {
    bool trigNeg = ((-diff) > (int32_t)trig);
    bool relNeg  = ((-diff) < (int32_t)rel);
    cand = pressed ? !relNeg : trigNeg;
  }

  // --- jednoduchý debounce na zmenu stavu ---
  uint32_t now = millis();
  if (cand != pressed) {
    if (now - tEdge > (uint32_t)DEBOUNCE_MS) {
      pressed = cand;
      tEdge = now;

      if (pressed) {
        // RISING EDGE -> toggle LED (cieľ), rampa dobehne sama
        setLedTarget(!ledOn);
        USBSerial.printf("TOUCH -> LED %s  raw=%u base=%u |Δ|=%u\n",
                         ledOn ? "ON" : "OFF", raw, baseline, absDiff);
      } else {
        USBSerial.println("RELEASE");
      }
    }
  } else {
    tEdge = now; // stabilný stav drží okno čerstvé
  }

  // --- pomalá adaptácia baseline len v pokoji (± BASE_ADAPT_PCT) ---
  uint32_t adaptBand = (uint32_t)((uint64_t)baseline * BASE_ADAPT_PCT / 100);
  if (!pressed && absDiff <= adaptBand) {
    // jednoduchá EMA ~0.1 %
    baseline = (baseline * 999 + raw) / 1000;
  }

  // --- re-detekcia polarity po dlhom pokoji ---
  bool quiet = (!pressed && absDiff <= adaptBand);
  if (quiet) {
    if (tQuietStart == 0) tQuietStart = now;
    if (pol != POL_UNKNOWN && (now - tQuietStart) >= (uint32_t)REDETECT_IDLE_MS) {
      pol = POL_UNKNOWN;
      USBSerial.println("POL=UNKNOWN (idle re-detect)");
      // nechaj tQuietStart bežať ďalej; autodetekcia nastane pri najbližšom výraznom odchýlení
    }
  } else {
    tQuietStart = 0;
  }

  // rampa pre PWM
  updateRamp();

  // debug každých ~300 ms
  static uint32_t tDbg = 0;
  if (now - tDbg > 300) {
    tDbg = now;
    int curPct = (currentPwm * 100) / MAX_PWM;
    int tgtPct = (targetPwm  * 100) / MAX_PWM;
    USBSerial.printf("raw=%u base=%u |Δ|=%u trig=%u rel=%u pol=%d LED=%d cur=%d%% tgt=%d%%\n",
                     raw, baseline, absDiff, trig, rel, (int)pol, (int)ledOn, curPct, tgtPct);
  }

  delay(2);

#ifndef DISABLE_WDT
  // nakŕm WDT na záver iterácie loopu
  esp_task_wdt_reset();
#endif
}
