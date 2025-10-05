#include <Arduino.h>
#include <esp_task_wdt.h>

// ======= PWM (TC4426A invert) =======
static const int PWM_PIN        = 1;
static const int PWM_CHANNEL    = 0;
static const int PWM_FREQ       = 20000;     // 20 kHz
static const int PWM_RESOLUTION = 10;        // 10-bit
static const int MAX_PWM_VALUE  = (1 << PWM_RESOLUTION) - 1;

// ======= TOUCH =======
static const int  TOUCH_GPIO      = 9;       // touch-capable pin
static const int  TOUCH_SAMPLES   = 20;      // ~26 ms okno
static const int  TOUCH_SAMPLE_US = 1300;
static const int  TOUCH_DEBOUNCE  = 90;      // ms
static const int  HOLD_OFF_MS     = 300;     // po dotyku ignorujeme ďalšie
static const int  POST_RELEASE_BLANK_MS = 800;

static const int  TRIG_PCT        = 16;      // spúšť pri |Δ| > 16 % baseline
static const int  RELEASE_PCT     = 10;       // hysterézia pre pustenie
static const int  FREEZE_PCT      = 5;       // baseline sa neadaptuje pri |Δ| > 5 %

// ======= STUCK FAILSAFE =======
static const int  STUCK_MS        = 8000;    // 8 s držaný dotyk -> failsafe
static const int  STUCK_BLANK_MS  = 1200;    // po odpichu ignoruj vstup

// ======= RAMP (plynulý nábeh) =======
static const int  RAMP_MS         = 350;     // dĺžka rampy 0↔100% (celý rozsah)
static const int  RAMP_STEP_MS    = 16;       // krok rampy

// ======= DEBUG =======
static const bool PRINT_DEBUG     = true;

// ------- Vnútorné stavy -------
static uint32_t baseline = 0;

enum Polarity { POL_UNKNOWN=0, POL_NEG=-1, POL_POS=1 };
static Polarity pol = POL_UNKNOWN;

static bool     pressed          = false;
static uint32_t tEdge            = 0;        // pre debounce
static uint32_t tRelease         = 0;        // pre post-release blank
static uint32_t holdOffUntil     = 0;        // hold-off po dotyku
static uint32_t pressedSince     = 0;        // kedy dotyk začal

// Dva levely 0% / 100%
static int currentPercent = 0;   // skutočne nastavené (stav rampy)
static int targetPercent  = 0;   // cieľ rampy (0 alebo 100)
static uint32_t lastRampStep = 0;

// ---------- Pomocné ----------
static inline void setPWMFromPercent(int percent) {
  percent = constrain(percent, 0, 100);
  int pwm_value = (MAX_PWM_VALUE * percent) / 100;
  int inv_pwm   = MAX_PWM_VALUE - pwm_value;   // invert kvôli TC4426A
  ledcWrite(PWM_CHANNEL, inv_pwm);
}

static inline void setTargetPercent(int percent) {
  targetPercent = constrain(percent, 0, 100);
}

static inline void updateRamp() {
  if (currentPercent == targetPercent) return;
  uint32_t now = millis();
  if (now - lastRampStep < (uint32_t)RAMP_STEP_MS) return;
  lastRampStep = now;

  // počet krokov pre celý rozsah
  int totalSteps = max(1, RAMP_MS / RAMP_STEP_MS);
  // koľko percent zmeniť na jeden krok pre lineárnu rampu
  int stepMag = max(1, 100 / totalSteps);

  int delta = targetPercent - currentPercent;
  int step  = (delta > 0) ? stepMag : -stepMag;

  // ak už sme blízko cieľa a krok by preletel, zacvakni na cieľ
  if (abs(delta) <= stepMag) currentPercent = targetPercent;
  else                       currentPercent += step;

  setPWMFromPercent(currentPercent);
}

static inline uint32_t touchSampleAvg() {
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
  while (millis() - t0 < ms) { acc += touchSampleAvg(); n++; delay(5); }
  baseline = (n ? (acc / n) : touchSampleAvg());
}

void setup() {
  USBSerial.begin(115200);
  delay(200);

  // PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  currentPercent = 0;                 // začni vypnuté
  targetPercent  = 0;
  setPWMFromPercent(currentPercent);

  calibrateBaseline(1200);

  if (PRINT_DEBUG) {
    USBSerial.println("\n== Touch + PWM init ==");
    USBSerial.printf("PWM: %d Hz, %d bit\n", PWM_FREQ, PWM_RESOLUTION);
    USBSerial.printf("Touch pin: GPIO%d\n", TOUCH_GPIO);
    USBSerial.printf("Baseline: %u\n", baseline);
    USBSerial.printf("Trigger: |raw-baseline| > %d%%, release < %d%% (hyst)\n",
                     TRIG_PCT, RELEASE_PCT);
  }

  // WATCHDOG (3 s)
  esp_task_wdt_init(3, true);   // reset CPU pri timeoute
  esp_task_wdt_add(NULL);       // pridaj aktuálnu task
}

void loop() {
  esp_task_wdt_reset(); // kŕm WDT

  // ---- Touch odmeranie ----
  uint32_t raw = touchSampleAvg();
  int64_t  diff = (int64_t)raw - (int64_t)baseline;
  uint32_t absDiff = (diff >= 0) ? diff : -diff;

  uint32_t trig = (uint32_t)((uint64_t)baseline * TRIG_PCT    / 100);
  uint32_t rel  = (uint32_t)((uint64_t)baseline * RELEASE_PCT / 100);

  // adaptácia baseline len keď je pokoj a mimo blankingu
  bool inBlank = (millis() - tRelease < (uint32_t)POST_RELEASE_BLANK_MS);
  if (!pressed && !inBlank) {
    if (absDiff <= (uint32_t)((uint64_t)baseline * FREEZE_PCT / 100)) {
      baseline = (baseline * 999 + raw) / 1000; // pomalá EMA
    }
  }

  // autodetekcia polarity (len naozaj veľké odchýlky)
  bool overTrig = (absDiff > trig);
  if (pol == POL_UNKNOWN && overTrig) {
    pol = (diff > 0) ? POL_POS : POL_NEG;
    if (PRINT_DEBUG) USBSerial.println(pol == POL_POS ? "POL=POS (raw stupa pri dotyku)"
                                                      : "POL=NEG (raw klesa pri dotyku)");
  }

  // kandidát na dotyk podľa polarity + hysterezia
  bool candidate = false;
  if (pol == POL_POS) {
    bool triggerPos = (diff > (int64_t)trig);
    bool releasePos = (diff < (int64_t)rel);
    candidate = pressed ? !releasePos : triggerPos;
  } else if (pol == POL_NEG) {
    bool triggerNeg = ((-diff) > (int64_t)trig);
    bool releaseNeg = ((-diff) < (int64_t)rel);
    candidate = pressed ? !releaseNeg : triggerNeg;
  } else {
    candidate = false; // kým nevieme polaritu
  }

  // hold-off/blanking brány
  uint32_t nowMs = millis();
  bool holdOffActive = (nowMs < holdOffUntil);
  if (inBlank && !pressed) candidate = false;         // po release ignoruj
  if (holdOffActive && !pressed) candidate = false;   // krátko po dotyku ignoruj

  // debounce + edge
  if (candidate != pressed) {
    if (nowMs - tEdge > (uint32_t)TOUCH_DEBOUNCE) {
      pressed = candidate;
      tEdge   = nowMs;

      if (pressed) {
        // TOUCH -> toggle cieľ a spusti rampu
        pressedSince = nowMs;
        holdOffUntil = nowMs + HOLD_OFF_MS;
        int next = (targetPercent == 0) ? 100 : 0;
        setTargetPercent(next);
        if (PRINT_DEBUG) {
          USBSerial.printf("TOUCH -> target %d%%  raw=%u base=%u |Δ|=%u\n",
                            next, raw, baseline, absDiff);
        }
      } else {
        // RELEASE
        tRelease = nowMs;
        if (PRINT_DEBUG) USBSerial.println("RELEASE");
      }
    }
  } else {
    tEdge = nowMs; // stabilný stav drží debounce okno resetované
  }

  // STUCK FAILSAFE
  if (pressed && (nowMs - pressedSince > (uint32_t)STUCK_MS)) {
    // „odpichni“ dotyk, spusti dlhší blanking a nechaj rampu bežať ako je
    pressed = false;
    tRelease = nowMs;
    holdOffUntil = nowMs + STUCK_BLANK_MS;
    if (PRINT_DEBUG) USBSerial.println("FAILSAFE: stuck touch -> force RELEASE + blank");
  }

  // RAMP update (plynulé dobiehanie k targetu)
  updateRamp();

  // občasný debug
  if (PRINT_DEBUG) {
    static uint32_t tDbg = 0;
    if (nowMs - tDbg > 250) {
      tDbg = nowMs;
      USBSerial.printf("raw=%u base=%u |Δ|=%u trig=%u rel=%u pol=%d cur=%d%% tgt=%d%%\n",
                       raw, baseline, absDiff, trig, rel, (int)pol, currentPercent, targetPercent);
    }
  }

  // krátky oddych (bezpečné pre WDT)
  delay(2);
}
