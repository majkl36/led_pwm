#include <Arduino.h>

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

// ===== STAV =====
static uint32_t baseline = 0;
enum Polarity { POL_UNKNOWN=0, POL_NEG=-1, POL_POS=1 };
static Polarity pol = POL_UNKNOWN;

static bool pressed = false;
static uint32_t tEdge = 0;

static bool ledOn = false; // 0% / 100%

// --- pomocné ---
static inline void setLed(bool on) {
  ledOn = on;
  int pwm = on ? MAX_PWM : 0;        // 100% / 0%
  int inv = MAX_PWM - pwm;           // TC4426A je inverting
  ledcWrite(PWM_CHANNEL, inv);
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
  setLed(false); // začni vypnuté (0%)

  calibrateBaseline(800);

  USBSerial.println("\n== Minimal Touch + PWM ==");
  USBSerial.printf("PWM: %d Hz, %d bit\n", PWM_FREQ, PWM_RESOLUTION);
  USBSerial.printf("Touch pin: GPIO%d\n", TOUCH_GPIO);
  USBSerial.printf("Baseline: %u\n", baseline);
  USBSerial.printf("Trig %d%%, Release %d%%\n", TRIG_PCT, RELEASE_PCT);
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
        // RISING EDGE -> toggle LED
        setLed(!ledOn);
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

  // debug každých ~300 ms
  static uint32_t tDbg = 0;
  if (now - tDbg > 300) {
    tDbg = now;
    USBSerial.printf("raw=%u base=%u |Δ|=%u trig=%u rel=%u pol=%d LED=%d\n",
                     raw, baseline, absDiff, trig, rel, (int)pol, (int)ledOn);
  }

  delay(2);
}
