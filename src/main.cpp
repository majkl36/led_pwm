#include <Arduino.h>

// ======= PWM (TC4426A invert) =======
static const int PWM_PIN        = 1;
static const int PWM_CHANNEL    = 0;
static const int PWM_FREQ       = 20000;
static const int PWM_RESOLUTION = 10;
static const int MAX_PWM_VALUE  = (1 << PWM_RESOLUTION) - 1;

// ======= TOUCH =======
static const int  TOUCH_GPIO      = 9;     // touch-capable pin
static const int  TOUCH_SAMPLES   = 20;    // ~20 * 1.3ms ≈ 26ms
static const int  TOUCH_SAMPLE_US = 1300;  // rozumné rozptylové oneskorenie
static const int  TOUCH_DEBOUNCE  = 90;    // ms
static const int  TOUCH_HOLDOFF   = 300;   // ms
static const int  TOUCH_CAL_MS    = 1200;  // ms
static const int  TRIG_PCT        = 15;    // spúšť pri |Δ| > 15% baseline
static const int  FREEZE_PCT      = 5;     // baseline sa neadaptuje pri |Δ| > 5%
static const bool PRINT_DEBUG     = true;

static uint32_t baseline = 0;

enum Polarity { POL_UNKNOWN=0, POL_NEG=-1, POL_POS=1 };
static Polarity pol = POL_UNKNOWN; // autodetekcia smeru pri prvom dotyku

// ======= Úrovne intenzity =======
static const int NUM_LEVELS = 2;
static const int intensity_levels[NUM_LEVELS] = {0, 100};

static inline uint32_t touchSampleAvg() {
  uint32_t s = 0;
  for (int i = 0; i < TOUCH_SAMPLES; ++i) {
    s += touchRead(TOUCH_GPIO);
    delayMicroseconds(TOUCH_SAMPLE_US);
  }
  return s / TOUCH_SAMPLES;
}

void calibrateBaseline(uint32_t ms) {
  uint32_t t0 = millis();
  uint64_t acc = 0; uint32_t n = 0;
  while (millis() - t0 < ms) { acc += touchSampleAvg(); n++; delay(5); }
  baseline = (n ? (acc / n) : touchSampleAvg());
}

void setIntensityPercent(int percent) {
  percent = constrain(percent, 0, 100);
  int pwm_value = (MAX_PWM_VALUE * percent) / 100;
  int inv_pwm   = MAX_PWM_VALUE - pwm_value; // invert pre TC4426A
  ledcWrite(PWM_CHANNEL, inv_pwm);
}

void setup() {
  USBSerial.begin(115200);
  delay(200);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, MAX_PWM_VALUE); // LED OFF (invert)

  calibrateBaseline(TOUCH_CAL_MS);

  USBSerial.println("\n== Touch + PWM init ==");
  USBSerial.printf("PWM: %d Hz, %d bit\n", PWM_FREQ, PWM_RESOLUTION);
  USBSerial.printf("Touch pin: GPIO%d\n", TOUCH_GPIO);
  USBSerial.printf("Baseline: %u\n", baseline);
  USBSerial.printf("Trigger: |raw-baseline| > %d%% baseline\n", TRIG_PCT);
}

void loop() {
  static int  level   = 0;
  static bool pressed = false;
  static uint32_t tEdge = 0;

  uint32_t raw = touchSampleAvg();
  // relatívna odchýlka
  int64_t diff = (int64_t)raw - (int64_t)baseline;
  uint32_t absDiff = (diff >= 0) ? diff : -diff;

  // Baseline: adaptuj len ak je odchýlka malá (<= FREEZE_PCT)
  if (!pressed) {
    if (absDiff <= (uint32_t)((uint64_t)baseline * FREEZE_PCT / 100)) {
      baseline = (baseline * 999 + raw) / 1000; // pomalá EMA
    }
  }

  // Autodetekcia polarity pri prvom „skutočnom“ dotyku
  bool overTrig = (absDiff > (uint32_t)((uint64_t)baseline * TRIG_PCT / 100));
  if (pol == POL_UNKNOWN && overTrig) {
    pol = (diff > 0) ? POL_POS : POL_NEG;
    if (PRINT_DEBUG) USBSerial.println(pol == POL_POS ? "POL=POS (raw stupa pri dotyku)" :
                                                       "POL=NEG (raw klesa pri dotyku)");
  }

  // Vyhodnotenie dotyku podľa zistenej polarity (kým ju nevieme, ignoruj eventy)
  bool now = false;
  if (pol == POL_POS)       now = overTrig && (diff > 0);
  else if (pol == POL_NEG)  now = overTrig && (diff < 0);
  else                      now = false;

  // debounce + edge detection
  if (now != pressed) {
    if (millis() - tEdge > (uint32_t)TOUCH_DEBOUNCE) {
      pressed = now;
      tEdge = millis();

      if (pressed) {
        level = (level + 1) % NUM_LEVELS;
        int percent = intensity_levels[level];
        setIntensityPercent(percent);

        if (PRINT_DEBUG) {
          USBSerial.printf("TOUCH -> level %d (%d%%)  raw=%u base=%u\n",
                            level, percent, raw, baseline);
        }
        delay(TOUCH_HOLDOFF); // proti „šúchanému“ dotyku
      } else {
        if (PRINT_DEBUG) USBSerial.println("RELEASE");
      }
    }
  } else {
    tEdge = millis();
  }

  if (PRINT_DEBUG) {
    static uint32_t tDbg = 0;
    if (millis() - tDbg > 200) {
      tDbg = millis();
      uint32_t thr = (uint32_t)((uint64_t)baseline * TRIG_PCT / 100);
      USBSerial.printf("raw=%u  base=%u  |Δ|=%u  trig=%u  pol=%d\n",
                       raw, baseline, absDiff, thr, (int)pol);
    }
  }

  delay(5);
}
