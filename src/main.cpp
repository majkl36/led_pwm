#include <Arduino.h>

static const int PWM_PIN = 1;
static const int PWM_CHANNEL = 0;
static const int PWM_FREQ = 20000;
static const int PWM_RESOLUTION = 10;
static const int MAX_PWM_VALUE = (1 << PWM_RESOLUTION) - 1;
static const int TOUCH_PIN = 9;
static const int NUM_LEVELS = 5;
static const int intensity_levels[NUM_LEVELS] = {20, 40, 60, 80, 100}; // percent


void setup() {
  USBSerial.begin(115200);
  delay(1000);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  pinMode(TOUCH_PIN, INPUT);
  // TC4426A is inverting -> MAX_PWM_VALUE = ON, 0 = OFF
  ledcWrite(PWM_CHANNEL, MAX_PWM_VALUE); //start with LED OFF
  USBSerial.println("LEDC: 20kHz, 10bit, inverted logic. Touch control on pin 3.");
}


void loop() {
  static int level = 0;
  static bool last_touch = false;
  bool touch = digitalRead(TOUCH_PIN) == HIGH;

  if (touch && !last_touch) {
    // On rising edge, cycle to next intensity level
    level = (level + 1) % NUM_LEVELS;
    int percent = intensity_levels[level];
    int pwm_value = (MAX_PWM_VALUE * percent) / 100;
    int inv_pwm = MAX_PWM_VALUE - pwm_value; // invert for TC4426A
    ledcWrite(PWM_CHANNEL, inv_pwm);
    USBSerial.print("Touch detected. Intensity: ");
    USBSerial.print(percent);
    USBSerial.print("% (PWM: ");
    USBSerial.print(inv_pwm);
    USBSerial.println(")");
    delay(200); // debounce
  }
  last_touch = touch;
  delay(10);
}