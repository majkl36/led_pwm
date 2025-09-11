#include <Arduino.h>

static const int PWM_PIN = 1;
static const int PWM_CHANNEL = 0;
static const int PWM_FREQ = 20000;
static const int PWM_RESOLUTION = 10;
static const int MAX_PWM_VALUE = (1 << PWM_RESOLUTION) - 1;

void setup() {
  Serial.begin(115200);
  delay(1000);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  // TC4426A is inverting -> MAX_PWM_VALUE = ON, 0 = OFF
  ledcWrite(PWM_CHANNEL, MAX_PWM_VALUE); //start with LED OFF
  Serial.println("LEDC: 20kHz, 11bit, inverted logic.");
}

void loop() {
  for (int i = 0; i <= MAX_PWM_VALUE; i += 8) {
    int inv_pwm = MAX_PWM_VALUE - i; // invert PWM value for TC4426A
    ledcWrite(PWM_CHANNEL, inv_pwm);
    Serial.print("PWM: ");
    Serial.println(inv_pwm);
    delay(5);
  }
  for (int i = MAX_PWM_VALUE; i >= 0; i -= 8) {
    int inv_pwm = MAX_PWM_VALUE - i; // invert PWM value for TC4426A
    ledcWrite(PWM_CHANNEL, inv_pwm);
    Serial.print("PWM: ");
    Serial.println(inv_pwm);
    delay(5);
  }
}