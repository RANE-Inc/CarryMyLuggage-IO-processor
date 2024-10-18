#include <Arduino.h>
#include <RP2040_PWM.h>

class Wheel {
private:
uint32_t pwm_pin;
uint32_t dir_pin;
uint32_t brake_pin;
float freq;
RP2040_PWM* pwmInstance;

void update_unsafe(uint32_t dutyCycle, bool reverse = false);

public:
Wheel(uint32_t pwm_pin, uint32_t dir_pin, uint32_t brake_pin, float freq = 16000.0f) 
  : 
  pwm_pin(pwm_pin),
  dir_pin(dir_pin),
  brake_pin(brake_pin),
  freq(freq) {}

bool begin();
void update(uint32_t dutyCycle, bool reverse = false);
};
