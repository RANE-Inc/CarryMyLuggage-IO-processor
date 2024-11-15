#include <Arduino.h>
#include <RP2040_PWM.h>

class Wheel {
private:
  const pin_size_t pwm_pin_;
  const pin_size_t dir_pin_;
  const pin_size_t brake_pin_;
  const pin_size_t sda_pin_;
  const pin_size_t scl_pin_;
  RP2040_PWM* pwm_;

  void update_unsafe(int32_t level);

public:
  Wheel(pin_size_t pwm_pin, pin_size_t dir_pin, pin_size_t brake_pin);

bool begin();
  void update(int32_t level);
};
