#include <Arduino.h>
#include <RP2040_PWM.h>
#include <AS5600.h>


class Wheel {
private:
  const pin_size_t pwm_pin_;
  const pin_size_t dir_pin_;
  const pin_size_t brake_pin_;
  const pin_size_t sda_pin_;
  const pin_size_t scl_pin_;
  RP2040_PWM* pwm_;
  AS5600* encoder_;
  int last_motor_update_; // in us
  int32_t current_command_;
  int32_t last_command_;
  int last_read_time_; // in us
  int32_t last_position_; // in counts
  float last_velocity_; // in counts/s

  void update_unsafe(int32_t level);

public:
  Wheel(pin_size_t pwm_pin, pin_size_t dir_pin, pin_size_t brake_pin, pin_size_t sda_pin, pin_size_t scl_pin);

  bool begin();
  void update(int32_t level);
  void update();
  void read_encoder();
  int32_t get_position();
  float get_velocity();
};
