#include "wheel.h"
Wheel::Wheel(pin_size_t pwm_pin, pin_size_t dir_pin, pin_size_t brake_pin) : 
    pwm_pin_(pwm_pin),
    dir_pin_(dir_pin),
    brake_pin_(brake_pin),
    pwm_(nullptr) {}

bool Wheel::begin() {
    pinMode(dir_pin_, OUTPUT);
    pinMode(brake_pin_, OUTPUT);

    digitalWrite(dir_pin_, LOW);
    digitalWrite(brake_pin_, LOW);

    pwm_ = new RP2040_PWM(pwm_pin_, 16.0*1000, (uint32_t)0);
    
    uint16_t level = 0; 
    pwm_->setPWM_manual(pwm_pin_, 8312, 1, level); // FIXME: find a way to hardcode 0 constant

        return true;
    }
    
    return false;
}

void Wheel::update(int32_t command) {
    if(/* E-Stop triggered */ false) update_unsafe(0);
    else {

        update_unsafe(current_command_);
    }
}

void Wheel::update_unsafe(int32_t command) {
    uint16_t level = (uint16_t)(command < 0 ? command*(-1) : command);

    if(level == 0) {
        digitalWrite(dir_pin_, LOW);
        pwm_->setPWM_manual(pwm_pin_, level);
        
        // delay(1000); // TODO: replace with non-blocking timer
        digitalWrite(brake_pin_, LOW);
    }
    else {
        digitalWrite(brake_pin_, HIGH);
        
        digitalWrite(dir_pin_, command < 0 ? HIGH : LOW);
        pwm_->setPWM_manual(pwm_pin_, level);
    }
}



