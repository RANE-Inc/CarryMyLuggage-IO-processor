#include "wheel.h"

bool Wheel::begin() {
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(brake_pin, OUTPUT);

    pwmInstance = new RP2040_PWM(pwm_pin, freq, (uint32_t)0);

    if(pwmInstance) {
        digitalWrite(dir_pin, LOW);
        digitalWrite(brake_pin, LOW);
        pwmInstance->setPWM();

        return true;
    }
    
    return false;
}

void Wheel::update(uint32_t dutyCycle, bool reverse) {
    if(/*E-stop not triggered*/true) update_unsafe(dutyCycle, reverse);
    else update_unsafe(0);
}

void Wheel::update_unsafe(uint32_t dutyCycle, bool reverse) {
    if(dutyCycle == 0) {
        pwmInstance->setPWM_Int(pwm_pin, freq, 0);
        digitalWrite(dir_pin, reverse ? HIGH : LOW);
        
        delay(1000); // TODO: replace with non-blocking timer
        digitalWrite(brake_pin, LOW);
    }
    else {
        digitalWrite(brake_pin, HIGH);
        
        pwmInstance->setPWM_Int(pwm_pin, freq, dutyCycle);
        digitalWrite(dir_pin, reverse ? HIGH : LOW);
    }
}


