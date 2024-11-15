#include "wheel.h"
#include "estop.h"

constexpr uint32_t sda_valid[2] = { __bitset({0, 4, 8, 12, 16, 20, 24, 28}) /* I2C0 */,
                                __bitset({2, 6, 10, 14, 18, 22, 26})  /* I2C1 */ };
constexpr uint32_t scl_valid[2] = { __bitset({1, 5, 9, 13, 17, 21, 25, 29}) /* I2C0 */,
                                __bitset({3, 7, 11, 15, 19, 23, 27})  /* I2C1 */ };

Wheel::Wheel(pin_size_t pwm_pin, pin_size_t dir_pin, pin_size_t brake_pin, pin_size_t sda_pin, pin_size_t scl_pin) : 
    pwm_pin_(pwm_pin),
    dir_pin_(dir_pin),
    brake_pin_(brake_pin),
    sda_pin_(sda_pin),
    scl_pin_(scl_pin),
    pwm_(nullptr), 
    encoder_(nullptr),
    last_read_time_(0),
    last_position_(0),
    last_velocity_(0) {}
    

bool Wheel::begin() {
    pinMode(dir_pin_, OUTPUT);
    pinMode(brake_pin_, OUTPUT);

    digitalWrite(dir_pin_, LOW);
    digitalWrite(brake_pin_, LOW);

    pwm_ = new RP2040_PWM(pwm_pin_, 16.0*1000, (uint32_t)0);
    
    uint16_t level = 0; 
    pwm_->setPWM_manual(pwm_pin_, 8312, 1, level); // FIXME: find a way to hardcode 0 constant
    
    if(((1<<sda_pin_) & sda_valid[0]) && ((1<<scl_pin_) & scl_valid[0])) {
        Wire.setSDA(sda_pin_);
        Wire.setSCL(scl_pin_);
        Wire.begin();

        encoder_ = new AS5600(&Wire);
    }
    else if(((1<<sda_pin_) & sda_valid[1]) && ((1<<scl_pin_) & scl_valid[1])) {
        Wire1.setSDA(sda_pin_);
        Wire1.setSCL(scl_pin_);
        Wire1.begin();

        encoder_ = new AS5600(&Wire1);
    }
    else return false;

    encoder_->begin();
    encoder_->setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    encoder_->resetCumulativePosition(0);

        return true;
    }
    
    return false;
}

void Wheel::update(int32_t command) {
    if(EStopHandler::getInstance()->triggered()) update_unsafe(0);
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

void Wheel::read_encoder() {
    uint32_t now  = micros();
    int32_t current_position = encoder_->getCumulativePosition(true);
    
    last_velocity_ = (current_position-last_position_) / (now - last_read_time_) * 1e6;
    last_position_ = current_position;
    last_read_time_ = now;
}

int32_t Wheel::get_position() {
    return last_position_;
}

float Wheel::get_velocity() {
    return last_velocity_;
}


