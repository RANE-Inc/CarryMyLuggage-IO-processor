#include <Arduino.h>
#include "wheel.h"

Wheel* wheel_L;
Wheel* wheel_R;

void setup() {
    Serial.begin(115200);

    wheel_L = new Wheel(3, 2, 6);
    wheel_R = new Wheel(4, 5, 7);

    wheel_L->begin();
    wheel_R->begin();
}

#define PWM_MIN 0
#define PWM_MAX 20000
#define PWM_STEP 100
#define PWM_DELAY 59
#define DIR_DELAY 1000

void loop() {
    delay(DIR_DELAY);
    for(int i = PWM_MIN; i <= PWM_MAX; i+=100) {
        wheel_R->update(i);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    for(int i = PWM_MAX; i >= PWM_MIN; i-=100) {
        wheel_R->update(i);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    delay(DIR_DELAY);
    for(int i = PWM_MIN; i <= PWM_MAX; i+=100) {
        wheel_R->update(i, true);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    for(int i = PWM_MAX; i >= PWM_MIN; i-=100) {
        wheel_R->update(i, true);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    delay(DIR_DELAY);
    for(int i = PWM_MIN; i <= PWM_MAX; i+=100) {
        wheel_L->update(i);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    for(int i = PWM_MAX; i >= PWM_MIN; i-=100) {
        wheel_L->update(i);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    delay(DIR_DELAY);
    for(int i = PWM_MIN; i <= PWM_MAX; i+=100) {
        wheel_L->update(i, true);
        Serial.println(i);
        delay(PWM_DELAY);
    }
    for(int i = PWM_MAX; i >= PWM_MIN; i-=100) {
        wheel_L->update(i, true);
        Serial.println(i);
        delay(PWM_DELAY);
    }
}
