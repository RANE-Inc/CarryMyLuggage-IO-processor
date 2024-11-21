#include "estop.h"


EStopHandler* EStopHandler::instance = nullptr;

EStopHandler::EStopHandler() : triggered_(false) {
    pinMode(LED_BUILTIN, OUTPUT);
}

EStopHandler* EStopHandler::getInstance() {
     if(instance == nullptr) instance = new EStopHandler();

    return instance;
}

void EStopHandler::trigger_stop() {
    digitalWrite(LED_BUILTIN, HIGH);
    triggered_ = true;
}

bool EStopHandler::triggered() {
    return triggered_;
}


EStop::EStop(pin_size_t pin) : pin_(pin) {}

bool EStop::begin() {
    pinMode(pin_, INPUT_PULLDOWN);

    // TODO: debounce
    attachInterrupt(pin_, EStop::interruptRoutine, FALLING);

    return true;
}

void EStop::interruptRoutine() {
    EStopHandler::getInstance()->trigger_stop();
}
