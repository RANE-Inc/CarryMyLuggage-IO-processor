#include <Arduino.h>

class EStopHandler {
private:
    static EStopHandler* instance;
    bool triggered_;
    
    EStopHandler();
public:
    static EStopHandler* getInstance();
    
    EStopHandler(const EStopHandler& obj) = delete;
    void trigger_stop();
    bool triggered();

};

class EStop {
private:
    pin_size_t pin_;
public:
    EStop() = delete;
    EStop(pin_size_t pin);
    bool begin();
    static void interruptRoutine();
};
