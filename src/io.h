#include <avr/io.h>

class LED{
    private:
        uint8_t pin;
        uint8_t brightness; // 0-255
        bool state; // on/off
    
    public:
        LED(uint8_t pin);
        void on();
        void off();
        // Adjust brightness based on a value from 0 to 255
        void adjust_brightness(uint8_t brightness);
};