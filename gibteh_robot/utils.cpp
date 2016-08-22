//
// вспомогательные функции
//
//

#include "utils.h"

void debug_info(const char *str)
{
#if SHOW_DEBUG_INFO
    Serial.println(str);
#endif
}

void debug_info(const __FlashStringHelper *ifsh)
{
#if SHOW_DEBUG_INFO
    Serial.println(ifsh);
#endif
}

int free_Ram ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

bool is_time(unsigned long &timeMark, unsigned long timeInterval)
{
    unsigned long timeCurrent;
    unsigned long timeElapsed;

    bool result = false;

    timeCurrent = millis();
    if( timeCurrent < timeMark ) { // Rollover detected
        // elapsed = all the ticks to overflow + all the ticks since overflow
        timeElapsed = (TIMECTL_MAXTICKS - timeMark) + timeCurrent;
    }
    else {
        timeElapsed = timeCurrent - timeMark;
    }

    if(timeElapsed >= timeInterval) {
        timeMark = timeCurrent;
        result = true;
    }

    return (result);
}

// считывание значения с простым устранением "дребезга контактов"
int debounce_digitalRead(uint8_t pin)
{
#if 1
    int cur_pinState = 0;
    uint8_t counter;
    uint8_t period = 50;

    uint8_t _counter = 0;
    while(_counter++ < 5) {
        counter = 0;
        for(uint8_t i=0; i<period; i++) {
            cur_pinState = digitalRead(pin);
            delayMicroseconds(1);
            if(digitalRead(pin) == cur_pinState) {
                ++counter;
            }
        }
        if(counter == period)
            break;
    }

    return cur_pinState;

#else
    return digitalRead(pin);
#endif
}
