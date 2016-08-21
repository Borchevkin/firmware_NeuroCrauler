//
// вспомогательные функции
//
// http://robocraft.ru
//

#ifndef _UTILS_H_
#define _UTILS_H_

#include "definitions.h"

void debug_info(const char* str);
void debug_info(const __FlashStringHelper *ifsh);

// возвращает количество свободной памяти
// http://robocraft.ru/blog/arduino/531.html
int free_Ram();

//**********************************************************
// based on IsTime() function - David Fowler, AKA uCHobby, http://www.uchobby.com 01/21/2012
// http://www.uchobby.com/index.php/2012/01/21/replacing-delay-in-arduino-sketches-istime-to-the-rescue/
//

#define TIMECTL_MAXTICKS  4294967295L
#define TIMECTL_INIT      0

bool is_Time(unsigned long &timeMark, unsigned long timeInterval);

// считывание значения с простым устранением "дребезга контактов"
int debounce_digitalRead(uint8_t pin);

#endif //#ifndef _UTILS_H_
