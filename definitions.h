//
// Robot definitions
//

#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SHOW_DEBUG_INFO     0

#define SERIAL_SPEED    9600

// servo count of robot leg
#define SERVO_COUNT 2

// PIN CONNECTIONS
#define LED_PIN 13

// servo
#define SERVO1_PIN 11
#define SERVO2_PIN 10

// ulrasonic
#define ULTRASONIC_TRIG_PIN 8
#define ULTRASONIC_ECHO_PIN 9

// end stop
#define END_SENSOR_PIN 2

// max ultrasonic distance
#define ULTRASONIC_MAX_DISTANCE_CM 200

// delay anfter servo movement
#define SERVO_MOVEMENT_DELAY_MS 20

// initial servo positions
#define SERVO1_DEFAULT_POS 90
#define SERVO2_DEFAULT_POS 90

// prefix for data packet
#define PCKT_START1      0xAA
#define PCKT_START2      0xAA

enum {MODE_PCKT_START1=0, MODE_PCKT_START2, MODE_PCKT_DATA};

#endif //#ifndef _DEFINITIONS_H_
