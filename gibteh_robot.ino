//
// прошивка робота для  
// мастер-класса по гибридным технологиям
// гибтехбфу.рф
//

/*
железо робота:

Bluetooth Bee DFRobot v2
    http://www.robototehnika.ru/e-store/catalog/248/1058/

Baud rate default: 9600
Pair: 1234

IMU
IMU (Inertial measurement unit - Инерционное измерительное устройство) модуль - позволяет определить положение в пространстве.
В составе :
3х осевой цифровой гироскоп ITG3205
3х осевой цифровой акселерометр ADXL345
3х осевой цифровой магнитометр HMC5883L
http://robocraft.ru/shop/index.php?route=product/product&path=45&product_id=265

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

Протокол взаимодействия ПК и робота по Bluetooth-каналу

от робота:
2 байта - заголовок 0xAAAA 
2 байта - последний посланный от ПК угол сервомашинки1
2 байта - последний посланный от ПК угол сервомашинки2
2 байта - угол наклона
1 байт - состояние датчика касания
2 байт - показания дальномера

от ПК:
2 байта - заголовок 0xAAAA 
2 байта - угол сервомашинки1
2 байта - угол сервомашинки2

двухбайтовые значения в формате little-endian (LSB, MSB)



используемые библиотеки:

http://playground.arduino.cc/Code/NewPing
    https://bitbucket.org/teckel12/arduino-new-ping/downloads/NewPing_v1.8.zip

Файлы проекта Razor AHRS ( v1.4.1 - Arduino+Processing+Matlab)
https://dev.qu.tu-berlin.de/login?back_url=https%3A%2F%2Fdev.qu.tu-berlin.de%2Fprojects%2Fsf-razor-9dof-ahrs%2Fwiki%2FTutorial
http://robocraft.ru/files/sensors/IMU/Razor%20AHRS%20Firmware%20and%20Test%20Sketch%20v1.4.1.zip

В секции "HARDWARE OPTIONS" раскомментировать сторку
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

http://robocraft.ru/files/sensors/IMU/IMU.ino
Пример скетча (минималистичная версия на базе Razor AHRS - убрана вся фильтрация, математика, совместимость с другими модулями и т.п.  - просто ROW с датчиков в Serial)

*/

#include <stdint.h>
#include "definitions.h"
#include "utils.h"
#include <Servo.h>
#include <Wire.h>
#include <NewPing.h> //#include <Ultrasonic.h>
#include "imu.h"

NewPing ultrasonic(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_DISTANCE_CM);

unsigned long imu_time = 0;
unsigned long blink_time = 0;
byte blink_val = 0;

#pragma pack(push, 1)
typedef struct RobotData {
    uint16_t header;
    uint16_t servo1_pos;
    uint16_t servo2_pos;
    uint16_t angle;
    uint8_t end_sensor;
    uint16_t distance;
} RobotData;

typedef struct PCData {
    uint16_t header;
    uint16_t servo1_pos;
    uint16_t servo2_pos;
} PCData;
#pragma pack(pop)

RobotData robot_data;
PCData pc_command;

int8_t buffer[32] = {0};
size_t buffer_size = 0;
int mode_ = MODE_PCKT_START1;

typedef struct RobotLeg {
    Servo servo[SERVO_COUNT];
    int pos[SERVO_COUNT];

    void init() {
        servo[0].attach(SERVO1_PIN);
        servo[1].attach(SERVO2_PIN);

        pinMode(END_SENSOR_PIN, INPUT);
    }

    void move(int16_t servo1_pos, int16_t servo2_pos) {
        pos[0] = servo1_pos;
        pos[1] = servo2_pos;

        for(int i=0; i<SERVO_COUNT; i++) {
            servo[i].write(pos[i]);
        }
        delay(SERVO_MOVEMENT_DELAY_MS);
    }
} RobotLeg;

RobotLeg robot_leg;

void setup()
{
    Serial.begin(SERIAL_SPEED);
    pinMode(LED_PIN, OUTPUT);

    robot_leg.init();

    init_imu();

    delay(1000);
    robot_leg.move(SERVO1_DEFAULT_POS, SERVO2_DEFAULT_POS);

#if SHOW_DEBUG_INFO
    Serial.print(F("Free RAM: "));
    Serial.println(free_Ram());
#endif
}

void loop()
{
    process_serial_commands();

    //if(is_Time(imu_time, IMU_INTERVAL)) {
    if(get_imu_angles()) {
        update_angle();
    }

    if(is_Time(blink_time, 1000)) {
        digitalWrite(LED_PIN, !blink_val);
        blink_val = !blink_val;
    }
}

void process_serial_commands()
{
    if(Serial.available()) {
        int data = Serial.read();

        if(data < 0) {
            return;
        }

        buffer[buffer_size++] = data;

        if( mode_ == MODE_PCKT_START1 ) {
            if(data == PCKT_START1) {
                mode_++;
            }
            else {
                buffer_size = 0;
            }
        }
        else if( mode_ == MODE_PCKT_START2 ) {
            if(data == PCKT_START2) {
                mode_++;
            }
            else {
                mode_ = MODE_PCKT_START1;
                buffer_size = 0;
            }
        }

        if(buffer_size == sizeof(pc_command)) {
            mode_ = MODE_PCKT_START1;
            buffer_size = 0;

            memcpy(&pc_command, buffer, sizeof(pc_command));
            make_command();
        }
    }
}

void make_command()
{
    robot_leg.move(pc_command.servo1_pos, pc_command.servo2_pos);

    send_robot_data();
}

void read_end_sensor()
{
    robot_data.end_sensor = digitalRead(END_SENSOR_PIN);

#if SHOW_DEBUG_INFO
    Serial.print(F("End sensor: "));
    Serial.println(robot_data.end_sensor);
#endif
}

void update_angle()
{
    robot_data.angle = (uint16_t)(TO_DEG(roll) + 180);
}

void read_ultrasonic()
{
    // get distance
    unsigned long cm = ultrasonic.ping_cm();
    robot_data.distance = (uint16_t)cm;

    if ((robot_data.distance < 1) || (robot_data.distance > ULTRASONIC_MAX_DISTANCE_CM)) {
        robot_data.distance = ULTRASONIC_MAX_DISTANCE_CM;
    }

#if SHOW_DEBUG_INFO
    Serial.print(F("US distance: "));
    Serial.print(cm);
    Serial.print(F(" : "));
    Serial.println(robot_data.distance);
#endif
}

void send_robot_data()
{
    robot_data.header = (PCKT_START1 << 8 ) | PCKT_START2;
    robot_data.servo1_pos = robot_leg.pos[0];
    robot_data.servo2_pos = robot_leg.pos[1];

    read_end_sensor();
    read_ultrasonic();

    // send packet
    send_buf((const uint8_t *)&robot_data, sizeof(robot_data));
}

void send_buf(const uint8_t *buf, const size_t buf_size)
{
    for(size_t i=0; i<buf_size; i++) {
        Serial.write(buf[i]);
    }

    Serial.flush(); // Waits for the transmission of outgoing serial data to complete.
}
