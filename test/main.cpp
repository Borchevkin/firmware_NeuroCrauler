//
// test robot-PC protocol
//

/*
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

*/

#define WIN32 1

// prefix for data packet
#define PCKT_START1      0xAA
#define PCKT_START2      0xAA

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include "serial.h"

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

// wait key
int waitKey(int msec)
{
    int res = 0;

#if defined(WIN32)

    // get handle to stdin used by console functions
    HANDLE hStdin =  GetStdHandle(STD_INPUT_HANDLE);

    // disable mouse events
    DWORD dwMode;
    GetConsoleMode(hStdin, &dwMode);
    SetConsoleMode(hStdin, dwMode & ~ENABLE_MOUSE_INPUT);
    FlushConsoleInputBuffer(hStdin);

    const int MaxEvents = 16;
    DWORD dwNumRead = 0;
    INPUT_RECORD buf[MaxEvents];

    // wait event
    if( WaitForSingleObject(hStdin, msec) == WAIT_OBJECT_0 ){
        // got event - reading code
        ReadConsoleInput(hStdin, buf, MaxEvents, &dwNumRead);
        // process each event
        for(DWORD i = 0; i < dwNumRead; i++){
            // if keyboard event
            if( buf[i].EventType == KEY_EVENT){
                KEY_EVENT_RECORD* pKey = (KEY_EVENT_RECORD*)&buf[i].Event.KeyEvent;
                // if key down event
                if( pKey->bKeyDown == TRUE){
                    res = pKey->wVirtualKeyCode;
                }
            }
        }
    }

#elif defined(LINUX)
    fd_set rfds;
    struct timeval tv;
    int retval;

    int nfds = STDIN_FILENO+1;

    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    tv.tv_sec = msec / 1000;
    tv.tv_usec = (msec % 1000) * 1000;

    if( tv.tv_usec > 1000000 )
    {
        tv.tv_sec++;
        tv.tv_usec -= 1000000;
    }

    retval = select(nfds, &rfds, NULL, NULL, msec!=0 ? &tv : 0);
    if (retval>0)
    {
        if(FD_ISSET(STDIN_FILENO, &rfds))
        {
            retval = read(STDIN_FILENO, &res, 1);
            if(retval<1)
                res = 0;
        }
    }
#endif //#if defined(WIN32)
    return res;
}


int main(int argc, char* argv[])
{
    printf("[i] Start...\n");

#if defined(WIN32)
    char _port[]="COM25";
#endif
    int _rate = 9600;

    char* port = _port;
    int rate = _rate;

    if(argc >= 3) {
        port = argv[1];
        rate = atoi(argv[2]);
    }
    else if(argc > 1) {
        port = argv[1];
    }
    else if(argc <= 1) {
        printf("Usage: \n");
        printf("program <port name> <baud rate> \n\n");
    }

    printf("[i] port: %s\n", port);
    printf("[i] rate: %d\n", rate);

    Serial serial;
    if( serial.open(port, rate) ) {
        return -1;
    }

    int res = 0;
    uint8_t buff[4096] = {0};
    size_t buff_size = 0;

    RobotData robot_data;
    PCData pc_command;
    pc_command.header = (PCKT_START1 << 8 ) | PCKT_START2;
    pc_command.servo1_pos = 45;
    pc_command.servo2_pos = 45;

    while( 1 ) {

        int key = waitKey(30);
        if(key != 0 ) printf( "[i] Key: %c (%d)\n", key ,key );
        if(key == 27) { //ESC
            break;
        }
        else if(key == 32) { // SPACE
            printf("45\n");
            pc_command.servo1_pos = 45;
            pc_command.servo2_pos = 45;

        }
        else if(key == 'w' || key == 'W') {
            printf("W\n");
            pc_command.servo1_pos += 2;
        }
        else if(key == 's' || key == 'S') {
            printf("S\n");
            pc_command.servo1_pos -= 2;
        }
        else if(key == 'a' || key == 'A') {
            printf("A\n");
            pc_command.servo2_pos += 2;
        }
        else if(key == 'd' || key == 'D') {
            printf("D\n");
            pc_command.servo2_pos -= 2;
        }

#if 1
        res = serial.write( (uint8_t *)&pc_command, sizeof(pc_command) );
        printf("[i] write: (%d)\n", res);
#endif

        if( res = serial.waitInput(500) ) {
            //printf("[i] waitInput: %d\n", res);
            if( (res = serial.available()) > 0 ) {
                printf("[i] available: %d\n", res);
                if( (res = serial.read(buff+buff_size, res)) > 0 ) {
                    buff_size += res;

                    // print data
                    printf("[i] read data(%d): \n", res);
#if 1
                    for(int i=0; i<buff_size; i++) {
                        printf("%02X ", buff[i]);
                        if(i>0 && (i+1)%16 == 0) {
                            printf("\t");
                            for(int j=i-15; j<=i; j++) {
                                printf("%c", buff[j]);
                            }
                            printf("\n");
                        }
                    }
                    printf("\n");
                    res = 0;
                    //buff.size = 0;
#endif

                    if( buff_size >= sizeof(RobotData) &&
                        (buff[0] == PCKT_START1 && buff[1] == PCKT_START1) ) {
                        memcpy(&robot_data, buff, sizeof(robot_data));
                        buff_size = 0;

                        printf("[i] servo1 pos: %d\n", robot_data.servo1_pos);
                        printf("[i] servo2 pos: %d\n", robot_data.servo2_pos);
                        printf("[i] angle: %d\n", robot_data.angle);
                        printf("[i] end: %d\n", robot_data.end_sensor);
                        printf("[i] distance: %d\n", robot_data.distance);
                    }
                    else {
                        printf("[!] no packet detected: %d\n", buff_size);
                    }
                }
                else {
                    printf("[!] too much data: %d (%d)\n", res, buff_size);
                    buff_size = 0;
                }
            }
            else {
                printf("[!] no available: %d\n", res);
            }
        }
        else {
            printf("[!] waitInput timeout: %d\n", res);
        }

#if defined(WIN32)
        Sleep(200);
#endif //#if defined(WIN32)
    }

    serial.close();

    printf("[i] End.\n");

    return 0;
}

