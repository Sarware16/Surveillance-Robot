#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "movement.h"
#include "uart0.h"
#include "navigate.h"
#include "linked_list.h"
int d = 0;
int valid = 0;


int main(void)
{
    initHw();
    initMovement();
    initUart0();
    setUart0BaudRate(19200, 40e6);
    USER_DATA data;
    while (true)
    {

        if(kbhitUart0())
            uartcmd(&data);
        if(DATA == 16)
        {
            valid = 1;
        }
        if(valid)
        {
            navigate();
            valid = 0;
        }
        if(DATA == 26)
        {
            wallpingtest();
        }

        d = measure_mm();
        waitMicrosecond(10000);

        motion_sense();
        waitMicrosecond(100000);
    }

}
