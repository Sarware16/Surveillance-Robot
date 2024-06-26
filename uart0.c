// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include <stdlib.h>
#include <string.h>
#include "movement.h"
#include "navigate.h"

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
int temp;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud (assuming fcyc = 40 MHz), 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 130;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 13;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo, masking off the flags
}

void getsUart0(USER_DATA *data)
{
    int count = 0;
    while(true)
    {
        char c = getcUart0();

        if((c==8||c==127)&&(count>0))
        {
            count--;
        }
        else if(c==13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if(c>=32)
        {
            data->buffer[count] = c;
            count++;
        }
        if(count==MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
    }
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

void parseFields(USER_DATA *data)
{
    int i;
    data->fieldCount = 0;
    for(i=0; i<MAX_CHARS; i++)
    {
        if(data->buffer[i]=='\0')
        {
            return;
        }
        if(i!=0)
        {
            if(((data->buffer[i]>=65)&&(data->buffer[i]<=90))||((data->buffer[i]>=97)&&(data->buffer[i]<=122)))
            {
                if(data->buffer[i-1] == '\0')
                {
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldType[data->fieldCount] = 'a';
                    data->fieldCount += 1;
                }

            }
            else if((data->buffer[i]>=48)&&(data->buffer[i]<=57))
            {
                if(data->buffer[i-1] == '\0')
                {
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldType[data->fieldCount] = 'n';
                    data->fieldCount += 1;
                }
            }
            else
            {
                data->buffer[i] = '\0';
            }
        }
        else
        {
            if(((data->buffer[i]>=65)&&(data->buffer[i]<=90))||((data->buffer[i]>=97)&&(data->buffer[i]<=122)))
            {
                data->fieldPosition[data->fieldCount] = i;
                data->fieldType[data->fieldCount] = 'a';
                data->fieldCount += 1;
            }
            else if((data->buffer[i]>=48)&&(data->buffer[i]<=57))
            {
                data->fieldPosition[data->fieldCount] = i;
                data->fieldType[data->fieldCount] = 'n';
                data->fieldCount += 1;
            }
            else
            {
                data->buffer[i] = '\0';
            }
        }
    }
    return;
}
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{

    if(fieldNumber<5)
    {
        if(data->fieldType[fieldNumber]=='a')
        {
            return &data->buffer[data->fieldPosition[fieldNumber]];
        }
    }
    return 0;

}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{

    if(fieldNumber<5)
    {
        if(data->fieldType[fieldNumber]=='n')
        {
            return atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
        }
    }
    return 0;

}
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int test = strcmp(strCommand, getFieldString(data,0));

    if(test == 0)
    {
        if(data->fieldCount >= minArguments)
        {
            return true;
        }
    }
    return false;
}
void uartcmd(USER_DATA * data)
{

    getsUart0(data);
    parseFields(data);

    if(isCommand(data, "forward", 1))
    {
        if(isCommand(data, "forward", 3))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            forward(temp,getFieldInteger(data, 2));
        }
        else if(isCommand(data, "forward", 2))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            forward(temp,0);
        }
        else
        {
            forward(1023, 0);
        }
    }
    else if(isCommand(data, "reverse", 1))
    {
        if(isCommand(data, "reverse", 3))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            reverse(temp,getFieldInteger(data, 2));
        }
        else if(isCommand(data, "reverse", 2))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            reverse(temp,0);
        }
        else
        {
            reverse(1023,0);
        }
    }
    else if(isCommand(data, "ccw", 1))
    {
        if(isCommand(data, "ccw", 3))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            ccw(temp,getFieldInteger(data, 2));
        }
        else if(isCommand(data, "ccw", 2))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            ccw(temp,0);
        }
        else
        {
            ccw(1023,0);
        }
    }
    else if(isCommand(data, "cw", 1))
    {
        if(isCommand(data, "cw", 3))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            cw(temp,getFieldInteger(data, 2));
        }
        else if(isCommand(data, "cw", 2))
        {
            temp = speedToPWMLoad(getFieldInteger(data, 1));
            cw(temp,0);
        }
        else
        {
            cw(1023,0);
        }
    }
    else if(isCommand(data, "stop", 1))
    {
        stop();
    }
    else if(isCommand(data, "navigate", 1))
    {
        DATA = 16;
    }
    else
    {
        putsUart0("Error: Invalid Command!\n");
    }
}





























