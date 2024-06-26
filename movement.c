// Robot Movement Library
// Anaf Mahbub

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Right Motor 1 on M0PWM6 (PC4), M0PWM3a
// Right Motor 2 on M0PWM7 (PC5), M0PWM3b
// Right Collector on (PC7)
// Left Motor 1 on M1PWM0 (PD0), M1PWM0a
// Left Motor 2 on M1PWM1 (PD1), M1PWM0b
// Left Collector on (PD6)
// Motor Sleep Button on (PE1)
// IR Detector on (PD2)
// Motion Sensor on (PE3)


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

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

//PortB masks
#define ECHO_MASK 4
#define TRIG_MASK 64
// PortC masks
#define RIGHT_MOTOR1 16
#define RIGHT_MOTOR2 32
#define LEFT_COLLECTOR 128
// PortD masks
#define LEFT_MOTOR1 1
#define LEFT_MOTOR2 2
#define RIGHT_COLLECTOR 64
#define IR_DETECTOR 4
// PortE masks
#define SLEEP_MASK 2
#define PIR_MASK 8
// BitBand Aliases
#define ECHO_PIN (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))
#define TRIG_PIN (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define SLEEP_BUTTON (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define PIR_SENSOR (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
// PortF masks
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
#define INPUT_PIN_0_MASK 1
#define T 22500.0
#define TICKS_PER_MS 40000

uint32_t time[50] = { 0 };
uint8_t count;
uint8_t code = 0;
uint8_t ADDR;
uint8_t ADDRNOT;

uint8_t DATANOT;
uint32_t newcode = 0;
char string[8] = { '0' };
int buffer;
float limit;
bool targetreached = false;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize RGB and Robot Movement
void initMovement()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3;
    _delay_cycles(3);

    //Configure ultra-sonic sensor:
    GPIO_PORTB_DIR_R |= TRIG_MASK;
    GPIO_PORTB_DIR_R &= ~ECHO_MASK;
    GPIO_PORTB_DEN_R |= TRIG_MASK | ECHO_MASK;


    // Configure Right Motor and Right Collector:
    GPIO_PORTC_DEN_R |= RIGHT_MOTOR2 | RIGHT_MOTOR1 | LEFT_COLLECTOR;
    GPIO_PORTC_DIR_R &= ~LEFT_COLLECTOR;
    GPIO_PORTC_PDR_R &= ~LEFT_COLLECTOR;
    GPIO_PORTC_AFSEL_R |= RIGHT_MOTOR2 | RIGHT_MOTOR1;
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_M0PWM6 | GPIO_PCTL_PC5_M0PWM7;

    // Configure Left Motor and Left Collector and IR Detector:
    GPIO_PORTD_DEN_R |= LEFT_MOTOR2 | LEFT_MOTOR1 | RIGHT_COLLECTOR | IR_DETECTOR;
    GPIO_PORTD_DIR_R &= ~(RIGHT_COLLECTOR | IR_DETECTOR);
    GPIO_PORTD_PDR_R |= RIGHT_COLLECTOR;
    GPIO_PORTD_AFSEL_R |= LEFT_MOTOR2 | LEFT_MOTOR1;
    GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M);
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_M1PWM0 | GPIO_PCTL_PD1_M1PWM1;

    //Configure falling edge interrupts on collector inputs
    GPIO_PORTC_IS_R &= ~LEFT_COLLECTOR;
    GPIO_PORTC_IBE_R &= ~LEFT_COLLECTOR;
    GPIO_PORTC_IEV_R &= ~LEFT_COLLECTOR;
    GPIO_PORTC_ICR_R = LEFT_COLLECTOR;
    GPIO_PORTC_IM_R |= LEFT_COLLECTOR;
    NVIC_EN0_R = 1 << (INT_GPIOC-16);

    GPIO_PORTD_IS_R &= ~(RIGHT_COLLECTOR | IR_DETECTOR);
    GPIO_PORTD_IBE_R &= ~(RIGHT_COLLECTOR | IR_DETECTOR);
    GPIO_PORTD_IEV_R &= ~(RIGHT_COLLECTOR | IR_DETECTOR);
    GPIO_PORTD_ICR_R = RIGHT_COLLECTOR | IR_DETECTOR;
    GPIO_PORTD_IM_R |= RIGHT_COLLECTOR | IR_DETECTOR;
    NVIC_EN0_R = 1 << (INT_GPIOD-16);

    // Configure SLEEP on H-Bridge and PIR Sensor:
    GPIO_PORTE_DIR_R |= SLEEP_MASK;
    GPIO_PORTE_DIR_R &= ~PIR_MASK;
    GPIO_PORTE_DEN_R |= SLEEP_MASK | PIR_MASK;
    SLEEP_BUTTON = 1;


    // Configure three LEDs
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    // Right Motor 1 on M0PWM6 (C4), M0PWM3a
    // Right Motor 2 on M0PWM7 (C5), M0PWM3b
    // Left Motor 1 on M1PWM0 (D0), M1PWM0a
    // Left Motor 2 on M1PWM1 (D1), M1PWM0b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3 (drives outs 6 and 7)
    PWM1_0_CTL_R = 0;                                // turn-off PWM1 generator 0 (drives outs 1 and 2)
                                                     // output 7 on PWM1, gen 3b, cmpb
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
                                                     // output 5 on PWM0, gen 3a, cmpa
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
                                                     // output 6 on PWM0, gen 3b, cmpb
    PWM1_0_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 0 on PWM1, gen 0a, cmpa
    PWM1_0_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 1 on PWM1, gen 0b, cmpb

    //Set load
    PWM0_3_LOAD_R = 1024;
    PWM1_0_LOAD_R = 1024;

    //RIGHT MOTOR                                    //Right Motor off
    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;

    //LEFT MOTOR                                     //Left Motor off
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = 0;

    //LEDS and Motors:
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 3
    PWM1_0_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 0
    PWM0_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN; //enable outputs
    PWM1_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN; // | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
    //Timer 1:
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one - shot
    TIMER1_TAILR_R = 1000000;                        // set load value to 1e6 for 40 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A)

    //Timer 2:
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one - shot
    TIMER2_TAILR_R = 1000000;                        // set load value to 1e6 for 40 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 39 (TIMER2A)

    //Timer 3:
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER3_TAILR_R = 0xFFFFFFFF;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER3_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER3_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
//    NVIC_EN1_R = 1 << (INT_TIMER3A-16-51);              // turn-on interrupt 37 (TIMER1A) in NVIC


    // Configure Wide Timer 3 as counter of external events on CCP0 pin
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_NEG;           // count positive edges
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                 // turn-off interrupts
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    NVIC_EN3_R |= 1 << (INT_WTIMER3A-16-100);         // turn-on interrupt 112 (WTIMER1A)




}

void LeftFallingEdgeIsr()
{
    GREEN_LED ^= 1;

    leftcount++;

    if((leftcount >= limit) && (limit != -1))
    {
        SLEEP_BUTTON = 0;
        targetreached = true;
    }


    GPIO_PORTC_IM_R &= ~LEFT_COLLECTOR;              // turn-off GPIO interrupt
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on one shot timer
    GPIO_PORTC_ICR_R = LEFT_COLLECTOR;              // clear interrupt
}
void RightFallingEdgeIsr()
{

    if(GPIO_PORTD_MIS_R & 4)
    {

        ADDR = 0;
        ADDRNOT = 0;
        DATA = 0;
        DATANOT = 0;


        if(WTIMER3_TAV_R >80*TICKS_PER_MS)
        {
            count = 0;
        }
        if(count == 0)
        {
            WTIMER3_TAV_R = 0;
        }
        time[count] = WTIMER3_TAV_R;

        if(count == 0)
        {
            count++;
        }
        else if(count == 1)
        {
            uint32_t t = time[1] - time[0];

            if((t >= 13*TICKS_PER_MS) && (t <= 14*TICKS_PER_MS))
                count++;
            else
                count = 0;
        }
        else if(count>1)
        {
            uint32_t data = time[count]-time[count-1];
            if((data>1.5*T && data<2.5*T) || (data>3.5*T && data<4.5*T))
                count++;
            else
                count = 0;
        }
        if (count == 34)
        {
            count = 0;
            int i;

            for(i = 1; i <= 34; i++)
            {
                uint32_t t = time[i+1]-time[i];

                if(t>(1.5*T) && t<(2.5*T))  //0
                    newcode |= 0 << (i-1);
                else if(t>(3.5*T) && t<(4.5*T))  //1
                    newcode |= 1 << (i-1);
            }

            ADDR = newcode >> 0;
            ADDRNOT = newcode >> 8;
            DATA = newcode >> 16;
            DATANOT = newcode >> 24;
            newcode = 0;
            waitMicrosecond(10000);

            remote();

        }


        GPIO_PORTD_ICR_R = IR_DETECTOR;                     // clear interrupt flag
        WTIMER3_ICR_R = TIMER_ICR_CAECINT;

    }
    if(GPIO_PORTD_MIS_R & 64)
    {
        BLUE_LED ^= 1;
        rightcount++;

        if((rightcount >= limit) && (limit != -1))
        {
            SLEEP_BUTTON = 0;
            targetreached = true;
        }

        GPIO_PORTD_IM_R &= ~(RIGHT_COLLECTOR);              // turn-off GPIO interrupt
        TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on one shot timer
        GPIO_PORTD_ICR_R = RIGHT_COLLECTOR;              // clear interrupt

    }
}
void LeftDebounceIsr()
{
    GPIO_PORTC_ICR_R = LEFT_COLLECTOR;
    GPIO_PORTC_IM_R |= LEFT_COLLECTOR;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;

}
void RightDebounceIsr()
{
    GPIO_PORTD_ICR_R = RIGHT_COLLECTOR;
    GPIO_PORTD_IM_R |= RIGHT_COLLECTOR;
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

void forward(int speed, int distance)
{
    leftcount = 0;
    rightcount = 0;

    // Wake from SLEEP
    SLEEP_BUTTON = 1;

    // Configure motor PWM based on speed
    PWM0_3_CMPA_R = 0;           // Assuming configuration for forward direction
    PWM0_3_CMPB_R = speed - 29;  // Speed adjustment for motor characteristics
    PWM1_0_CMPA_R = speed;       // Same here
    PWM1_0_CMPB_R = 0;           // Assuming configuration for forward direction

    if(distance != 0)
    {
        limit = ((distance*10) / 245) * 2;  // Calculate the necessary count for given distance

    }
    else
    {
        limit = -1;
    }
}

void reverse(int speed, int distance)
{
    leftcount = 0;
    rightcount = 0;

    //Wake from SLEEP
    SLEEP_BUTTON = 1;

    //RIGHT MOTOR
    PWM0_3_CMPA_R = speed - 34;
    PWM0_3_CMPB_R = 0;

    //LEFT MOTOR
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = speed;

    if(distance != 0)
    {
        limit = ((distance*10) / 245) * 2;  // Calculate the necessary count for given distance
    }
    else
    {
        limit = -1;
    }
}

void ccw(int speed, int angle)
{

    leftcount = 0;
    rightcount = 0;

    //Wake from SLEEP
    SLEEP_BUTTON = 1;

    //RIGHT MOTOR
    PWM0_3_CMPA_R = 0;
    PWM0_3_CMPB_R = speed - 35;

    //LEFT MOTOR
    PWM1_0_CMPA_R = 0;
    PWM1_0_CMPB_R = speed;

    if(angle != 0)
    {
        limit = ((2* angle) / 73) * 3;  // Calculate the necessary count for given distance
    }
    else
    {
        limit = -1;
    }
}

void cw(int speed, int angle)
{

    leftcount = 0;
    rightcount = 0;

    //Wake from SLEEP
    SLEEP_BUTTON = 1;

    //LEFT MOTOR
    PWM1_0_CMPA_R = speed;
    PWM1_0_CMPB_R = 0;

    //RIGHT MOTOR
    PWM0_3_CMPA_R = speed;
    PWM0_3_CMPB_R = 0;

    if(angle != 0)
    {
        limit = ((2* angle) / 73) * 3;  // Calculate the necessary count for given distance
    }
    else
    {
        limit = -1;
    }
}

void stop()
{
    leftcount = 0;
    rightcount = 0;

    SLEEP_BUTTON = 0;
}


void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
}
void sendPulse()
{
    TRIG_PIN = 1;
    waitMicrosecond(10);
    TRIG_PIN = 0;
}

uint32_t measure_mm()
{
    uint32_t start, end, duration, dist;

    sendPulse();
    while(!ECHO_PIN);
    start = TIMER3_TAR_R;
    while(ECHO_PIN);
    end = TIMER3_TAR_R;

    duration = start - end;
    dist = (duration * 0.34 * 0.025) / 2;

    return dist;
}
void remote()
{
    if(DATA == 64)
    {
        forward(1023, 0);
    }
    else if(DATA == 65)
        reverse(1023, 0);
    else if(DATA == 7)
        ccw(1023, 0);
    else if(DATA == 6)
        cw(1023, 0);
    else if(DATA == 68)
        stop();
//    else if(DATA == 16)
//    {
//       valid ^= 1;
//    }
    else
        waitMicrosecond(100);

}
bool motion_sense()
{
    if(PIR_SENSOR)
    {
        RED_LED = 1;
        return true;
    }
    else
    {
        RED_LED = 0;
        return false;
    }
}
int speedToPWMLoad(int speed){
    // Constants calculated from the linear relationship
    double m = (10000.0 - 3000.0) / (1023.0 - 750.0);
    double b = 10000.0 - m * 1023.0;

    // Calculate PWM load
    int pwmLoad = (int)((speed - b) / m + 0.5);  // Adding 0.5 for rounding to nearest integer

    // Ensuring PWM load is within the allowed range
    if (pwmLoad > 1023) pwmLoad = 1023;
    if (pwmLoad < 750) pwmLoad = 750;

    return pwmLoad;
}
void swapRows(int arr[][2], int row1, int row2, int colCount) {
    int i;
    for(i = 0; i < colCount; i++)
    {
        int temp = arr[row1][i];
        arr[row1][i] = arr[row2][i];
        arr[row2][i] = temp;
    }
}

void selectionSort2D(int arr[][2], int rowCount, int colCount) {
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < rowCount - 1; i++) {
        // Find the row with minimum element in the first column
        min_idx = i;
        for (j = i + 1; j < rowCount; j++)
            if (arr[j][0] < arr[min_idx][0])
                min_idx = j;

        // Swap the found minimum row with the current row
        swapRows(arr, min_idx, i, colCount);
    }
}


void navigate()
{
        int arr[4][2];

        int temp;
        int dist1;
        int dist2;
        int dist3;
        int reqDist;

        int a = 0;
        while(a < 4)
        {
            cw(1023, 90);
            waitMicrosecond(750000);
            dist1 = measure_mm();
            dist2 = measure_mm();
            dist3 = measure_mm();
            reqDist = (dist1 + dist2 + dist3) / 3;


            arr[a][0] = reqDist;
            arr[a][1] = a + 1;
            waitMicrosecond(250000);
            a++;
        }

        selectionSort2D(arr, 4, 2);
        int reqAngle = (arr[3][1]) * 90;
        reqDist = arr[3][0];
        temp = arr[3][0];

        cw(1023, reqAngle);
        waitMicrosecond(500000);
        if(motion_sense())
        {
            waitMicrosecond(3000000);
        }
        forward(1023, reqDist);
        while(temp >= 200)
        {
            temp = measure_mm();
        }
        stop();

}
void wallpingtest()
{
    int temp = 300;
    forward(1023, 0);
    while(temp >= 200)
    {
        temp = measure_mm();
    }
    stop();

}







