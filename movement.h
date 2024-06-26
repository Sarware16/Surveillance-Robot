// Robot Movement Library
// Anaf mahbub

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// RED   on M1PWM5 (PF1), M1PWM2b
// BLUE  on M1PWM6 (PF2), M1PWM3a
// GREEN on M1PWM7 (PF3), M1PWM3b
// Right Motor 1 on M0PWM6 (PC4), M0PWM3a
// Right Motor 2 on M0PWM7 (PC5), M0PWM3b
// Right Collector on (PC7)
// Left Motor 1 on M1PWM0 (PD0), M1PWM0a
// Left Motor 2 on M1PWM1 (PD1), M1PWM0b
// Left Collector on (PD6)
// Motor Sleep Button on (PE1)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include <stdint.h>
#include <stdbool.h>

int leftcount;
int rightcount;
uint8_t DATA;
//static uint8_t valid = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initHw();
void initMovement();
void forward(int speed, int distance);
void reverse(int speed, int distance);
void ccw(int speed, int angle);
void cw(int speed, int angle);
void stop();
uint32_t measure_mm();
void remote();
bool motion_sense();
int speedToPWMLoad(int speed);
void navigate();
void wallpingtest();
#endif
