#ifndef AVONIC_MOTOR_H_
#define AVONIC_MOTOR_H_

// *****************************************************
// avonic_mototr.h - this is the program to output PWM on the main motor and tail motor
//
// Author: Sam Zhang
// Last modified:   28.05.2018
// *****************************************************

#include <stdint.h>
#include <stdbool.h>

//*******************************************
//     PWM config details.
//*******************************************
#define MAIN_PWM_DUTY_CYCLE 0
#define TAIL_PWM_DUTY_CYCLE 0
#define MAX_PWM 95
#define MIN_PWM 5
#define PWM_DUTYCYCLE_MIN_FLYING 8
#define PWM_RATE_HZ  200
#define PWM_DIVIDER_CODE  SYSCTL_PWMDIV_2
#define PWM_DIVIDER  2
//*******************************************
//      Main PWM Hardware Details.
//*******************************************
//---Main Rotor PWM: M0PWM7,PC5, J4-05
#define PWM_MAIN_BASE       PWM0_BASE
#define PWM_MAIN_GEN        PWM_GEN_3   //covers outputs 6 and 7
#define PWM_MAIN_OUTNUM     PWM_OUT_7
#define PWM_MAIN_OUTBIT     PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM SYSCTL_PERIPH_PWM0 //module 0
#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

//*******************************************
//      Tail PWM Hardware Details.
//*******************************************
//---Tail Rotor PWM: M1PWM5,PF1, J3-10
#define PWM_TAIL_BASE       PWM1_BASE
#define PWM_TAIL_GEN        PWM_GEN_2   //covers outputs 6 and 7
#define PWM_TAIL_OUTNUM     PWM_OUT_5
#define PWM_TAIL_OUTBIT     PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM SYSCTL_PERIPH_PWM1 //module 1
#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1
//*******************************************
//      PWM limits.
//*******************************************
#define MAX(MIN_PWM,b) MIN_PWM>b?MIN_PWM:b      //if MIN_PWM bigger then b return MIN_PWM else return b (prevents b going above PWM limit)
#define MIN(MAX_PWM,b) MAX_PWM >b?b:MAX_PWM     //if MAX_PWM  bigger then b return b, else return MAX_PWM (prevents b going below PWM limit)

//*****************************************************************************
// ReturnMainDutyCycle: If EnableMotors true then returns current duty cycle of
// the main motor
//*****************************************************************************
uint32_t ReturnMainDutyCycle(void);

//*****************************************************************************
// ReturnTailDutyCycle: If EnableMotors true then returns current duty cycle of
// the tail motor
//*****************************************************************************
uint32_t ReturnTailDutyCycle(void);

//*****************************************************************************
// EnableMotor: Allows main and tail PWM signal to control main and tail motors.
//*****************************************************************************
void EnableMotor (void);

//*****************************************************************************
// disableMotor: Blocks main and tail PWM signal to the main and tail motors.
//*****************************************************************************
void disableMotor (void);

//*****************************************************************************
// set_main_PWM: set up the PWM signal for main motor
//*****************************************************************************
void set_main_PWM(uint32_t alt_duty);

//*****************************************************************************
// set_tail_PWM: set up the PWM signal for tail motor
//*****************************************************************************
void set_tail_PWM(uint32_t yaw_duty);

//*****************************************************************************
// initMotors: Initialise the variables associated with the set of PWM
//*****************************************************************************
void initMotors(void);

#endif /* AVONIC_MOTOR_H_ */
