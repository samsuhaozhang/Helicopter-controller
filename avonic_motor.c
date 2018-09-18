//****************************************************************
//
// avonic_mototr.c - this is the program to output PWM on the main motor and tail motor
// M0PWM7 (J4-05, PC5) is used for the main motor
// M1PWM5 (J3-10, PF1) is used for the tail motor
//
// Author: Sam Zhang

// adapted from P.J. Bones UCECE (pwm 100.c)
// ***************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "avonic_motor.h"
// *******************************************************
// Globals to module
// *******************************************************
static uint32_t alt_out = 0;
static uint32_t yaw_out = 0;
static bool EnableMotors = false;
//********************************************************

//*****************************************************************************
// EnableMotor: Allows main and tail PWM signal to control main and tail motors.
//*****************************************************************************
void
EnableMotor (void)
{
    //main motor PWM signal
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    //tail motor PWM signal
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
    EnableMotors = true;
}
//*****************************************************************************
// disableMotor: Blocks main and tail PWM signal to the main and tail motors.
//*****************************************************************************
disableMotor (void)
{
    //main motor PWM signal
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    //tail motor PWM signal
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
    EnableMotors = false;
}
//*****************************************************************************
// ReturnMainDutyCycle: If EnableMotors true then returns current duty cycle of
// the main motor
//*****************************************************************************
uint32_t ReturnMainDutyCycle (void)
{
    if (EnableMotors) {
        return alt_out;
    } else {
        return 0;
    }
}
//*****************************************************************************
// ReturnTailDutyCycle: If EnableMotors true then returns current duty cycle of
// the tail motor
//*****************************************************************************
uint32_t ReturnTailDutyCycle (void)
{
    if (EnableMotors) {
        return yaw_out;
    } else {
        return 0;
    }
}
//*****************************************************************************
// set_main_PWM: set up the PWM signal for main motor
//*****************************************************************************
void set_main_PWM(uint32_t alt_duty)
{
    //if PWM_DUTYCYCLE_MIN_FLYING bigger then alt_duty return
    //PWM_DUTYCYCLE_MIN_FLYING else return alt_duty (prevents b going above PWM limit)
    alt_duty = MAX(PWM_DUTYCYCLE_MIN_FLYING, alt_duty);

    //if MAX_PWM bigger then alt_duty return alt_duty, else return
    //MAX_PWM (prevents alt_duty going below PWM limit)
    alt_duty = MIN(MAX_PWM, alt_duty);
    alt_out = alt_duty;

    // Calculate the PWM period corresponding to PWM_RATE_HZ.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * alt_duty / 100);
}
//*****************************************************************************
// set_tail_PWM: set up the PWM signal for tail motor
//*****************************************************************************
void set_tail_PWM(uint32_t yaw_duty)
{
    //if MIN_PWM bigger then yaw_duty return MIN_PWM else return
    //yaw_duty (prevents b going above PWM limit)
    yaw_duty = MAX(MIN_PWM, yaw_duty);

    //if MAX_PWM bigger then yaw_duty return yaw_duty, else return
    //MAX_PWM (prevents yaw_duty going below PWM limit)
    yaw_duty = MIN(MAX_PWM, yaw_duty);
    yaw_out = yaw_duty;

    // Calculate the PWM period corresponding to PWM_RATE_HZ.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_HZ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * yaw_duty / 100);
}
//*****************************************************************************
// initMotors: Initialise the variables associated with the set of PWM
//*****************************************************************************
void initMotors()
{
    //---SET UP FOR MAIN MOTOR------
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    set_main_PWM(MAIN_PWM_DUTY_CYCLE);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);

    //return main motor duty cycle;
    ReturnMainDutyCycle ();

    //---SET UP FOR TAIL MOTOR--------
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    set_tail_PWM(TAIL_PWM_DUTY_CYCLE);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
    //return_main_PWM();

    //return tail motor duty cycle;
    ReturnTailDutyCycle ();
}


