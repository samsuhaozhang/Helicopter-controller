//*****************************************************************************
// avonic_main.c - this is the main program that performs avionic control of the helicopter
//
// Author: Sam Zhang
//*****************************************************************************
// This program is used to run the.c
// The following .c modules and flies are required to run avonic_main.c
// buttons.c, circBufT.c, ustdlib.c, avonic_motor.c, avonic_uart.c and OrbitOLED folder
//
//*****************************************************************************
//Linked files specifically created for the helicopter controller
#include <avonic_uart.h>
#include <buttons.h>
#include "circBufT.h"
#include "avonic_motor.h"

// Linked files specifically created for programing the TIVA TM4C123G board and orbitbooster
#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stdlib.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "driverlib/timer.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "driverlib/pin_map.h"

/***********************************************************************************************
 * Defined Constants
 **********************************************************************************************/
#define BUF_SIZE 8              // Buffer size
#define SAMPLE_RATE_HZ 320      // Sampling rate

/***********************************************************************************************
 * Global Variables
 **********************************************************************************************/
// Global Variables for ADC conversion
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts

// Global Variables for quadrature decoding
static volatile bool Yaw_reference_found = false;
static volatile bool ground_channel_A = 0;  // set channel B value to be a LOW
static volatile bool ground_channel_B = 0;  // set channel A to be a HIGH
static volatile bool current_channel_A;     // reads pwm signal of channel A
static volatile bool current_channel_B;     // reads pwm signal of channel B
static int32_t Yaw = 0;                     // incremental yaw tracking

/***********************************************************************************************
 * Defined the states for the helicopter controller FINITE STATE MACHINE
 **********************************************************************************************/
typedef enum HelicopterState {
    TURN_ON_MODE = 0 ,
    CALABRATION_MODE = 1,
    FLYING_MODE = 2,
    LANDING_MODE = 3,
    RESET_MODE = 4
} HelicopterState_t;

static HelicopterState_t current_mode = TURN_ON_MODE;
static HelicopterState_t next_mode;

/***********************************************************************************************
 * Channel_IntHandler
 * This handler does the quadrature decoding to determine the direction of the yaw.
 * A pair of digital signal logic level from PIN j1-03 and PIN j1-04 are passed through several
 * conditional statements to determine the state change and thus determine the direction of yaw.
 * Ascending state change implies clockwise yaw, descending implies implies counter clockwise yaw.
 * After yaw is determined, pass pair of signal logic is updated and interrupt is cleared.
 **********************************************************************************************/
void
Channel_IntHandler(void)
{
    current_channel_A = GPIOPinRead(GPIO_PORTB_BASE, (GPIO_PIN_0));
    current_channel_B = GPIOPinRead(GPIO_PORTB_BASE, (GPIO_PIN_1));

    if (ground_channel_A == 0 && ground_channel_B == 0) {               // state 1
        if  (current_channel_A == 0 && current_channel_B == 1) {        // state 2
            Yaw++;      // clockwise
        }
        else if (current_channel_A == 1 && current_channel_B == 0) {    // state 4
            Yaw--;      // counter clockwise
        }
    }
    else if (ground_channel_A == 0 && ground_channel_B == 1) {          // state 2
        if  (current_channel_A == 1 && current_channel_B == 1) {        // state 3
            Yaw++;      // clockwise
        }
        else if  (current_channel_A == 0 && current_channel_B == 0) {   // state 1
            Yaw--;      // counter clockwise
        }
    }
    else if (ground_channel_A == 1 && ground_channel_B == 1) {          // state 3
        if  (current_channel_A == 1 && current_channel_B == 0) {        // state 4
            Yaw++;      // clockwise
        }
        else if (current_channel_A == 0 && current_channel_B == 1) {    // state 2
            Yaw--;      // counter clockwise
        }
    }
    else if (ground_channel_A == 1 && ground_channel_B == 0) {          // state 4
        if  (current_channel_A == 0 && current_channel_B == 0) {        // state 1
            Yaw++;      // clockwise
        }
        else if (current_channel_A == 1 && current_channel_B == 1) {    // state 3
            Yaw--;      // counter clockwise
        }
    }

    ground_channel_A = current_channel_A;       // update channel A value
    ground_channel_B = current_channel_B;       // update channel B value
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1); // Clean up, clearing the interrupt
}

/***********************************************************************************************
 * initYAW
 * This protocol initialises all the GPIO peripheral for the yaw
 * first the GPIO peripheral GPIO B BASE was enabled. Then PB0 and PB1 were changed to inputs.
 * Then pins 0 and 1 were set for port B to trigger Port B interrupt for rising edge and
 * falling edge on those pins. The Channel_IntHandler function was then attached and the interrupt was then enabled
 **********************************************************************************************/
void
initYAW (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTB_BASE, Channel_IntHandler);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

/***********************************************************************************************
 * Yaw_Calibration_Handler
 * This handler is used find "parking" postion and then be set as the reference for tracking yaw.
 * As reference is always HIGH, on once A LOW signal is detected then it will trigger a flag
 * boolean Yaw_reference_found to become true indicting that the reference is found now the helicopter
 * is in the flying position.
 **********************************************************************************************/
void
Yaw_Calibration_Handler(void)
{
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    if (Yaw_reference_found == false) {
        if (GPIOPinRead (GPIO_PORTC_BASE, GPIO_PIN_4) == 0) {
            Yaw_reference_found = true;
        }
    }

}

/***********************************************************************************************
 * initYawCalibration
 * This protocol initialises all the GPIO peripheral for the calibration, sets helicopter to faces
 * the camera before take off.
 * First the GPIO peripheral GPIO C BASE was enabled. Then PC4 changed to be an input pin, the input
 * pin was then configuration of a pull-down termination since it was an active LOW. Then pin 4 then
 * set for port C to trigger Port C interrupt for rising edge and falling edge on the pin before
 * finally the GPIO interrupt was enabled
 **********************************************************************************************/
void
initYawCalibration (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOIntRegister (GPIO_PORTC_BASE, Yaw_Calibration_Handler);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet (GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);
}

/***********************************************************************************************
 * SysTickIntHandler
 * The interrupt handler for the for SysTick interrupt.
 * Update the button functions
 * Initialized processor trigger for a sample sequence
 * g_ulSampCnt used to count up the buffer size for later use
 **********************************************************************************************/
void
SysTickIntHandler(void)
{
    updateButtons();
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
}

/***********************************************************************************************
 * ADCIntHandler
 * The handler for the ADC conversion complete interrupt and Writes to the circular buffer.
 * First it gets the single sample from ADC0. Then it is placed in the circular buffer (advancing write index)
 * Then when placed in the buffer the interrupt is cleaned and cleared, read to handle the next value
 **********************************************************************************************/
void
ADCIntHandler(void)
{
    uint32_t ulValue;
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    writeCircBuf (&g_inBuffer, ulValue);
    ADCIntClear(ADC0_BASE, 3);
}

/***********************************************************************************************
 * initClock
 * Initialization functions required for the clock including SysTick.
 * first Set the clock rate to 20 MHz. Then wet up the period for the SysTick timer. Then sets
 * SysTick timer period as a function of the system clock. Then register the interrupt handler and
 * Enables the systick interrupt and device. then finally set the PWM clock rate to 10 MHz
 * (which means 80 Hz min rate)
 **********************************************************************************************/
void
initClock (void)
{
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
    SysTickEnable();
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
}

/***********************************************************************************************
 * initADC
 * Protocol for the Initialization of ADC
 * first ADC0 peripheral is enabled for configuration and use. Then Enable sample sequence 3 with
 *  a processor signal trigger. Then Since sample sequence 3 is enabled and registered to an
 *  interrupt handler. Enabling  interrupts for ADC0 sequence 3 clears any outstanding interrupts.
 **********************************************************************************************/
void
initADC (void)
{
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 data-sheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    // changed channel 0 (ADC_CTL_CH0) to channel 9 (ADC_CTL_CH9) which means change from AIN0 to AIN9
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

/***********************************************************************************************
 * initDisplay
 * Protocol for Initialize the Orbit OLED display
 * calls the orbit OLED interface .c file, thus all the Protocol and API function relating to the
 * OLED display are called and setup for use.
 **********************************************************************************************/
void
initDisplay (void)
{
    OLEDInitialise ();
}


/***********************************************************************************************
 * displayState
 * Function used to display a clear screen on the OLED screen and also used to display the a string
 * of size 30 characters wide. This function also shows the butstr string, number of presses and
 * stateStr string
 **********************************************************************************************/
void
displayState (char *butStr, int32_t numPushes, char *stateStr, uint8_t charLine)
{
    char string[30];
        OLEDStringDraw ("                 ", 0, charLine);
        usnprintf (string, sizeof(string), "%s %2d %s", butStr, numPushes, stateStr);
        OLEDStringDraw (string, 0, charLine);
}


void

/***********************************************************************************************
 * main: This is the main function for helicopter controller program
 **********************************************************************************************/
int
main(void)
{
    // Initialized local variables
    uint8_t i;                  // i variable used to index the circular buffer
    uint32_t sum;
    int32_t ground_volt = 0;    // voltage when the helicopter is grounded state
    int32_t current_volt;       // voltage of the helicopter at any state
    uint32_t adc_max = 4096;    // 12 bit ADC maximum value
    uint32_t  v_max = 3300;     // upper limit of operation voltage
    uint8_t first_time = 1;     // flag used indicating the buffer is first time full
    uint32_t mean_ADC = 0;      // the initial ADC value

    //initialization of variable for tracking height and yaw
    uint32_t alt_target = 0;
    uint32_t yaw_target = 0;
    double alt_current = 0;
    double yaw_current = 0;

    // The initialization of variable required for PID control
    double yaw_pass_error = 0;
    double alt_pass_error = 0;

    double alt_kp = 0.7;
    double alt_ki = 0.03;
    double alt_kd = 2.2;

    double yaw_kp = 0.67;
    double yaw_ki = 0.1;
    double yaw_kd = 2.6;

    double alt_P =0;
    double alt_I =0;
    double alt_D =0;

    double yaw_P =0;
    double yaw_I =0;
    double yaw_D=0;

    double alt_control=0;
    double yaw_control=0;

    double alt_intergrated_error = 0;
    double yaw_intergrated_error = 0;

    double alt_derivative_error = 0;
    double yaw_derivative_error = 0;

    double alt_error = 0;
    double yaw_error = 0;

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset(PWM_MAIN_PERIPH_GPIO); //Used for PWM output
    SysCtlPeripheralReset(PWM_MAIN_PERIPH_PWM);  //Main Rotor PWM

    SysCtlPeripheralReset(PWM_TAIL_PERIPH_GPIO); //Used for PWM output
    SysCtlPeripheralReset(PWM_TAIL_PERIPH_PWM);  //Tail Rotor PWM

    SysCtlPeripheralReset(UART_USB_PERIPH_UART); // Reset UART
    SysCtlPeripheralReset(UART_USB_PERIPH_GPIO);


    initClock ();
    initYawCalibration ();
    initYAW ();
    initButtons ();
    initADC ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initialiseUSB_UART ();
    initSysTick ();
    initMotors();
    EnableMotor();

    IntMasterEnable();  // Enable interrupts to the processor.

    while (1)
    {
            uint32_t main_duty = ReturnMainDutyCycle();
            uint32_t tail_duty = ReturnTailDutyCycle();

            //Perform a software reset of the entire device
            if (checkButton(RESET) == PUSHED) {
                SysCtlReset();
            }
            /********************************START of State machine section *****************************/
            // each switch case represents a state on the helicopter controller
            switch (current_mode) {
            // Turn on state of controller
            case TURN_ON_MODE:
                disableMotor(); // ensure that when at TUN ON MODE the motors are always off
                if (checkButton(SWITCH_RIGHT) == PUSHED){
                    next_mode = CALABRATION_MODE;
                }
                break;

            // Calabration state of controller
            case CALABRATION_MODE:
                EnableMotor();
                 if (Yaw_reference_found == true)
                 {
                     IntMasterDisable();
                     yaw_current  = 0;
                     yaw_target = 0;
                     yaw_error = 0;
                     yaw_pass_error = 0;
                     yaw_intergrated_error = 0;
                     yaw_derivative_error = 0;
                     IntMasterEnable();
                     next_mode = FLYING_MODE;
                 }
                break;

            // Flying state of controller
            // UP, DOWN, CCW, CW  buttons are only functional when in flying mode
            case FLYING_MODE:
                if (checkButton(UP) == PUSHED){
                    alt_target += 10;
                }
                else if (checkButton(DOWN) == PUSHED){
                    alt_target -= 10;
                }
                else if (checkButton(LEFT) == PUSHED) {
                    yaw_target += 15;
                }
                else if (checkButton(RIGHT) == PUSHED) {
                    yaw_target -= 15;
                }
                else if (checkButton(SWITCH_RIGHT) == RELEASED){
                    next_mode = LANDING_MODE;
                }
                break;

            // Landing state of controller
            case LANDING_MODE:
                yaw_target = 0;
                if (((yaw_target - yaw_current) < 30 | (yaw_target - yaw_current) > -30) && alt_current == alt_target) {
                    alt_target -= 5;
                }
                if (alt_current == 0) {
                    next_mode = TURN_ON_MODE;
                }
                break;
            }
            /********************************END of State machine section *****************************/
            sum = 0;

            // for loop used to constantly update the mean ADC value
            for (i = 0; i < BUF_SIZE; i++) {
                sum = sum + readCircBuf (&g_inBuffer);
                mean_ADC = ((2 * sum + BUF_SIZE) / 2) / BUF_SIZE;
            }

            // if statement used to get the mean voltage at the start and check the buffer is full
            if (first_time && (g_ulSampCnt % BUF_SIZE) == (BUF_SIZE - 1)) {
                ground_volt = (mean_ADC * v_max)/ adc_max;      // calculate the ground voltage
                first_time = 0;     // flag to tell program to end the calibration
            }
            // if statement used to claculate current voltage
            if (first_time == 0) {
                current_volt = (mean_ADC * v_max)/ adc_max;     // calaculate voltage eqaution
                }
            IntMasterDisable();
            // Critical section, all interrupts disabled while determining the altitude and yaw of helicopter
            alt_current = ((ground_volt - current_volt)/8);     // the equation to calculate the altitude
            yaw_current = (Yaw * 360)/(4*112);      // the equation to calculate the yaw
            IntMasterEnable();

            /*******************************PID control START **************************/
            //enabling the PID during the flying calibration and landing state
            if (current_mode == FLYING_MODE | current_mode == CALABRATION_MODE | current_mode== LANDING_MODE){
                IntMasterDisable();
                // Critical section for PID control
                alt_error = alt_target- alt_current;
                yaw_error = yaw_target- yaw_current;

                alt_intergrated_error += alt_error;
                yaw_intergrated_error += yaw_error;

                alt_derivative_error = alt_error - alt_pass_error;
                yaw_derivative_error = yaw_error - yaw_pass_error;

                alt_P = alt_kp * alt_error;
                alt_I = alt_ki * alt_intergrated_error;
                alt_D = alt_kd * (alt_derivative_error);

                yaw_P = yaw_kp * yaw_error;
                yaw_I = yaw_ki * yaw_intergrated_error;
                yaw_D = yaw_kd * (yaw_derivative_error);

                alt_control = (alt_P + alt_I + alt_D);
                yaw_control = (yaw_P + yaw_I + yaw_D);

                alt_pass_error = alt_error;
                yaw_pass_error = yaw_error;
                IntMasterEnable();
            }
            /*******************************PID control END **************************/

            // If statement uses set fixed duty cycles during calbration state only
            if ((current_mode == CALABRATION_MODE) && (Yaw_reference_found == false)){
                set_main_PWM(12);
                set_tail_PWM(14);
            }
            else {
                set_main_PWM(alt_control);
                set_tail_PWM(yaw_control);
            }
            current_mode = next_mode; // updating the mode after each cycle

            /******************************display on UART START *****************************/
            usnprintf (statusStr, sizeof(statusStr), "Alt:%2d(%2d)YAW:%2d(%2d)\n\rM:%2dT:%2d\n\r", (int)alt_current, alt_target, (int)yaw_current, yaw_target, (int)main_duty, (int)tail_duty);
            UARTSend (statusStr);
            if(current_mode == 0 | current_mode == 1 | current_mode == 4)
            {
                usnprintf (statusStr, sizeof(statusStr), "Mode[LANDED]\n\r");
            }
            else if (current_mode == 2) {
                usnprintf (statusStr, sizeof(statusStr), "Mode[FLYING]\n\r");
            }
            else if (current_mode == 3) {
                usnprintf (statusStr, sizeof(statusStr), "Mode[LANDING]\n\r");
           }
           UARTSend (statusStr);
           /******************************display on UART END *****************************/

           SysCtlDelay (SysCtlClockGet() / 24);  // Update display at ~ 8 Hz
    }
}
