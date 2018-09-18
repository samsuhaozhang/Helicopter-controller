//*****************************************************************************
// Milestone_2.c - this is the main program that performs the tasks set in milestone 2 of the avionic control project
//
// Author: Sam Zhang
// Last modified:   17.04.2018
//*****************************************************************************
// This program is  modified version of milestone_1.c
// The following modules are required to run Milestone_2.c
// buttons.c, button.h, circBufT.c, circBufT.h, OrbitOLED folder and ustdlib.c

//*****************************************************************************

//milestone 1 includes
#include <stdint.h>
#include <stdbool.h>
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
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons.h"

//milestone 2 includes
#include "driverlib/pin_map.h"

// Constants
#define BUF_SIZE 8              // Buffer size
#define SAMPLE_RATE_HZ 320       // Sampling rate


// Initialize Global variables for milestone 1
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;        // Counter for the interrupts
static uint32_t display_state = 1;  // flag used to switch the display on the OLED screen

// Initialize Global variables for milestone 2
static volatile bool ground_channel_A = 0;  // set channel B value to be a LOW
static volatile bool ground_channel_B = 0;  // set channel A to be a HIGH
static volatile bool current_channel_A;     // reads pwm signal of channel A
static volatile bool current_channel_B;     // reads pwm signal of channel B
static int32_t Yaw = 0;                     // incremental yaw tracking

#define GPIO_PIN_0              0x00000001  // GPIO pin 0
#define GPIO_PIN_1              0x00000002  // GPIO pin 1


// The interrupt handler for the yaw function
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
            Yaw--;      // anti clockwise
        }
    }
    else if (ground_channel_A == 0 && ground_channel_B == 1) {          // state 2
        if  (current_channel_A == 1 && current_channel_B == 1) {        // state 3
            Yaw++;      // clockwise
        }
        else if  (current_channel_A == 0 && current_channel_B == 0) {   // state 1
            Yaw--;      // anti clockwise
        }
    }
    else if (ground_channel_A == 1 && ground_channel_B == 1) {          // state 3
        if  (current_channel_A == 1 && current_channel_B == 0) {        // state 4
            Yaw++;      // clockwise
        }
        else if (current_channel_A == 0 && current_channel_B == 1) {    // state 2
            Yaw--;      // anti clockwise
        }
    }
    else if (ground_channel_A == 1 && ground_channel_B == 0) {          // state 4
        if  (current_channel_A == 0 && current_channel_B == 0) {        // state 1
            Yaw++;      // clockwise
        }
        else if (current_channel_A == 1 && current_channel_B == 1) {    // state 3
            Yaw--;      // anti clockwise
        }
    }

    ground_channel_A = current_channel_A;       // update channel A value
    ground_channel_B = current_channel_B;       // update channel B value

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1); // Clean up, clearing the interrupt
}

// initialization of the GPIO peripheral for the yaw
void
initYAW (void)
{
    // Enable the GPIO B BASE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Change the PB0 and PB1 to inputs
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    // Set pins 0 and 1 for port B to trigger Port B interrupt for rising edge  and falling edge on those pins
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

    // Attach a function to the interrupt for port A (change to B, use your own interruption handler function)
    GPIOIntRegister(GPIO_PORTB_BASE, Channel_IntHandler);

    // Enable interrupts for port B on those pins PB) and PB1
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

// The interrupt handler for the for SysTick interrupt.
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

// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

// Initialization functions for the clock (incl. SysTick), ADC, display
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

// Initialization of ADC
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

void
initDisplay (void)
{
    // Initialize the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************

// Function to display the mean ADC value, voltage, altitude percentage and blank space.
void
displayState (char *butStr, int32_t numPushes, char *stateStr, uint8_t charLine)
{
    char string[17]; // Display fits 16 characters wide.

    if(display_state == 3) {
        // display clear screen
        OLEDStringDraw ("                 ", 0, charLine);
        OLEDStringDraw ("                 ", 1, charLine);

    } else{
        //displays helicopter altitude, yaw and voltage reading
        usnprintf (string, sizeof(string), "%s %2d %s", butStr, numPushes, stateStr);
        OLEDStringDraw (string, 0, charLine);
    }

}
// Main function for the helicopter milestone
int
main(void) // this is the main function for controller program
 {
    // Initialized local variables
    uint32_t i;                 // j variable used to index the circular buffer
    uint32_t butState;          // flag to check the state of button
    uint32_t sum;

    int32_t ground_volt = 0;    // voltage when the helicopter is grounded state
    int32_t current_volt;       // voltage of the helicopter at any state
    int32_t percentage = 0;

    uint32_t adc_max = 4096;    // 12 bit ADC maximum value
    uint32_t  v_max = 3300;     // upper limit of operation voltage

    uint32_t first_time = 1;    // flag used indicating the buffer is first time full
    uint32_t mean_ADC = 0;      // the initial ADC value

    int32_t yaw_degree = 0;     // yaw in degrees

    initYAW ();
    initClock ();
    initButtons ();
    initADC ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);

    IntMasterEnable();  // Enable interrupts to the processor.

    while (1)
    {
        sum = 0;
        // for loop used to constantly update the mean ADC value
        for (i = 0; i < BUF_SIZE; i++) {
            sum = sum + readCircBuf (&g_inBuffer);
            mean_ADC = ((2 * sum + BUF_SIZE) / 2) / BUF_SIZE;
        }

        if (first_time && (g_ulSampCnt % BUF_SIZE) == (BUF_SIZE - 1)) {  // if statement used to get the mean voltage at the start and check the buffer is full
            ground_volt = (mean_ADC * v_max)/ adc_max; // calculate the ground voltage
            first_time = 0;                 // flag to tell program to end the calibration
        }

        if (first_time == 0) {
            current_volt = (mean_ADC * v_max)/ adc_max; // calculate the current voltage
        }

        percentage = ((ground_volt - current_volt)/8); // the equation to calculate the altitude percentage
        yaw_degree = (Yaw * 360)/(4*112) ;

        if (display_state == 1) { // denote the OLED display of the altitude percentage and yaw in dregree as display state 1
            displayState ("ALTITUDE", percentage, "%    ", 0);
            displayState ("YAW", yaw_degree, "DEGREE    ", 1);

        }
        else if (display_state == 2) {// denote the OLED display of the current mean ADC and current voltage as display state 2
            displayState ("ADC MEAN", mean_ADC, " ", 0);
            displayState ("VOLTAGE", current_volt, "mV", 1);
        }
        else if (display_state == 3) { // denote the blank OLED screen as display state 3
            displayState ("\n", current_volt, "\n", 0);
            displayState ("\n", current_volt, "\n", 1);
        }

        butState = checkButton (LEFT);  // when left button is pressed, OLED screen will switch to display state 1
        switch (butState)
        {
        case PUSHED:
            first_time = 1;
            display_state = 1;
            break;
        }

        if (display_state == 1 ){               // when up button is pressed for first time, OLED screen will switch to display state 2
            butState = checkButton (UP);
            switch (butState)
            {
            case PUSHED:
                display_state = 2;
                break;
            }
        } else if (display_state == 2) {        // when up button is pressed for second time, OLED screen will switch to display state 3
            butState = checkButton (UP);
            switch (butState)
            {
            case PUSHED:
                display_state = 3;
                break;
            }
        } else if (display_state == 3){         // when up button is pressed for third time, OLED screen will switch to display state 1
            butState = checkButton (UP);
            switch (butState)
            {
            case PUSHED:
                display_state = 1;
                break;
            }
        }
        SysCtlDelay (SysCtlClockGet() / 24);  // Update display at ~ 8 Hz
    }
}
