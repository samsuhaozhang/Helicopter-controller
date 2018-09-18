#ifndef AVONIC_UART_H_
#define AVONIC_UART_H_
//****************************************************************
//
// avonic uart.c this is the demo code for UART communication with the helicopter rigs
// Link with modules:  buttons, OrbitOLEDInterface
//
// Author: Sam Zhang
// Last modified:   29.5.2017
// ***************************************************************/
#include <stdint.h>
#include <stdbool.h>
//********************************************************
// Global Constants
//********************************************************
#define SYSTICK_RATE_HZ 100
#define SLOWTICK_RATE_HZ 4
#define MAX_STR_LEN 32
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX
//********************************************************
// Global variables
//********************************************************
char statusStr[MAX_STR_LEN + 1];
//*******************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*******************************************************************
void
SysTickIntHandler (void);

//********************************************************
// Initialisation functions: clock, SysTick, display, UART
//********************************************************
void
initClock (void);

//*******************************************************************
void
initSysTick (void);

// *******************************************************
void
initDisplay (void);

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initialiseUSB_UART (void);

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer);
#endif

/* AVONIC_UART_H_ */
