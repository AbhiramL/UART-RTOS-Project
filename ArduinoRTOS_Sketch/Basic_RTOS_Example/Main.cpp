/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h> //samd21
#include "task.h"
#include "custom_uart.h"


//Beginning of Auto generated function prototypes by Atmel Studio

//End of Auto generated function prototypes by Atmel Studio



//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define  LED_PIN  13 //Led Pin: Typical Arduino Board
#define LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high

// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
//#define SERIAL          SerialUSB
#define SERIAL          Serial1

//**************************************************************************
// global variables
//**************************************************************************
uartPort* uart0 = NULL;

void setup() 
{

   uart0 = new uartPort(SERIAL0);	
   uart0->write((uint8_t*)"HELLO", 5);
//  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  //while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

   
  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
 
  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
 

  // Start the RTOS, this function will never return and will schedule the tasks.
	
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
	int i;
    uart0->write((uint8_t*)".", 1);
	
	i=0;
	while(i++ < 1000){};
}


//*****************************************************************
