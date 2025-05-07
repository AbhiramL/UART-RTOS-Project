/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <FreeRTOS_SAMD21.h> //samd21
#include "FreeRTOS.h"
#include "task.h"
#include "AL_Serial.h"
#include "SerialMonitor.h"

#define  LED_PIN  13 //Led Pin: Typical Arduino Board
#define LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high


//**************************************************************************
// global variables
//**************************************************************************
#define TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
#define SM_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

TaskHandle_t Handle_serialMonitor;


void setup() 
{

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
	vSetErrorLed(LED_PIN, LED_LIGHTUP_STATE);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	
	uartPort* serial0 = uartPort::getInstance(0);
	
	serial0->write((uint8_t*)"HELLO", sizeof("HELLO"));
	
  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
	xTaskCreate(serialMonitor, "Serial Monitor", TASK_STACK_SIZE, NULL, SM_TASK_PRIORITY, &Handle_serialMonitor);

  // Start the RTOS, this function will never return and will schedule the tasks.
	vTaskStartScheduler();
}

void loop()
{
	
};
