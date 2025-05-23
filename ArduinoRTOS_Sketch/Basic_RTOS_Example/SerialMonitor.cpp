/*
 * SerialMonitor.cpp
 *
 * Created: 5/5/2025 7:24:27 PM
 *  Author: Abhir
 */ 

#include <FreeRTOS_SAMD21.h> //samd21
#include "SerialMonitor.h"
#include "AL_Serial.h"

void serialMonitor(void *p)
{
	uint8_t byteData;
	uint8_t dataReceived[100];
	uint32_t counter;	
		
	//get instance of serial ports
	uartPort* serial0 = uartPort::getInstance(0);

	while (1)
	{
		//check if ring buffer contains data		
		counter = serial0->peekRcvd();
		if(counter)
		{
			//read bytes received
			serial0->read(dataReceived, &counter);
			
			//send them back 
			serial0->write(dataReceived, counter);

		}		
		else
		{
			serial0->write((uint8_t*)"U",1);
		}
				
		//sleep for 1000ticks.
		vTaskDelay(1000);	
	}//end while
}//end task
