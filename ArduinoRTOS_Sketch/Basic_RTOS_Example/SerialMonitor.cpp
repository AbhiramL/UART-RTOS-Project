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
	//get instance of serial ports
	uartPort* serial0 = uartPort::getInstance(0);
	
	uint8_t byteData;
	uint8_t dataReceived[100];
	while (1)
	{
		//check if ring buffer contains data
		byteData = serial0->readByte();
		int counter = 0;
		
		if(byteData != NULL)	//if yes, read all the data with while loop
		{
			do 
			{
				dataReceived[counter] = byteData;
				counter++;
				byteData = serial0->readByte();
				
			} while (byteData != NULL);
		}
		
		serial0->write((uint8_t*)"HELLO", sizeof("HELLO"));
				
		//sleep
		vTaskDelay(1000);	
	}//end while
}//end task
