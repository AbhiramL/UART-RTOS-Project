#ifndef CUSTOM_UART_H_
#define CUSTOM_UART_H_

#include <sam.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "custom_uart_reg_def.h"
//#include "wiring_private.h"
//#include "WVariant.h"

/****************************************************/
//uart0 class that has access to all registers
/****************************************************/
class uartPort
{
	public:
		uartPort(uartRegisterMap* baseAddr);
	    uartPort(uartRegisterMap* baseAddr, uint32_t baudRate);
		uartPort(uartRegisterMap* baseAddr, uint8_t txPin, uint8_t rxPin, uint32_t baudRate);
		uint32_t write(uint8_t* buf, uint32_t maxSize);
		uint8_t read();
		~uartPort();
		
	private:
		struct FIFO_Buffer
		{
			uint8_t buffer[200];
			uint8_t* tail;
			uint8_t* head;
			uint8_t count;
		};
	
		void initClockNVIC();
			
		uartRegisterMap* uartRegisters;
		GenClockRegMap* genClkReg;
};

/****************************************************/









#endif /* CUSTOM_UART_H_ */