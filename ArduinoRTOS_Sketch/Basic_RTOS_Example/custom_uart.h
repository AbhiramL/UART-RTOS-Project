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
		void IRQ_RXC_Handler();
		~uartPort();
		
	private:
			
		struct ringBuf
		{
			uint8_t buf[100];
			int head = 0;
			int tail = 0;
		};
		
		void initClockNVIC();
		
		uartRegisterMap* uartRegisters;
		GenClockRegMap* genClkReg;
		ringBuf receivedBytes;
		
		
};

/****************************************************/









#endif /* CUSTOM_UART_H_ */