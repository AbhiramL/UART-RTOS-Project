#ifndef CUSTOM_UART_H_
#define CUSTOM_UART_H_

#include <sam.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "AL_Serial_Reg.h"

class uartPort
{
	public:
		uint32_t write(uint8_t* buf, uint32_t maxSize);
		uint8_t readByte();
		static uartPort* getInstance(int portNum);
		static void IRQ_Handler(int portNum);
		~uartPort();
		
	private:			
		struct ringBuf
		{
			uint8_t buf[100];
			int head = 0;
			int tail = 0;
		};
		
		uartPort(uartRegisterMap* baseAddr);
		uartPort(uartRegisterMap* baseAddr, uint32_t baudRate);
		uartPort(uartRegisterMap* baseAddr, uint8_t txPin, uint8_t rxPin, uint32_t baudRate);
		
		void initClockNVIC();
		
		uartRegisterMap* uartRegisters;
		GenClockRegMap* genClkReg;
		ringBuf receivedBytes;
		uint8_t returnBuffer;
};


#endif /* CUSTOM_UART_H_ */