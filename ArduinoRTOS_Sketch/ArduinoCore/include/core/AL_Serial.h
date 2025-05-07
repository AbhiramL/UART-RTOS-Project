#ifndef CUSTOM_UART_H_
#define CUSTOM_UART_H_

#include <sam.h>
#include <stdio.h>
#include <stdlib.h>
#include "AL_Serial_Reg.h"

class uartPort
{
	public:
		void write(uint8_t* buf, uint16_t maxSize);
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
		
		void writeByte(uint8_t data);
		
		void initClockNVIC();
		
		uartRegisterMap* uartRegisters;
		ringBuf rx_buffer;
		ringBuf tx_buffer;
};


#endif /* CUSTOM_UART_H_ */