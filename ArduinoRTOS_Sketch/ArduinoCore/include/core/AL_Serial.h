#ifndef CUSTOM_UART_H_
#define CUSTOM_UART_H_

#include <sam.h>
#include <stdio.h>
#include <stdlib.h>
#include "AL_Serial_Reg.h"

class uartPort
{
	public:
		const static uint32_t UART_RING_BUFFER_SIZE = 100;
	
		static uartPort* getInstance(int portNum);

		static void IRQ_Handler(int portNum);

		uint32_t write(uint8_t* buf, uint16_t maxSize);
		uint32_t read(uint8_t* buf, uint32_t *bytesread);

		uint32_t peekRcvd();		
		uint32_t readByte(uint8_t* buf);
		uint32_t writeByte(uint8_t data);
				
		~uartPort();
		
	private:			
		struct ringBuf
		{
			uint8_t buf[UART_RING_BUFFER_SIZE];
			int head = 0;
			int tail = 0;
		};
		
		uartPort(uartRegisterMap* baseAddr);
		//uartPort(uartRegisterMap* baseAddr, uint32_t baudRate);
		//uartPort(uartRegisterMap* baseAddr, uint8_t txPin, uint8_t rxPin, uint32_t baudRate);
		
		void initClockNVIC();
		
		uartRegisterMap* uartRegisters;
		ringBuf rx_buffer;
		ringBuf tx_buffer;
};


#endif /* CUSTOM_UART_H_ */