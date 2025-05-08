
#include "AL_Serial.h"
#include "Arduino.h"
#include "wiring_private.h"

#define UART0_TX_PIN_ID		1
#define UART0_RX_PIN_ID		0
#define UART0_TX_PAD_ID		0x1
#define UART0_RX_PAD_ID		0x3 


//#include "custom_uart_reg_def.h"

uartPort* uartPort::getInstance(int portNum)
{
	static uartPort serialPort0 = uartPort(SERIAL0);
	//static uartPort serialPort1(SERIAL1);
	//static uartPort serialPort0(SERIAL0);
	
	uartPort* instance = NULL;
	
	switch(portNum)
	{
		case 0:
			instance = &serialPort0;
			break;
			
		case 1:
			//instance = &serialPort1;
			break;
			
		default:
			instance = &serialPort0;
			break;
	}
	
	return instance;
}


uartPort::uartPort(uartRegisterMap* baseAddr, uint8_t txPin, uint8_t rxPin, uint32_t baudRate)
{
	//base addr should be only Serial 0.
	uartRegisters = baseAddr;
		
	//Initialize generic clock to use internal clock for Async
	initClockNVIC();
	
	//set all registers to default values
	uartRegisters->CTRLA.bit_data.swrst = 1;
	while(uartRegisters->SYNCBUSY.bit_data.swrst){};
	
	//Disable port temporarily
	uartRegisters->CTRLA.bit_data.enable = 0x0;
	
	//Init tx rx pins, pin 0 and 1 correspond to rx and tx pins on M0 PRO
	pinPeripheral(UART0_RX_PIN_ID,g_APinDescription[UART0_RX_PIN_ID].ulPinType); //rx id 0, { PORTA, 11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
	pinPeripheral(UART0_TX_PIN_ID,g_APinDescription[UART0_TX_PIN_ID].ulPinType); //tx id 1, //{ PORTA, 10, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]
	
	//Set tx and rx pads
	uartRegisters->CTRLA.bit_data.rxpo = UART0_RX_PAD_ID; //sercom0 pad 3
	uartRegisters->CTRLA.bit_data.txpo = UART0_TX_PAD_ID; //sercom0 pad 2
	
	//Set default sample rate x16
	uartRegisters->CTRLA.bit_data.sampr = 0x1; //x16 sample rate with frac baud rate generation
	
	//Set default Baud rate, 115200, 16 sample rate
	uint32_t baudTimes8 = (SystemCoreClock * 8) / (16 * 115200);
	uartRegisters->BAUD.FRAC.FP   = (baudTimes8 % 8);
	uartRegisters->BAUD.FRAC.BAUD = (baudTimes8 / 8);
	
	//Set internal clock mode
	uartRegisters->CTRLA.bit_data.mode = 0x1;
	
	//Set enable interrupts
	uartRegisters->INTENSET.bit_data.err = 1;
	uartRegisters->INTENSET.bit_data.rxc = 1;
	uartRegisters->INTENSET.bit_data.txc = 1;	
	
	//Set char size
	uartRegisters->CTRLB.bit_data.chsize = 0x0;
	
	//Set LSB or MSB first
	uartRegisters->CTRLA.bit_data.dord = 1; //MSB=0, LSB=1;
	
	//Set parity
	uartRegisters->CTRLB.bit_data.pmode = 0; //0=even 1=odd parity
	
	//Set stop bit
	uartRegisters->CTRLB.bit_data.sbmode = 0; //0=1 stop bit  1=2 stop bit
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	
	//Enable port
	uartRegisters->CTRLA.bit_data.enable = 0x1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.txen = 1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.rxen = 1;
	
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
}

uartPort::uartPort(uartRegisterMap* baseAddr, uint32_t baudRate)
{
	//base addr should be only Serial 0.
	uartRegisters = baseAddr;
	
	
	//Initialize generic clock to use internal clock for Async
	initClockNVIC();
	
	//set all registers to default values
	uartRegisters->CTRLA.bit_data.swrst = 1;
	while(uartRegisters->SYNCBUSY.bit_data.swrst){};
	
	//Disable port temporarily
	uartRegisters->CTRLA.bit_data.enable = 0x0;
	
	//Init tx rx pins, pin 0 and 1 correspond to rx and tx pins on M0 PRO
	pinPeripheral(UART0_RX_PIN_ID,g_APinDescription[UART0_RX_PIN_ID].ulPinType); //rx id 0, { PORTA, 11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
	pinPeripheral(UART0_TX_PIN_ID,g_APinDescription[UART0_TX_PIN_ID].ulPinType); //tx id 1, //{ PORTA, 10, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]
	
	//Set tx and rx pads
	uartRegisters->CTRLA.bit_data.rxpo = UART0_RX_PAD_ID; //sercom0 pad 3
	uartRegisters->CTRLA.bit_data.txpo = UART0_TX_PAD_ID; //sercom0 pad 2
	
	//Set default sample rate x16
	uartRegisters->CTRLA.bit_data.sampr = 0x1; //x16 sample rate with frac baud rate generation
	
	//Set default Baud rate, 115200, 16 sample rate
	uint32_t baudTimes8 = (SystemCoreClock * 8) / (16 * 115200);
	uartRegisters->BAUD.FRAC.FP   = (baudTimes8 % 8);
	uartRegisters->BAUD.FRAC.BAUD = (baudTimes8 / 8);
	
	//Set internal clock mode
	uartRegisters->CTRLA.bit_data.mode = 0x1;
	
	//Set enable interrupts
	uartRegisters->INTENSET.bit_data.err = 1;
	uartRegisters->INTENSET.bit_data.rxc = 1;
	uartRegisters->INTENSET.bit_data.txc = 1;	

	
	//Set char size
	uartRegisters->CTRLB.bit_data.chsize = 0x0;
	
	//Set LSB or MSB first
	uartRegisters->CTRLA.bit_data.dord = 1; //MSB=0, LSB=1;
	
	//Set parity
	uartRegisters->CTRLB.bit_data.pmode = 0; //0=even 1=odd parity
	
	//Set stop bit
	uartRegisters->CTRLB.bit_data.sbmode = 0; //0=1 stop bit  1=2 stop bit
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	
	//Enable port
	uartRegisters->CTRLA.bit_data.enable = 0x1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.txen = 1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.rxen = 1;
	
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
}

uartPort::uartPort(uartRegisterMap* baseAddr)
{
	//base addr should be only Serial 0.
	uartRegisters = baseAddr;
	
	//Initialize generic clock to use internal clock for Async
	initClockNVIC();
	
	//set all registers to default values
	uartRegisters->CTRLA.bit_data.swrst = 1;
	while(uartRegisters->SYNCBUSY.bit_data.swrst){};
	
	//Disable port temporarily
	uartRegisters->CTRLA.bit_data.enable = 0x0;
	
	//Init tx rx pins, pin 0 and 1 correspond to rx and tx pins on M0 PRO
	pinPeripheral(UART0_RX_PIN_ID,g_APinDescription[UART0_RX_PIN_ID].ulPinType); //rx id 0, { PORTA, 11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
	pinPeripheral(UART0_TX_PIN_ID,g_APinDescription[UART0_TX_PIN_ID].ulPinType); //tx id 1, //{ PORTA, 10, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]
	
	//Set tx and rx pads
	uartRegisters->CTRLA.bit_data.rxpo = UART0_RX_PAD_ID; //sercom0 pad 3
	uartRegisters->CTRLA.bit_data.txpo = UART0_TX_PAD_ID; //sercom0 pad 2
		
	//Set default sample rate x16
	uartRegisters->CTRLA.bit_data.sampr = 0x1; //x16 sample rate with frac baud rate generation
	
	//Set default Baud rate, 115200, 16 sample rate
	uint32_t baudTimes8 = (SystemCoreClock * 8) / (16 * 115200);
	uartRegisters->BAUD.FRAC.FP   = (baudTimes8 % 8);
	uartRegisters->BAUD.FRAC.BAUD = (baudTimes8 / 8);
	
	//Set internal clock mode
	uartRegisters->CTRLA.bit_data.mode = 0x1;
	
	//Set enable interrupts
	uartRegisters->INTENSET.bit_data.err = 1;
	uartRegisters->INTENSET.bit_data.rxc = 1;
	uartRegisters->INTENSET.bit_data.txc = 1;	
	
	
	//Set char size
	uartRegisters->CTRLB.bit_data.chsize = 0x0;
	
	//Set LSB or MSB first
	uartRegisters->CTRLA.bit_data.dord = 1; //MSB=0, LSB=1;
	
	//Set parity
	uartRegisters->CTRLB.bit_data.pmode = 0; //0=even 1=odd parity
	
	//Set stop bit
	uartRegisters->CTRLB.bit_data.sbmode = 0; //0=1 stop bit  1=2 stop bit
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	
	//Enable port
	uartRegisters->CTRLA.bit_data.enable = 0x1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.txen = 1;
	
	//Wait for ENABLE sync
	while (uartRegisters->SYNCBUSY.bit_data.enable){};
	uartRegisters->CTRLB.bit_data.rxen = 1;
	
	while (uartRegisters->SYNCBUSY.bit_data.enable){};		
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Purpose:Write buffer of size 'maxSize' into the Uart ring buffer.
// Parameters:
// Result: 1: Success, 0:Failure.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint32_t uartPort::write(uint8_t* buf, uint16_t maxSize)
{
	uint32_t result = 1;
	
	for(uint16_t i = 0; i < maxSize; i++)
	{
		if( !(result= writeByte(buf[i])) )
		{
			break;
		};
	}
	
	return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Purpose:Write 1 bytes into the Uart ring buffer.
// Parameters:
// Result: 1: Success, 0:Failure.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint32_t uartPort::writeByte(uint8_t byte)
{
	uint32_t result = 1;
	
	//kickstart transmission
	if(this->uartRegisters->INTFLAG.bit_data.dre)
	{
		this->uartRegisters->DATA.bit_data.data = byte;
	}
	else
	{
		tx_buffer.buf[tx_buffer.tail] = byte;
			
		if(++tx_buffer.tail >= sizeof(tx_buffer.buf)) //if tail is at the end of transmit data buffer
		{
			tx_buffer.tail = 0;   //reset tail
		}
	}
	
	return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Purpose:
// Parameters:
// Result: 
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint32_t uartPort::peekRcvd()
{
	uint32_t cnt;
	cnt = (rx_buffer.tail >= rx_buffer.head)? (rx_buffer.tail - rx_buffer.head):(UART_RING_BUFFER_SIZE - (rx_buffer.head - rx_buffer.tail));
	return(cnt);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Purpose:read 1 bytes from the Uart ring buffer.
// Parameters:
// Result: 1: Success, 0:Failure.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint32_t uartPort::readByte(uint8_t* buf)
{
	uint32_t result = 1;

	if(rx_buffer.head == rx_buffer.tail) //no bytes in the buffer to read
	{
		//ring buffer empty,
		result = 0;
	}
	else  //bytes exist in ring buffer
	{
		*buf = rx_buffer.buf[rx_buffer.head];
		
		//increment head in the ring buffer
		if(++rx_buffer.head >= sizeof(rx_buffer.buf))
		{
			rx_buffer.head = 0;
		}
	}
	
	return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Purpose:read Uart received content into the buffer.
// Parameters:
// Result: 1: Success, 0:Failure.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint32_t uartPort::read(uint8_t* buf, uint32_t* bytesread)
{
	uint32_t result = 1;
	
	*bytesread = 0;

	//while there are bytes to read
	while(readByte(&buf[*bytesread]))
	{
		*bytesread = *bytesread + 1;
	}

	return result;
}


void uartPort::IRQ_Handler(int portNum)
{
	uartPort* uport = uartPort::getInstance(portNum);
	
	if(uport->uartRegisters->INTFLAG.bit_data.err == 1) //frame error occurred
	{
		//discard byte in DATA register by reading it and not saving it
		uint8_t dataByte = uport->uartRegisters->DATA.bit_data.data;
		
		//clear FERR bit by writing a 1 to it
		uport->uartRegisters->INTFLAG.bit_data.err = 1;
	}
	
	if( (uport->uartRegisters->INTFLAG.bit_data.rxc == 1) ) //data available to be read
	{
		uport->rx_buffer.buf[uport->rx_buffer.tail] = uport->uartRegisters->DATA.bit_data.data; //read DATA reg
		
		if(++uport->rx_buffer.tail >= sizeof(uport->rx_buffer.buf)) //if tail is at the end of the buf length
		{
			uport->rx_buffer.tail = 0;  //reset tail
		}
	}


	//clear txc interrupt as there is no more data to transmit
	uport->uartRegisters->INTFLAG.bit_data.txc = 1;


	if( (uport->uartRegisters->INTFLAG.bit_data.dre)  &&
	    (uport->tx_buffer.head != uport->tx_buffer.tail) ) //data reg empty, proceed to write to DATA reg
	{
		uport->uartRegisters->DATA.bit_data.data = (uint16_t)uport->tx_buffer.buf[uport->tx_buffer.head]; //write head byte to DATA reg
		
		if(++uport->tx_buffer.head >= sizeof(uport->tx_buffer.buf)) //if head is at the end of transmit data buffer
		{
			uport->tx_buffer.head = 0;   //reset head
		}
	}

}

uartPort::~uartPort()
{
	//Disable hardware
	uartRegisters->CTRLA.bit_data.enable = 0;// &= ~(1 << 1);
}

void uartPort::initClockNVIC( void )
{
	uint8_t clockId = 0;
	IRQn_Type IdNvic=PendSV_IRQn ; // Dummy init to intercept potential error later

	if(uartRegisters == SERIAL0)
	{
		clockId = GCM_SERCOM0_CORE;
		IdNvic = SERCOM0_IRQn;
	}
	/*else if(sercom == SERCOM1)
	{
		clockId = GCM_SERCOM1_CORE;
		IdNvic = SERCOM1_IRQn;
	}
	else if(sercom == SERCOM2)
	{
		clockId = GCM_SERCOM2_CORE;
		IdNvic = SERCOM2_IRQn;
	}
	else if(sercom == SERCOM3)
	{
		clockId = GCM_SERCOM3_CORE;
		IdNvic = SERCOM3_IRQn;
	}
	#if defined(SERCOM4)
	else if(sercom == SERCOM4)
	{
		clockId = GCM_SERCOM4_CORE;
		IdNvic = SERCOM4_IRQn;
	}
	#endif // SERCOM4
	#if defined(SERCOM5)
	else if(sercom == SERCOM5)
	{
		clockId = GCM_SERCOM5_CORE;
		IdNvic = SERCOM5_IRQn;
	}
	#endif // SERCOM5

	if ( IdNvic == PendSV_IRQn )
	{
		// We got a problem here
		return ;
	}*/

	// Setting NVIC
	NVIC_EnableIRQ(IdNvic);
	NVIC_SetPriority (IdNvic, SERCOM_NVIC_PRIORITY);  /* set Priority */

	//Setting clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
	GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
	GCLK_CLKCTRL_CLKEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}
}

