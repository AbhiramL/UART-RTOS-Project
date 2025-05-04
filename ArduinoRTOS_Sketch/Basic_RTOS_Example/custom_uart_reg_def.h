/*
 * custom_uart_reg_def.h
 *
 * Created: 5/3/2025 3:48:35 PM
 *  Author: Abhir
 */ 
#ifndef CUSTOM_UART_REG_DEF_H_
#define CUSTOM_UART_REG_DEF_H_

#define GCLK              0x40000C00UL /**< \brief (GCLK) APB Base Address */
#define GCLK_SERCOM0_CLKID          (0x14U)
#define GCLK_SERCOM1_CLKID          (0x15U)
#define GCLK_SERCOM2_CLKID          (0x16U)
#define GCLK_SERCOM3_CLKID          (0x17U)

#define SERIAL0_BASE_ADDR 0x42000800UL
#define SERIAL0 ((uartRegisterMap *)SERIAL0_BASE_ADDR)


/****************************************************/
//Generic clock register structure
typedef union {
	struct {
		uint8_t  swrst:1;          /*!< bit:      0  Software Reset                     */
		uint8_t  :7;               /*!< bit:  1.. 7  Reserved                           */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} GCLK_CTRL_register;

typedef union {
	struct {
		uint8_t  :7;               /*!< bit:  0.. 6  Reserved                           */
		uint8_t  syncbusy:1;       /*!< bit:      7  Synchronization Busy Status        */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} GCLK_STATUS_register;

typedef union {
	struct {
		uint16_t id:6;             /*!< bit:  0.. 5  Generic Clock Selection ID         */
		uint16_t :2;               /*!< bit:  6.. 7  Reserved                           */
		uint16_t gen:4;            /*!< bit:  8..11  Generic Clock Generator            */
		uint16_t :2;               /*!< bit: 12..13  Reserved                           */
		uint16_t clken:1;          /*!< bit:     14  Clock Enable                       */
		uint16_t writeLock:1;        /*!< bit:     15  Write Lock                         */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint16_t reg_value;                /*!< Type      used for register access              */
} GCLK_CLKCTRL_register;

typedef union {
	struct {
		uint32_t id:4;             /*!< bit:  0.. 3  Generic Clock Generator Selection  */
		uint32_t :4;               /*!< bit:  4.. 7  Reserved                           */
		uint32_t src:5;            /*!< bit:  8..12  Source Select                      */
		uint32_t :3;               /*!< bit: 13..15  Reserved                           */
		uint32_t genen:1;          /*!< bit:     16  Generic Clock Generator Enable     */
		uint32_t idc:1;            /*!< bit:     17  Improve Duty Cycle                 */
		uint32_t oov:1;            /*!< bit:     18  Output Off Value                   */
		uint32_t oe:1;             /*!< bit:     19  Output Enable                      */
		uint32_t divsel:1;         /*!< bit:     20  Divide Selection                   */
		uint32_t rstdby:1;       /*!< bit:     21  Run in Standby                     */
		uint32_t :10;              /*!< bit: 22..31  Reserved                           */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint32_t reg_value;                /*!< Type      used for register access              */
} GCLK_GENCTRL_register;

typedef union {
	struct {
		uint32_t id:4;             /*!< bit:  0.. 3  Generic Clock Generator Selection  */
		uint32_t :4;               /*!< bit:  4.. 7  Reserved                           */
		uint32_t divf:16;           /*!< bit:  8..23  Division Factor                    */
		uint32_t :8;               /*!< bit: 24..31  Reserved                           */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint32_t reg_value;                /*!< Type      used for register access              */
} GCLK_GENDIV_register;

typedef struct {
	__IO GCLK_CTRL_register            CTRL;        /**< \brief Offset: 0x0 (R/W  8) Control */
	__I  GCLK_STATUS_register          STATUS;      /**< \brief Offset: 0x1 (R/   8) Status */
	__IO GCLK_CLKCTRL_register         CLKCTRL;     /**< \brief Offset: 0x2 (R/W 16) Generic Clock Control */
	__IO GCLK_GENCTRL_register         GENCTRL;     /**< \brief Offset: 0x4 (R/W 32) Generic Clock Generator Control */
	__IO GCLK_GENDIV_register          GENDIV;      /**< \brief Offset: 0x8 (R/W 32) Generic Clock Generator Division */
} GenClockRegMap;

/****************************************************/
//unique register structure for each type of register

//REG_SERCOM0_USART_CTRLA    (0x42000800U)
typedef union {
	struct
	{//right to left order:
		uint32_t swrst:1;    //bit 0 - set to 1 to rst all reg in sercom
		uint32_t enable:1;   //bit 1 - enable peripheral
		uint32_t mode:3;     //bit 2-4 - 0-sync mode, ext clock, 1-async, internal clock
		uint32_t res1:2;     //bit 5-6 - reserved
		uint32_t runstdby:1; //bit 7 - 1-interupts wake up the device
		uint32_t ibon:1;     //bit 8 - flag indicating buf overflow
		uint32_t res2:4;     //bit 9-12 - reserved
		uint32_t sampr:3;    //bit 13-15 - sample rate
		uint32_t txpo:2;     //bit 16-17 - set tx pin
		uint32_t res3:2;     //bit 18-19 - reserved
		uint32_t rxpo:2;     //bit 20-21 - set rx pin
		uint32_t sampa:2;    //bit 22-23 - sample adjustment
		uint32_t form:4;     //bit 24-27 - set usart frame format
		uint32_t cmode:1;    //bit 28 - communication mode(async or sync)
		uint32_t cpol:1;     //bit 29 - clock polarity
		uint32_t dord:1;     //bit 30 - data order(msb or lsb)
		uint32_t res4:1;     //bit 31 - reserved
	} bit_data;
	uint32_t reg_value;
} CTRLA_register;

//REG_SERCOM0_USART_CTRLB    (0x42000804U)
typedef union {
	struct {
		uint32_t chsize:3;         /*!< bit:  0.. 2  Character Size                     */
		uint32_t rsvd1:3;               /*!< bit:  3.. 5  Reserved                           */
		uint32_t sbmode:1;         /*!< bit:      6  Stop Bit Mode                      */
		uint32_t rsvd2:1;               /*!< bit:      7  Reserved                           */
		uint32_t colden:1;         /*!< bit:      8  Collision Detection Enable         */
		uint32_t sfde:1;           /*!< bit:      9  Start of Frame Detection Enable    */
		uint32_t enc:1;            /*!< bit:     10  Encoding Format, used with IRDA                    */
		uint32_t rsvd3:2;               /*!< bit: 11..12  Reserved                           */
		uint32_t pmode:1;          /*!< bit:     13  Parity Mode                        */
		uint32_t rsvd4:2;               /*!< bit: 14..15  Reserved                           */
		uint32_t txen:1;           /*!< bit:     16  Transmitter Enable                 */
		uint32_t rxen:1;           /*!< bit:     17  Receiver Enable                    */
		uint32_t rsvd5:14;              /*!< bit: 18..31  Reserved                           */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint32_t reg_value;                /*!< Type      used for register access              */
} CTRLB_register;

//REG_SERCOM0_USART_BAUD     (0x4200080CU)
typedef union {
	struct {
		uint16_t BAUD:16;          /*!< bit:  0..15  Baud Rate Value                    */
	} bit;                       /*!< Structure used for bit  access                  */
	struct { // FRAC mode
		uint16_t BAUD:13;          /*!< bit:  0..12  Baud Rate Value                    */
		uint16_t FP:3;             /*!< bit: 13..15  Fractional Part                    */
	} FRAC;                      /*!< Structure used for FRAC                         */
	struct { // FRACFP mode
		uint16_t BAUD:13;          /*!< bit:  0..12  Baud Rate Value                    */
		uint16_t FP:3;             /*!< bit: 13..15  Fractional Part                    */
	} FRACFP;                    /*!< Structure used for FRACFP                       */
	struct { // USARTFP mode
		uint16_t BAUD:16;          /*!< bit:  0..15  Baud Rate Value                    */
	} USARTFP;                   /*!< Structure used for USARTFP                      */
	uint16_t reg_value;                /*!< Type      used for register access              */
} BAUD_register;

//REG_SERCOM0_USART_RXPL     (0x4200080EU)
typedef union {
	struct {
		uint8_t  rxpl:8;           /*!< bit:  0.. 7  Receive Pulse Length               */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} RXPL_register;

//REG_SERCOM0_USART_INTENCLR (0x42000814U)
typedef union {
	struct {
		uint8_t  dre:1;            /*!< bit:      0  Data Register Empty Interrupt Disable */
		uint8_t  txc:1;            /*!< bit:      1  Transmit Complete Interrupt Disable */
		uint8_t  rxc:1;            /*!< bit:      2  Receive Complete Interrupt Disable */
		uint8_t  rxs:1;            /*!< bit:      3  Receive Start Interrupt Disable    */
		uint8_t  ctsic:1;          /*!< bit:      4  Clear To Send Input Change Interrupt Disable */
		uint8_t  rxbrk:1;          /*!< bit:      5  Break Received Interrupt Disable   */
		uint8_t  rsvd1:1;               /*!< bit:      6  Reserved                           */
		uint8_t  err:1;          /*!< bit:      7  Combined Error Interrupt Disable   */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} INTENCLR_register;

//REG_SERCOM0_USART_INTENSET (0x42000816U)
typedef union {
	struct {
		uint8_t  dre:1;           /*!< bit:      0  Data Register Empty Interrupt Enable */
		uint8_t  txc:1;           /*!< bit:      1  Transmit Complete Interrupt Enable */
		uint8_t  rxc:1;           /*!< bit:      2  Receive Complete Interrupt Enable  */
		uint8_t  rxs:1;           /*!< bit:      3  Receive Start Interrupt Enable     */
		uint8_t  ctsic:1;         /*!< bit:      4  Clear To Send Input Change Interrupt Enable */
		uint8_t  rxbrk:1;         /*!< bit:      5  Break Received Interrupt Enable    */
		uint8_t  rsvd1:1;              /*!< bit:      6  Reserved                           */
		uint8_t  err:1;           /*!< bit:      7  Combined Error Interrupt Enable    */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} INTENSET_register;

//REG_SERCOM0_USART_INTFLAG  (0x42000818U)
typedef union { // __I to avoid read-modify-write on write-to-clear register
	struct {
		__I uint8_t  dre:1;            /*!< bit:      0  Data Register Empty Interrupt      */
		__I uint8_t  txc:1;            /*!< bit:      1  Transmit Complete Interrupt        */
		__I uint8_t  rxc:1;            /*!< bit:      2  Receive Complete Interrupt         */
		__I uint8_t  rxs:1;            /*!< bit:      3  Receive Start Interrupt            */
		__I uint8_t  ctsic:1;          /*!< bit:      4  Clear To Send Input Change Interrupt */
		__I uint8_t  rxbrk:1;          /*!< bit:      5  Break Received Interrupt           */
		__I uint8_t  rsvd1:1;               /*!< bit:      6  Reserved                           */
		__I uint8_t  err:1;          /*!< bit:      7  Combined Error Interrupt           */
	} bit_data;                       /*!< Structure used for bit  access                  */
	uint8_t reg_value;                 /*!< Type      used for register access              */
} INTFLAG_register;

//REG_SERCOM0_USART_STATUS   (0x4200081AU)
typedef union {
	struct {
		uint16_t perr:1;           /*!< bit:      0  Parity Error                       */
		uint16_t ferr:1;           /*!< bit:      1  Frame Error                        */
		uint16_t bufovf:1;         /*!< bit:      2  Buffer Overflow                    */
		uint16_t cts:1;            /*!< bit:      3  Clear To Send                      */
		uint16_t isf:1;            /*!< bit:      4  Inconsistent Sync Field            */
		uint16_t coll:1;           /*!< bit:      5  Collision Detected                 */
		uint16_t rsvd1:10;              /*!< bit:  6..15  Reserved                           */
	} bit_data;                       /*!< Structure used for bit access                   */
	uint16_t reg_value;                /*!< Type used for register access                   */
} STATUS_register;


//REG_SERCOM0_USART_SYNCBUSY (0x4200081CU)
typedef union {
	struct {
		uint32_t swrst:1;          /*!< bit:      0  Software Reset Synchronization Busy */
		uint32_t enable:1;         /*!< bit:      1  SERCOM Enable Synchronization Busy */
		uint32_t ctrlb:1;          /*!< bit:      2  CTRLB Synchronization Busy         */
		uint32_t rsvd1:29;              /*!< bit:  3..31  Reserved                           */
	} bit_data;                       /*!< Structure used for bit access                   */
	uint32_t reg_value;                /*!< Type used for register access                   */
} SYNCBUSY_register;

//REG_SERCOM0_USART_DATA     (0x42000828U)
typedef union {
	struct {
		uint16_t data:9;           /*!< bit:  0.. 8  Data Value                         */
		uint16_t rsvd1:7;               /*!< bit:  9..15  Reserved                           */
	} bit_data;                       /*!< Structure used for bit access                   */
	uint16_t reg_value;                /*!< Type used for register access                   */
} DATA_register;

//REG_SERCOM0_USART_DBGCTRL  (0x42000830U)
typedef union {
	struct {
		uint8_t  dbgstop:1;        /*!< bit:      0  Debug Mode                         */
		uint8_t  rsvd1:7;               /*!< bit:  1.. 7  Reserved                           */
	} bit_data;                       /*!< Structure used for bit access                   */
	uint8_t reg_value;                 /*!< Type used for register access                   */
} DBGCTRL_register;


/****************************************************/
//structure containing all the registers for sercom0

typedef struct{
	__IO CTRLA_register CTRLA;
	__IO CTRLB_register CTRLB;
	volatile uint8_t reserved1[4];
	__IO BAUD_register  BAUD;
	__IO RXPL_register  RXPL;
	volatile uint8_t reserved2[5];
	__IO INTENCLR_register INTENCLR;
	volatile uint8_t reserved3[1];
	__IO INTENSET_register INTENSET;
	volatile uint8_t reserved4[1];
	__IO INTFLAG_register  INTFLAG;
	volatile uint8_t reserved5[1];
	__IO STATUS_register STATUS;
	__IO SYNCBUSY_register SYNCBUSY;
	volatile uint8_t reserved6[8];
	__IO DATA_register	DATA;
	volatile uint8_t reserved7[6];
	__IO DBGCTRL_register DBGCTRL;
										
}uartRegisterMap;



#endif /* CUSTOM_UART_REG_DEF_H_ */