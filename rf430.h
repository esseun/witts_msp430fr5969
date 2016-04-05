/*
	rf430.h

	IMPORTANT: Contents of the following file are a result of modifications made
	to the sample temperature software provided with the Texas Instruments
	MSP430FR5969. The resulting software is to be embedded on the Texas Instruments
	TIDA-00217 Field Powered NFC and Microcontroller Reference Design Board for the
	purpose of implementing WiTTS (Wireless Temperature Tracking System) as part of
	the Accelerated Masters in Software Engineering graduate project proposal at
	the California State University Fullerton. The software will not be reproduced,
	copied,	modified, distributed, performed, displayed or sold for any other purpose
	or used with any other non-Texas Instruments hardware.

	YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
	PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
	INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
	NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE
	AUTHOR OR TEXAS INSTRUMENTS BE LIABLE OR OBLIGATED UNDER CONTRACT,
	NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL
	EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT
	LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL
	DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,
	TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
	LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/

#ifndef RF430_EXAMPLE_H_
#define RF430_EXAMPLE_H_

void rf430_i2c_init(void);
unsigned int read_register(unsigned int reg_addr);
unsigned int read_register_BIP8(unsigned int reg_addr);
void read_continuous(unsigned int reg_addr, unsigned char* read_data, unsigned int data_length);

void write_register(unsigned int reg_addr, unsigned int value);
void write_continuous(unsigned int reg_addr, unsigned char* write_data, unsigned int data_length);
void write_register_BIP8(unsigned int reg_addr, unsigned int value);

void update_rf430(void);
void decToAscii(unsigned char* Ascii, signed char Decimal);

unsigned char FRAM_Message[256]; // FRAM container for holding the NDEF Message

signed char t_Fahrenheit;
signed char t_Celcius;

/* MSP-EXP430FR5969 port definitions */
/* I2C */
#define PORT_I2C_OUT	P1OUT
#define PORT_I2C_DIR	P1DIR
#define PORT_I2C_SEL0	P1SEL0
#define PORT_I2C_SEL1	P1SEL1
#define SDA	BIT6
#define SCL BIT7

/* CS */
#define PORT_CS 		P3OUT
#define PORT_CS_SEL0	P3SEL0
#define PORT_CS_DIR		P3DIR
#define PORT_CS_PIN		BIT4


/* INTO */
#define PORT_INTO_IN	P2IN
#define PORT_INTO_OUT	P2OUT
#define PORT_INTO_DIR	P2DIR
#define PORT_INTO_SEL0	P2SEL0
#define PORT_INTO_SEL1	P2SEL1
#define PORT_INTO_REN	P2REN
#define PORT_INTO_IE	P2IE
#define PORT_INTO_IES	P2IES
#define PORT_INTO_IFG	P2IFG
#define INTO	BIT2

/* RST */
#define PORT_RST_OUT	P4OUT
#define PORT_RST_DIR	P4DIR
#define PORT_RST_SEL0	P4SEL0
#define PORT_RST_SEL1	P4SEL1
#define RST	BIT4

#define CONTROL_REG 		0xFFFE
#define STATUS_REG			0xFFFC
#define VERSION_REG			0xFFEE

/* control register bits */
#define SW_RESET		BIT0
#define RF_ENABLE		BIT1
#define INT_ENABLE		BIT2
#define INTO_HIGH		BIT3
#define INTO_DRIVE		BIT4
#define BIP8_ENABLE		BIT5
#define STANDBY_ENABLE	BIT6
#define TEST430_ENABLE	BIT7

/* status register bits */
#define READY			BIT0
#define CRC_ACTIVE		BIT1
#define RF_BUSY			BIT2


#endif /* RF430_EXAMPLE_H_ */
