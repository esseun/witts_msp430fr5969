/*
	rf430.c

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

#include "msp430.h"
#include "rf430.h"

unsigned char RxData[2] = {0,0};
unsigned char TxData[2] = {0,0};
unsigned char TxAddr[2] = {0,0};

void rf430_i2c_init(){
	/* configure eUSCI for I2C */

	/* Software reset enabled */
	UCB0CTL1 |= UCSWRST;
	/* I2C mode, Master mode, sync, transmitter */
	UCB0CTLW0 |= UCMODE_3  + UCMST + UCSYNC + UCTR;
	/* SMCLK = 8MHz */
	UCB0CTLW0 |= UCSSEL_2;
	/* Baudrate = SMLK/40 = 200kHz */
	UCB0BRW = 30;

	/* slave address - determined by pins E0, E1, and E2 on the RF430CL330H */
	UCB0I2CSA  = 0x0028;
	UCB0CTL1  &= ~UCSWRST;
}

void update_rf430(void)
{
	rf430_i2c_init();

	decToAscii(&FRAM_Message[34], t_Fahrenheit);
    /* Modify RF430_DEFAULT_DATA[] to include celcius reading */
	/* decToAscii(&FRAM_Message[64], t_Celcius); */

	write_continuous(26, &FRAM_Message[26], 45);

	/* enable RF */
	write_register(CONTROL_REG, RF_ENABLE);
}

void decToAscii(unsigned char* Ascii, signed char Decimal){
	char Neg = 0;
	unsigned char Hunds, Tens, Ones;

	if (Decimal < 0) {
		Neg = 1;
		Decimal = ~Decimal + 1;
	}
	Ones = (Decimal % 10);
	Tens = ((Decimal / 10) % 10);
	Hunds = ((Decimal / 100) % 10);

	if (Hunds == 1) {
		*Ascii++ = Hunds | 0x30;
	} else if (Neg == 1) {
		*Ascii++ = '-';
	} else {
		*Ascii++ = ' ';
	}

	*Ascii++ = Tens | 0x30;
	*Ascii   = Ones | 0x30;
}

unsigned int read_register(unsigned int reg_addr)
{
	/* msb */
	TxAddr[0] = reg_addr >> 8;
	/* lsb */
	TxAddr[1] = reg_addr & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0002;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];
	while (!(UCB0IFG & UCBCNTIFG));

	/* i2c read */
	UCB0CTL1 &= ~UCTR;
	UCB0CTL1 |= UCTXSTT;
	while (!(UCB0IFG & UCRXIFG0));
	RxData[0] = UCB0RXBUF;
	UCB0CTLW0 |= UCTXSTP;
	while (!(UCB0IFG & UCRXIFG0));
	RxData[1] = UCB0RXBUF;
	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;

	return RxData[1] << 8 | RxData[0];
}

unsigned int read_register_BIP8(unsigned int reg_addr)
{
	unsigned char BIP8 = 0;

	/* msb */
	TxAddr[0] = reg_addr >> 8;
	/* lsb */
	TxAddr[1] = reg_addr & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0002;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	BIP8 ^= TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];
	BIP8 ^= TxAddr[1];
	while (!(UCB0IFG & UCBCNTIFG));

	/* i2c read */
	UCB0CTL1 &= ~UCTR;
	UCB0CTL1 |= UCTXSTT;
	while (!(UCB0IFG & UCRXIFG0));
	RxData[0] = UCB0RXBUF;
	BIP8 ^= RxData[0];
	while (!(UCB0IFG & UCRXIFG0));
	RxData[1] = UCB0RXBUF;
	BIP8 ^= RxData[1];
	UCB0CTLW0 |= UCTXSTP;
	while (!(UCB0IFG & UCRXIFG0));

	/* break if BIP8 doesn't match */
	if (BIP8 != UCB0RXBUF) {
		__no_operation();
	}

	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;

	return RxData[0] << 8 | RxData[1];
}

void read_continuous(unsigned int reg_addr, unsigned char* read_data, unsigned int data_length)
{
	unsigned int i;

	/* msb */
	TxAddr[0] = reg_addr >> 8;
	/* lsb */
	TxAddr[1] = reg_addr & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0002;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];
	while (!(UCB0IFG & UCBCNTIFG));

	/* i2c read */
	UCB0CTL1 &= ~UCTR;
	UCB0CTL1 |= UCTXSTT;
	while (!(UCB0IFG & UCRXIFG0));

	for(i = 0; i < data_length-1; i++)
	{
		while (!(UCB0IFG & UCRXIFG0));
		read_data[i] = UCB0RXBUF;
		if (i == data_length-1) {
			UCB0CTL1 |= UCTXSTP;
		}
	}

	UCB0CTLW0 |= UCTXSTP;
	while (!(UCB0IFG & UCRXIFG0));
	read_data[i] = UCB0RXBUF;

	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;
}

void write_register(unsigned int reg_addr, unsigned int value)
{
	TxAddr[0] = reg_addr >> 8;
	TxAddr[1] = reg_addr & 0xFF;
	TxData[0] = value >> 8;
	TxData[1] = value & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0004;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];

	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxData[1];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxData[0];
	while (!(UCB0IFG & UCBCNTIFG));
	UCB0CTL1 |= UCTXSTP;
	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;

}

void write_register_BIP8(unsigned int reg_addr, unsigned int value)
{
	unsigned char BIP8 = 0;

	TxAddr[0] = reg_addr >> 8;
	TxAddr[1] = reg_addr & 0xFF;
	TxData[0] = value >> 8;
	TxData[1] = value & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = 0x0005;
	UCB0CTL1  &= ~UCSWRST;

	/* start ic2 write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	BIP8 ^= TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];
	BIP8 ^= TxAddr[1];

	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxData[0];
	BIP8 ^= TxData[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxData[1];
	BIP8 ^= TxData[1];

	/* send BIP8 byte */
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = BIP8;

	while (!(UCB0IFG & UCBCNTIFG));
	UCB0CTL1 |= UCTXSTP;
	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;

}

void write_continuous(unsigned int reg_addr, unsigned char* write_data, unsigned int data_length)
{
	unsigned int i;

	TxAddr[0] = reg_addr >> 8;
	TxAddr[1] = reg_addr & 0xFF;

	UCB0CTLW1 = UCASTP_1;
	UCB0TBCNT = data_length;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[0];
	while (!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TxAddr[1];

	for(i = 0; i < data_length; i++)
	{
		while (!(UCB0IFG & UCTXIFG0));
		UCB0TXBUF = write_data[i];
	}

	while (!(UCB0IFG & UCTXIFG0));
	while (!(UCB0IFG & UCBCNTIFG));
	UCB0CTL1 |= UCTXSTP;
	while (!(UCB0IFG & UCSTPIFG));
	UCB0CTL1  |= UCSWRST;

}
