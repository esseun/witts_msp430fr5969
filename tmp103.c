/*
	tmp103.c

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
#include "tmp103.h"

signed char TemperatureData = 0;

void temp_i2c_init(void){
    /* configure eUSCI for I2C */

	/* Software reset enabled */
	UCB0CTL1 |= UCSWRST;
	/* I2C mode, Master mode, sync, transmitter */
	UCB0CTLW0 |= UCMODE_3  + UCMST + UCSYNC + UCTR;
	/* SMCLK = 8MHz */
	UCB0CTLW0 |= UCSSEL_2;
	/* Baudrate = SMLK/40 = 200kHz */
	UCB0BRW = 30;

	UCB0I2CSA  = TMP103_I2C_ADDR;
	UCB0CTL1  &= ~UCSWRST;
}

void temp_read(void){

	temp_i2c_init();

	UCB0CTL1  |= UCSWRST;
	/* generate STOP condition */
	UCB0CTLW1 = UCASTP_2;
	UCB0TBCNT = 0x0001;
	UCB0CTL1  &= ~UCSWRST;

	/* start i2c write operation */
	UCB0CTL1 |= UCTXSTT + UCTR;
	while(!(UCB0IFG & UCTXIFG0));
	UCB0TXBUF = TEMP_REG;
	while(!(UCB0IFG & UCBCNTIFG));

	UCB0CTL1 &= ~UCTR;

	UCB0CTL1 |= UCTXSTT; 			// Repeated start
	UCB0CTLW0 |= UCTXSTP; 			// Send stop after next RX byte
	while(!(UCB0IFG & UCRXIFG0));
	t_Celcius = UCB0RXBUF;
	while (!(UCB0IFG & UCSTPIFG));  // Ensure stop condition got sent
	UCB0CTL1  |= UCSWRST;

	t_Fahrenheit = (t_Celcius*9)/5.0 + 32;
}





























