/*
	main.c

	IMPORTANT: Contents of the following file are a result of modifications made
	to the sample temperature software provided with the Texas Instruments
	MSP430FR5969. The resulting software is to be embedded on the Texas Instruments
	TIDA-00217 Field Powered NFC and Microcontroller Reference Design Board for the
	purpose of implementing WiTTS (Wireless Temperature Tracking System) as part of
	the Accelerated Masters in Software Engineering graduate project proposal at
	the California State University Fullerton and will not be reproduced, copied,
	modified, distributed, performed, displayed or sold for any other purpose or
	used with any other non-Texas Instruments hardware.

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
#include "tmp103.h"
#include "stdio.h"

void timer_init();
void low_power_delay_ms(unsigned int ms);
void msp430_init();

unsigned char into_fired = 0;

const unsigned char RF430_DEFAULT_DATA[] = {                            \
    /* NDEF Tag Application Name */                                     \
    0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,                           \
                                                                        \
    /* Capability Container ID */                                       \
    0xE1, 0x03,                                                         \
    0x00  0x0F, /* CCLEN */                                             \
    0x20,       /* Mapping version 2.0 */                               \
    0x00, 0x3B, /* MLe (49 bytes); Maximum R-APDU data size */          \
    0x00, 0x34, /* MLc (52 bytes); Maximum C-APDU data size */          \
    0x04,       /* Tag, File Control TLV (4 = NDEF file) */             \
    0x06,       /* Length, File Control TLV (6 = 6 bytes of datag) */   \
    0xE1, 0x04, /* File Identifier */                                   \
    0x0B, 0xDF, /* Max NDEF size (3037 bytes of useable memory) */      \
    0x00,       /* NDEF file read access condition, no security */      \
    0x00,       /* NDEF file write access condition, no security */     \
                                                                        \
    /* NDEF File ID */                                                  \
    0xE1, 0x04,                                                         \
                                                                        \
    /* NDEF File for Hello World  (48 bytes total length) */            \
    0x00, 0x2A, /* 0x39 NLEN; NDEF length (3 byte long message) */      \
    0xD1, 0x01, 0x26,  /* Last byte has to be changed to NLEN - 4 */    \
    0x54, /* T = text */                                                \
    0x02,                                                               \
    0x65, 0x6E, /* 'e', 'n', */                                         \
                                                                        \
    /* '80' */                                                          \
    0x38, 0x30                                                          \
};

extern unsigned char FRAM_Message[256]; // FRAM container for holding the NDEF Message
unsigned char read_data[100];


void main (void)
{
	volatile unsigned int test = 0;
	volatile unsigned int flags = 0;

	msp430_init();

	low_power_delay_ms(20);

	while (!(read_register(STATUS_REG) & READY)) {
		low_power_delay_ms(1);
	}

	/****************************************************************************/
    /* Errata Fix : Unresponsive RF - recommended firmware                      */
    /****************************************************************************/
	/*
	 * Versions C and D (0x0101, 0x0201) have this issue.
	 * Read the version register and apply the fix accordingly.
	 */
	{
		unsigned int version;
		version = read_register(VERSION_REG);

		if (version == 0x0101 || version == 0x0201) {
			write_register(0xFFE0, 0x004E);
			write_register(0xFFFE, 0x0080);
			if (version == 0x0101) {
				// ver. C
				write_register(0x2a98, 0x0650);
			} else {
				// ver. D
				write_register(0x2a6e, 0x0650);
			}
			write_register(0x2814, 0);
			write_register(0xFFE0, 0);
		}
		/* Upon exit of this block, the control register is set to 0x0 */
	}

	/* check if content is valid and set to default data if it is not */
	if(FRAM_Message[0] != 0xD2)
	{
		memcpy(FRAM_Message, RF430_DEFAULT_DATA , 37);
	}

    while (1)
    {
    	low_power_delay_ms(1);
    	temp_read();
    	update_rf430();
    	low_power_delay_ms(3000);

    	/* RF430CL330H reset */
		PORT_RST_OUT &= ~RST;
		low_power_delay_ms(1);
		/* release */
		PORT_RST_OUT |= RST;

		low_power_delay_ms(20);
		while(!(read_register(STATUS_REG) & READY));
    }
}

void low_power_delay_ms(unsigned int ms){
	TA1CTL |= TACLR;

	/* initialize RF430CL33H within delay */
	TA1CCR0 = ms*10;
	TA1CTL |= MC__UP;
	__bis_SR_register(LPM3_bits + GIE);
	TA1CTL |= MC__STOP;
}

void msp430_init(){
	/* turn off watch-dog */
	WDTCTL = WDTPW + WDTHOLD;
	PMMCTL0 = PMMPW;
	/* clear locked i/o pins */
	PM5CTL0 &= ~LOCKLPM5;

	CSCTL0_H = 0xA5;
	CSCTL1 |= DCOFSEL0 + DCOFSEL1;          // Set max. DCO setting = 8MHz
	CSCTL2 = SELA_1 + SELS_3 + SELM_3;      // set ACLK = VLOCLK = 10kHz
	CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;      // set VLOCLK / 1

	//*** Drive unused pins low, for lowest power  ***//
	P1DIR = 0xFF;
	P2DIR = 0xFF;
	P3DIR = 0xFF;
	P4DIR = 0xFF;

	P1OUT = 0x00;
	P2OUT = 0x00;
	P3OUT = 0x00;
	P4OUT = 0x00;
	//************************************************//

	rf430_i2c_init();

	/* configure pins for i2c */
	PORT_I2C_SEL0 &= ~(SCL + SDA);
	PORT_I2C_SEL1 |= SCL + SDA;

	rf430_i2c_init();

	/* DRIVE CS Low for i2c mode */
	PORT_CS_SEL0 |= PORT_CS_PIN;
	PORT_CS_DIR |= PORT_CS_PIN;
	PORT_CS &= ~PORT_CS_PIN;

	/* reset rf430 */
	PORT_RST_SEL0 &= ~RST;
	PORT_RST_SEL1 &= ~RST;
	PORT_RST_OUT &= ~RST;
	PORT_RST_DIR |= RST;
	__delay_cycles(100);
	PORT_RST_OUT |= RST;

	/* DRIVE CS Low for i2c mode */
	PORT_CS_SEL0 |= PORT_CS_PIN;
	PORT_CS_DIR |= PORT_CS_PIN;
	PORT_CS &= ~PORT_CS_PIN;

	/* configure pin for INTO interrupts */
	PORT_INTO_SEL0 &= ~INTO;
	PORT_INTO_SEL1 &= ~INTO;
	PORT_INTO_DIR &= ~INTO;
	PORT_INTO_OUT |= INTO;
	PORT_INTO_REN |= INTO;
	PORT_INTO_IFG &= ~INTO;
	PORT_INTO_IES |= INTO;

	timer_init();
}

/* ISR */
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	/* INTO interrupt fired */
	if(PORT_INTO_IFG & INTO)
	{
		into_fired = 1;

		/* disable INTO and clear interrupt flag */
		PORT_INTO_IE &= ~INTO;
		PORT_INTO_IFG &= ~INTO;

		/* wake up to handle INTO */
		__bic_SR_register_on_exit(LPM3_bits);
	}
}

void timer_init(){
	TA1CCTL0 = CCIE;
	TA1CCR0 = 20000;
	TA1CTL = TASSEL_1;
}

/* Timer A1 interrupt service routine */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
	/* stop timer */
    TA1CTL &= ~(MC_3);
	LPM3_EXIT;
}


