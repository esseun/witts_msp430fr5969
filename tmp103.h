/*
	tmp103.h

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

#ifndef TMP103_H_
#define TMP103_H_

void temp_i2c_init(void);
void temp_read(void);

extern signed char t_Fahrenheit;
extern signed char t_Celcius;

#define TMP103_I2C_ADDR 	0x0070
/* TMP103 Register Addresses */
#define TEMP_REG		0x00
#define CONF_REG 		0x01
#define TLOW_REG		0x02
#define THIGH_REG 		0x03

#endif /* TMP103_H_ */
