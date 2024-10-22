/*
 * myMax7219.c
 *
 *  Created on: Oct 30, 2019
 *      Author: Andi
 */

#include "myMax7219.h"

/*
 * my variable
 */


/*
 * shift out a byte
 */
void max7219_shiftout(uint8_t bytedata) {
	uint8_t i = 0;

	for(i=0; i<8; i++){
		Ser_Data((bytedata<<i) & 0x80); //the shift is made in reverse order for this ic
		Ser_Clk();						//Serial clock
	}
}

/*
 * shift out data to a selected number
 */
void max7219_send(uint8_t icnum, uint8_t reg, uint8_t data) {
	uint8_t i = 0;

	if(icnum < MAX7219_ICNUMBER) {
		//send no op to following ic
		for(i=icnum; i<(MAX7219_ICNUMBER-1); i++) {
			max7219_shiftout(MAX7218_REGNOOP); //no op reg
			max7219_shiftout(MAX7218_REGNOOP); //no op data
		}

		//send info to current ic
		max7219_shiftout(reg); //send reg
		max7219_shiftout(data); //send data

		//send no op to previous ic
		for(i=0; i<icnum; i++) {
			max7219_shiftout(MAX7218_REGNOOP); //no op reg
			max7219_shiftout(MAX7218_REGNOOP); //no op data
		}
		Ser_Load(); //load up
	}
}


/*
 * set shutdown for a selected ic
 */
void max7219_shutdown(uint8_t icnum, uint8_t value) {
	if(value == 0 || value == 1)
		max7219_send(icnum, MAX7218_REGSHUTDOWN, value);
}


/*
 * set brightness for a selected ic
 */
void max7219_intensity(uint8_t icnum, uint8_t value) {
	if(value < 16)
		max7219_send(icnum, MAX7218_REGINTENSITY, value);
}


/*
 * set test mode for a selected ic
 */
void max7219_test(uint8_t icnum, uint8_t value) {
	if(value == 0 || value == 1)
		max7219_send(icnum, MAX7218_REGTEST, value);
}


/*
 * set active output for a selected ic
 */
void max7219_scanlimit(uint8_t icnum, uint8_t value) {
	if(value < 8)
		max7219_send(icnum, MAX7218_REGSCANLIMIT, value);
}


/*
 * set decode mode for a selected ic
 */
void max7219_decode(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDECODE, value);
}

/*
 * set output 0 for a selected ic
 */
void max7219_digit0(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT0, value);
}


/*
 * set output 1 for a selected ic
 */
void max7219_digit1(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT1, value);
}


/*
 * set output 2 for a selected ic
 */
void max7219_digit2(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT2, value);
}


/*
 * set output 3 for a selected ic
 */
void max7219_digit3(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT3, value);
}

/*
 * set output 4 for a selected ic
 */
void max7219_digit4(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT4, value);
}


/*
 * set ouput 5 for a selected ic
 */
void max7219_digit5(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT5, value);
}

/*
 * set output 6 for a selected ic
 */
void max7219_digit6(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT6, value);
}


/*
 * set output 7 for a selected ic
 */
void max7219_digit7(uint8_t icnum, uint8_t value) {
	max7219_send(icnum, MAX7218_REGDIGIT7, value);
}


/*
 * set output digit for a selected ic
 */
void max7219_digit(uint8_t icnum, uint8_t digit, uint8_t value) {
	switch(digit) {
		case 0:
			max7219_digit0(icnum, value);
			break;
		case 1:
			max7219_digit1(icnum, value);
			break;
		case 2:
			max7219_digit2(icnum, value);
			break;
		case 3:
			max7219_digit3(icnum, value);
			break;
		case 4:
			max7219_digit4(icnum, value);
			break;
		case 5:
			max7219_digit5(icnum, value);
			break;
		case 6:
			max7219_digit6(icnum, value);
			break;
		case 7:
			max7219_digit7(icnum, value);
			break;
	}
}


