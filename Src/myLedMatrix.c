/*
 * myLedMatrix.c
 *
 *  Created on: Oct 23, 2019
 *      Author: Andi
 */

#include "myLedMatrix.h"

void myLedMatrix_init(void){
	//max7219_init();
	uint8_t ledmatrix = 0;
	for(ledmatrix=0; ledmatrix<LEDMATRIX7219_MAXLEDMATRIX; ledmatrix++) {
		uint8_t j = 0;
		for(j=0; j<0; j++)
			myLedMatrix_data[ledmatrix][j]=0;	//set columns to zero
		max7219_shutdown(ledmatrix, 1);
		max7219_test(ledmatrix, 0);
		max7219_decode(ledmatrix, 0);
		max7219_intensity(ledmatrix, 15);
		max7219_scanlimit(ledmatrix, 7);
		myLedMatrix_reset(ledmatrix);
	}
}

void myLedMatrix_reset(uint8_t ledmatrix){
	uint8_t j = 0;
	for(j=0; j<8; j++) {
		myLedMatrix_data[ledmatrix][j] = 0; //set columns to zero
		max7219_digit(ledmatrix, j, 0); //reset display
	}
}

void myLedMatrix_setmatrix(uint8_t ledmatrix, uint8_t rows[8]){
	uint8_t j = 0;
	if(ledmatrix < LEDMATRIX7219_MAXLEDMATRIX) {
		for(j=0; j<8; j++) {
			myLedMatrix_data[ledmatrix][j] = rows[j];
			max7219_digit(ledmatrix, j, rows[j]);
		}
	}
}

void myLedMatrix_setrow(uint8_t ledmatrix, uint8_t col, uint8_t row){
	if(ledmatrix < LEDMATRIX7219_MAXLEDMATRIX && col < 8 && row < (1<<8)) {
		myLedMatrix_data[ledmatrix][col] = row;
		max7219_digit(ledmatrix, col, row);
	}
}
void myLedMatrix_setled(uint8_t ledmatrix, uint8_t lednum, uint8_t status){
	if(lednum < 64) {
		uint8_t col = lednum%8;
		uint8_t row = myLedMatrix_data[ledmatrix][col];
		if(status)
			row |= (1<<(lednum/8));
		else
			row &= ~(1<<(lednum/8));

		myLedMatrix_setrow(ledmatrix, col, row);
	}
}
void myLedMatrix_setledon(uint8_t ledmatrix, uint8_t lednum){
	myLedMatrix_setled(ledmatrix, lednum, 1);
}

void myLedMatrix_setledoff(uint8_t ledmatrix, uint8_t lednum){
	myLedMatrix_setled(ledmatrix, lednum, 0);
}

void myLedMatrix_setintensity(uint8_t ledmatrix, uint8_t intensity){
	max7219_intensity(ledmatrix, intensity);
}

//void myLedMatrix_Init(void){
//	myLedMatrix_Write(0x09, 0x00, 0);	//No Decode
//	myLedMatrix_Write(0x09, 0x00, 1);	//No Decode
//	myLedMatrix_Write(0x0C, 0x01, 0);	//Normal mode
//	myLedMatrix_Write(0x0C, 0x01, 1);	//Normal mode
//	myLedMatrix_Write(0x0B, 0x07, 0);	//Scan 7 digits
//	myLedMatrix_Write(0x0B, 0x07, 1);	//Scan 7 digits
//	myLedMatrix_Write(0x0A, 0x07, 0);	//Maximum intensity
//	myLedMatrix_Write(0x0A, 0x07, 1);	//Maximum intensity
//}
//
//void myLedMatrix_Write(uint8_t addr, uint8_t data, uint8_t n){
//	uint8_t i,j;
//	switch(n){
//	case 0:{
//		for(i=0; i<8; i++){
//			Ser_Data((addr<<i) & (0x80));
//			Ser_Clk();
//		}
//
//		for(i=0; i<8; i++){
//			Ser_Data((data<<i) & (0x80));
//			Ser_Clk();
//		}
//		//Ser_Load();
//	}break;
//	case 1:{
//		for(i=0; i<8; i++){
//			Ser_Data((addr<<i) & (0x80));
//			Ser_Clk();
//		}
//
//		for(i=0; i<8; i++){
//			Ser_Data((data<<i) & (0x80));
//			Ser_Clk();
//		}
//		Ser_Load();
//	}break;
//	}
//
//}
