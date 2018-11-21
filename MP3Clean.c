#include <lpc17xx_gpio.h>
#include <lpc17xx_systick.h>
#include "lpc17xx_uart.h"		// Central include files
#include "lpc17xx_pinsel.h"
#include "lpc_types.h"
#include "serial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lpc17xx_i2c.h"
#include <limits.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <math.h>

int systickcounter = 0;


int write_usb_serial_blocking(char *buf,int length);
void serial_init(void);
void LCD_Send(uint8_t * data, int length, int delay);
uint8_t getColAddr(int col);
uint8_t scanCol(uint8_t colAddr);
int decodeRow(uint8_t row);
//void SysTick_Handler(void);
uint16_t ADC_To_Voltage(uint16_t adcOutput);

int main()
{
	SYSTICK_InternalInit(100);
	SYSTICK_IntCmd(ENABLE);

	display("Started");
	serial_init();
	I2C_init();
	LCD_init();
	ADC(1,1);
	DAC();
	display("\n\r");

	//keypad_init();
}
/*
void SysTick_Handler(void) {
	if (systickcounter % 1 == 0) {
	}
	systickcounter ++;
}
*/
void DAC() {
	DAC_Init(LPC_DAC);
}
int counterToSine(int counter,int ampl) {
	double x = counter/57.2958;
	return (sin(x)+1)*512 / ampl;
}
void ADC(int freq, int ampl) {
	ADC_Init(LPC_ADC, 200000);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_BurstCmd(LPC_ADC,ENABLE);
	uint16_t data = 0;
	delayTimer(10);
	DAC();
	int counter = 0;

	while (1) {
		data = ADC_ChannelGetData(LPC_ADC, 0);
		data = data >> 3;
		
		/*char result[50];
		sprintf(result, "%d", data);
		char outputMsg[80];
		strcpy (outputMsg,"\n\r");
		strcat(outputMsg, result);
		display(outputMsg);
	*/
		counter += 1;
		DAC_UpdateValue(LPC_DAC,counterToSine(counter,ampl));
		delayTimer(27300/freq);

		//if (counter % 1800 == 0){freq++;}
		//if (counter % 2700 == 0){ampl++;}
		
	}
}
uint16_t ADC_To_Voltage(uint16_t adcOutput){
  return adcOutput/4095 * 3;
}

void I2C_init(void)
{
	PINSEL_CFG_Type PinCfg;				// Pin configuration for UART

 	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
    PinCfg.Funcnum = 3;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 23;
	PinCfg.Funcnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 26;
	PinCfg.Funcnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	I2C_Init(LPC_I2C1, 100000);
	I2C_Cmd(LPC_I2C1, ENABLE);
}
void LCD_init() {
	LCD_Send((uint8_t[]){0x00,0x34,0x0c,0x06,0x35,0x04,0x10,0x42,0x9f,0x34,0x02}, 11, 1);
	LCD_Send((uint8_t[]){0x00, 0x01}, 2, 1);
	LCD_Send((uint8_t[]){0x00,0x80}, 2, 0);
	ClearLCD();
}

void TextToLCD(char * text) {
	uint8_t ASCIIText[16];
	ASCIIText[0] = 0x40;
	int counter;
	for (counter = 1; counter <= strlen(text)+1; counter++) {
		ASCIIText[counter] = ((int) text[counter-1]) + 128;
	}
	display(text);
	LCD_Send(ASCIIText, strlen(text)+1, 1);
}

void newLineLCD() {
	LCD_Send((uint8_t[]){0x00, 0xc0}, 2, 0);
}

void ClearLCD() {
	int counter = 81;
	uint8_t eighty[81];
	eighty[0] = 0x40;
	while (counter != 1) {
		eighty[counter] = 0xA0;
		counter--;
	}
	LCD_Send(eighty , 81, 0);

}

void LCD_Send(uint8_t * data, int length, int delay) {
	//display((uint8_t *) "Sending data...");
	I2C_M_SETUP_Type transferCfg;
	uint32_t intaddrss = 59;	
	transferCfg.sl_addr7bit = intaddrss;
	transferCfg.tx_data = data;
	transferCfg.tx_length = length;
	transferCfg.retransmissions_max = 3;
    transferCfg.rx_data = NULL;
    transferCfg.rx_length = 0;

	I2C_MasterTransferData(LPC_I2C1, &transferCfg, I2C_TRANSFER_POLLING);
	if (delay == 1) {
		delayTimer(1000000);
	}
}
void display(char * message) {
	write_usb_serial_blocking(message,(unsigned)strlen(message));
}

int write_usb_serial_blocking(char *buf,int length)
{
	return(UART_Send((LPC_UART_TypeDef *)LPC_UART0,(uint8_t *)buf,length, BLOCKING));
}
void serial_init(void)
{
	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;	
	PINSEL_CFG_Type PinCfg;				

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);

	UART_ConfigStructInit(&UARTConfigStruct);

	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	UART_Init((LPC_UART_TypeDef *)LPC_UART0, &UARTConfigStruct);		
	UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART0, &UARTFIFOConfigStruct);	
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);			
	
}

void keypad_init() {
	char * buttonChars[4][4] = {{"A", "3", "2", "1"}, {"B", "6", "5", "4"}, {"C", "9", "8", "7"},{"D", "#", "0", "*"}};

	//display(buttonChars[1][1]);
	int counter = 0;
	uint8_t data;
	uint8_t row;
	int irow;
	int icol;
	int wrapCounter = 0;
	while (1) {
		if (counter <= 3) {
			data = scanCol((uint8_t) getColAddr(counter));
			if (data != getColAddr(counter)) {
				row = data & 0x0f;
				icol = counter;
				irow = decodeRow(row);
				//display("reached");
				if (wrapCounter % 16 == 0 && wrapCounter != 0){newLineLCD();}
				TextToLCD(buttonChars[irow][3 - icol]);
				wrapCounter++;
				delayTimer(2000000);
			}
			counter ++;

		} else {
			counter = 0;
		}
	}
}

uint8_t scanCol(uint8_t colAddr) {
	uint8_t dataRecieved[1];
	uint8_t dataTransfer[1];
	dataTransfer[0] = colAddr;
	//display("\n\rRecieving data...");
	I2C_M_SETUP_Type transferCfg;
	uint8_t intaddrss = 33;
	unsigned char charToReturn = "0";
	transferCfg.sl_addr7bit = intaddrss;
	transferCfg.tx_data = dataTransfer;
	transferCfg.tx_length = 1;
	transferCfg.retransmissions_max = 0;
    transferCfg.rx_data = &dataRecieved;
    transferCfg.rx_length = 1;
	I2C_MasterTransferData(LPC_I2C1, &transferCfg, I2C_TRANSFER_POLLING);
	return(dataRecieved[0]);
}
int decodeRow(uint8_t row) {
	if (row == 0x7) {
		return 0;
	}
	if (row == 0xb) {
		return 1;
	}
	if (row == 0xd) {
		return 2;
	}
	if (row == 0xe) {
		return 3;
	}
}
uint8_t getColAddr(int col) {
	if (col == 0) {
		return 0x7f;
	}
	if (col == 1) {
		return 0xbf;
	}
	if (col == 2) {
		return 0xdf;
	}
	if (col == 3) {
		return 0xef;
	}
}

void delayTimer(int time) {
	while (time > 0) {
		time -= 1;
	}
}
