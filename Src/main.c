#include <stdint.h>
#include<string.h>
#include "stm32fxx.h"
#include <stdio.h>

USART_Handle_t usart2_handle;
char msg_sent[128], msg_rec[128];
uint8_t pressed_key = 0x31;

void delay(uint16_t mult) {
	for (uint32_t i = 0; i < (mult * 16000); i++)
		;
}

void delayMs(uint32_t n) {
	uint32_t i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++)
			;
}

void Keypad_GPIO_Init(void) {
	GPIO_Handle_t keypad_gpios;

	GPIO_PClkCtrl(GPIOE, ENABLE);

	keypad_gpios.pGPIOx = GPIOE;
	keypad_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	keypad_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	keypad_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	keypad_gpios.GPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;

	/****Columns (inputs) settings****/
	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_init(&keypad_gpios);

	/****Rows (outputs) settings****/
	keypad_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	keypad_gpios.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_init(&keypad_gpios);

	keypad_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_init(&keypad_gpios);

}

uint8_t key_pad(void) {
	uint8_t keypad[4][4] = { { 0x31, 0x32, 0x33, 0x41 }, { 0x34, 0x35, 0x36,
			0x42 }, { 0x37, 0x38, 0x39, 0x43 }, { 0x2A, 0x30, 0x23, 0x44 } };

	uint8_t row = 0;
	uint8_t column = 0;
	uint8_t keypad_temp;
	for (row = 0; row < 4; row++) {
		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_2, HIGH);
		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_3, HIGH);
		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_4, HIGH);
		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_5, HIGH);
		GPIO_WriteToOutputPin(GPIOE, (GPIO_PIN_2 + row), LOW);

		for (column = 0; column < 4; column++) {
			if (GPIO_ReadFromInputPin(GPIOE, (GPIO_PIN_7 + column)) == 0) {
				//key pressed
				delay(80);
				keypad_temp = keypad[row][column];
				printf("%c\n", keypad[row][column]);
				//return keypad_temp;
			}
		}
	}
	return keypad_temp;
}

void USART2_Init(void) {
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_BAUD_1200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLENGTH_8;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void USART2_GPIO_Init(void) {
	GPIO_Handle_t usart_gpios;

	GPIO_PClkCtrl(GPIOA, ENABLE);

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFuncMode = 7;

	//usart2 tx
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&usart_gpios);

	//usart rx
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_init(&usart_gpios);


}

void LCD_GPIO_Init(void) {
	GPIO_Handle_t GpioDpins, GpioCpins;

	GPIO_PClkCtrl(GPIOD, ENABLE);
	GPIO_PClkCtrl(GPIOC, ENABLE);

	GpioDpins.pGPIOx = GPIOD; //PD0 to PD7 for the data lines
	GpioDpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioDpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioDpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioDpins.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GpioCpins.pGPIOx = GPIOC; // PC0 to PC2 for control lines
	GpioCpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioCpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioCpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioCpins.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_init(&GpioDpins);

	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_init(&GpioCpins);
	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_init(&GpioCpins);
	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&GpioCpins);

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW); //RW set to Write

}

void LCD_command(unsigned char input) {
	GPIO_WriteToOutputPort(GPIOD, input);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_0, LOW); //RS to 0
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, HIGH); //EN high
	delayMs(10);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW); //EN low
	delayMs(20);

}

void LCD_Init(void) {
	unsigned char commands[] = { 0x38, 0x06, 0x01, 0x0F };
	delayMs(45);
	LCD_command(0x30);
	delayMs(15);
	LCD_command(0x30);
	delayMs(10);
	LCD_command(0x30);
	delayMs(1);

	for (uint8_t i = 0; i <= 3; i++) {
		LCD_command(commands[i]);
	}

}

void LCD_data(char data) {
	GPIO_WriteToOutputPort(GPIOD, data);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_0, HIGH); //RS to 0
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, HIGH); //EN high
	delayMs(10);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW); //EN low
	delayMs(10);
}

int main(void) {
	Keypad_GPIO_Init();
	USART2_GPIO_Init();
	USART2_Init();
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	USART_PCtrl(USART2, ENABLE);
	LCD_GPIO_Init();
	LCD_Init();
	delayMs(10);

	//USART_ReceiveData(&usart2_handle, (uint8_t*)msg_rec, 1);
	//USART_SendData(&usart2_handle, (uint8_t*)msg_sent, 1);
	//receiving the sent data and sending the pressed character from my keypad
	//displaying the received data on the LCD.. must have key pad GPIOs as well
	//send what is returned from key_pad()

	while (1) {
		//while ( USART_ReceiveDataIT(&usart2_handle,(uint8_t*)msg_rec,1) !=
		//USART_READY ){
		pressed_key = key_pad();
		if (pressed_key != 0) {
			msg_sent[0] = pressed_key;
			printf("%c\n", pressed_key);
			USART_SendData(&usart2_handle, (uint8_t*) msg_sent, 1);
			printf("%c\n", pressed_key);
		}
		//}
		//USART_ReceiveData(&usart2_handle,(uint8_t*)msg_rec,1);
		//LCD_data((uint8_t) *msg_rec);
		delayMs(60);

	}
	return 0;
}

void USART2_IRQHandler(void) {
	USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t Event) {
	if (Event == USART_EVENT_RX_CMPLT) {
		USART_ReceiveDataIT(&usart2_handle, (uint8_t*) msg_rec, 1);
		LCD_command(0x01); // clear screen
		delayMs(30);
		LCD_command(0x2); // return cursor home
		delayMs(100);
		LCD_data((uint8_t) *msg_rec);
		delayMs(60); // wait for about 1 sec
	} else if (Event == USART_EVENT_TX_CMPLT) {
		;
	}
}

