#include "stm32f4xx.h"
#include <stdio.h>

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "misc.h"
#include <stm32f4xx_usart.h>



unsigned char PWM[8]; // PWM registerler
unsigned char SRG[8]; // Shadow Registerler
unsigned char CNTR; // PWM Counter

//bayraklar
char x_koordinat = 0, y_koordinat = 0; //koordinat bayraklarimiz


GPIO_InitTypeDef GPIO_InitStructure; //GPIO icin
NVIC_InitTypeDef NVIC_InitStructure;


void Delay(__IO uint32_t nCount);

void init_GPIO(void);

void init_SPI(void);

char Write(char Adr, unsigned char Data);
char Read(char Adr);
char SPI_CMD(short DAT);






