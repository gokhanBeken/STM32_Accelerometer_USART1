#include "main.h"



USART_InitTypeDef USART_InitStructure; //uart icin

void init_USART1(uint32_t baudrate);
//void USART1_IRQHandler(void);
void SendChar1(char Tx);
void SendTxt1(char *Txt);
void USART_puts(USART_TypeDef* USARTx, char *s);
void USART1_IRQHandler(void);
