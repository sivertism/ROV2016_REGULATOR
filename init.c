#include "stm32f30x.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

void usart_init(void){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Clock setup */
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStructure);

	/* UART setup */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity =  USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);

	/* GPIO Config */
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* USART2 Tx (PA2) */
	GPIO_InitStructure_UART.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure_UART.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

	/* USART2 Rx (PA3) */
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

	/* Enable USART2 */
	USART_Cmd(USART2, ENABLE);
}


