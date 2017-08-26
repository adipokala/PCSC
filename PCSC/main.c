#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_spi.h"
#include "misc.h"
#include <stdio.h>

#define MAX_STRLEN 50

void led_configure();
//void tim2_configure();
//void delay(int sec);
void TM_Delay_Init(void);
void TM_DelayMicros(uint32_t micros);
void TM_DelayMillis(uint32_t millis);
void usart1_configure();
void Mes2Usart1 (char *ptr);
void USART1_SendChar (char ch);
void RC522_Init();

volatile unsigned char flg = 0;
char received_string[MAX_STRLEN+1];
volatile int cnt = 0;
float f;
unsigned int ch;
//volatile int count = 0;
uint32_t multiplier;

int main(void)
{
	RCC_ClocksTypeDef RCC_Clocks;

	SystemInit();
	SystemCoreClockUpdate();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_SYSCFG, ENABLE);

	led_configure();
//	tim2_configure();
	usart1_configure();
	TM_Delay_Init();

	RCC_GetClocksFreq(&RCC_Clocks);

//	delay(2);
	TM_DelayMicros(1000);

	if((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		Mes2Usart1("HSE ON\r\n");
	}
	else
	{
		Mes2Usart1("HSI ON\r\n");
	}

	ch = (RCC->CFGR & RCC_CFGR_SWS);
	sprintf(received_string, "value - %u\r\n", ch);
	Mes2Usart1(received_string);

	f = (float) (RCC_Clocks.SYSCLK_Frequency/1000000);
	sprintf(received_string, "Main sys clock - %4.1f MHz\r\n", f);
	Mes2Usart1(received_string);
	f = (float) (RCC_Clocks.HCLK_Frequency/1000000);
	sprintf(received_string, "AHB Clock - %4.1f MHz\r\n", f);
	Mes2Usart1(received_string);
	f = (float) (RCC_Clocks.PCLK1_Frequency/1000000);
	sprintf(received_string, "PCLK1_Frequency - %4.1f MHz\r\n",f);
	Mes2Usart1(received_string);
	f = (float) (RCC_Clocks.PCLK2_Frequency/1000000);
	sprintf(received_string, "PCLK2_Frequency - %4.1f MHz\r\n",f);
	Mes2Usart1(received_string);

    while(1)
    {
    	GPIO_ToggleBits(GPIOB, GPIO_Pin_0);
//    	delay(5);
    	TM_DelayMillis(500);
    	Mes2Usart1("Hello world..!!\r\n");
    }
}

void led_configure()
{
	GPIO_InitTypeDef led;
	led.GPIO_Pin = GPIO_Pin_0;
	led.GPIO_Mode = GPIO_Mode_OUT;
	led.GPIO_OType = GPIO_OType_PP;
	led.GPIO_PuPd = GPIO_PuPd_NOPULL;
	led.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &led);

	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

void TM_Delay_Init(void)
{
	RCC_ClocksTypeDef RCC_Clks;

	/* Get system clocks */
	RCC_GetClocksFreq(&RCC_Clks);

	/* While loop takes 4 cycles */
	/* For 1 us delay, we need to divide with 4M */
	multiplier = RCC_Clks.HCLK_Frequency / 4000000;
}

void TM_DelayMicros(uint32_t micros)
{
	/* Multiply micros with multipler */
	/* Substract 10 */
	micros = micros * multiplier - 10;
	/* 4 cycles for one loop */
	while (micros--);
}

void TM_DelayMillis(uint32_t millis)
{
	/* Multiply millis with multipler */
	/* Substract 10 */
	millis = 1000 * millis * multiplier - 10;
	/* 4 cycles for one loop */
	while (millis--);
}


void usart1_configure()
{
	GPIO_InitTypeDef gusart;
	USART_InitTypeDef USART1_InitStruct;
	NVIC_InitTypeDef NVIC1_InitStruct;

	gusart.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	gusart.GPIO_Mode = GPIO_Mode_AF;
	gusart.GPIO_OType = GPIO_OType_PP;
	gusart.GPIO_PuPd = GPIO_PuPd_UP;
	gusart.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gusart);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	USART1_InitStruct.USART_BaudRate = 115200;
	USART1_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART1_InitStruct.USART_StopBits = USART_StopBits_1;
	USART1_InitStruct.USART_Parity = USART_Parity_No;
	USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART1_InitStruct);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC1_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC1_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC1_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC1_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC1_InitStruct);

	USART_Cmd(USART1, ENABLE);
}

void Mes2Usart1 (char *ptr)
{
 while (*ptr)
	USART1_SendChar(*ptr++);
}

void USART1_SendChar (char ch)
{
 	USART_SendData(USART1, ch);
 	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_IRQHandler(void)
{
		char t = (char)USART_ReceiveData(USART1);
		if (flg) return;   // if buffer is full, do not accept any new chars

		if (t != '\r' && t != '\n')
		{
		  received_string[cnt] = t;
		  if (cnt < MAX_STRLEN) cnt++;
		}
	    else
		{
	    	received_string[cnt]='\0';
			flg = 1;//Mes2Usart1(received_string);
		}
}

void RC522_Init()
{
	GPIO_InitTypeDef spi;
	SPI_InitTypeDef rc522;
	spi.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4; //3 rst --- 4 nss
	spi.GPIO_Speed = GPIO_Speed_50MHz;
	spi.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &spi);

	spi.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // 5 sck --- 6 miso --- 7 mosi
	spi.GPIO_Speed = GPIO_Speed_50MHz;
	spi.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &spi);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5 | GPIO_PinSource6 | GPIO_PinSource7, GPIO_AF_SPI1);

	rc522.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	rc522.SPI_Mode = SPI_Mode_Master;
	rc522.SPI_DataSize = SPI_DataSize_8b;
	rc522.SPI_CPOL = SPI_CPOL_Low;
	rc522.SPI_CPHA = SPI_CPHA_1Edge;
	rc522.SPI_NSS = SPI_NSS_Soft;
	rc522.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	rc522.SPI_FirstBit = SPI_FirstBit_MSB;
	rc522.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &rc522);

	SPI_Cmd(SPI1, ENABLE);
}

//void tim2_configure()
//{
//	TIM_TimeBaseInitTypeDef forDelay;
//	NVIC_InitTypeDef nvic;
//
//	forDelay.TIM_ClockDivision = TIM_CKD_DIV1;
//	forDelay.TIM_CounterMode = TIM_CounterMode_Up;
//	forDelay.TIM_Prescaler = 0;
//	forDelay.TIM_Period = 2000 - 1;
//	forDelay.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM2, &forDelay);
//
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//
//	nvic.NVIC_IRQChannel = TIM2_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 0;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);
//}
//
//void TIM2_IRQHandler()
//{
//	if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update))
//	{
//		count++;
//	}
////	TIM_SetAutoreload(TIM2, 1999);
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//}
//
//void delay(int sec)
//{
//	TIM_SetCounter(TIM2, 1999);
//	TIM_Cmd(TIM2, ENABLE);
//	while(count != sec)
//	{
//		Mes2Usart1("1\r\n");
//	}
//	count = 0;
//	TIM_Cmd(TIM2, DISABLE);
//}
