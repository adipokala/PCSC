#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_spi.h"
#include "misc.h"

void led_configure();
void tim2_configure();
void delay(int sec);

int main(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    while(1)
    {
    	GPIO_ToggleBits(GPIOB, GPIO_Pin_0);
    	delay(5);
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

void tim2_configure()
{
	TIM_TimeBaseInitTypeDef forDelay;
	NVIC_InitTypeDef nvic;

	forDelay.TIM_ClockDivision = TIM_CKD_DIV1;
	forDelay.TIM_CounterMode = TIM_CounterMode_Up;
	forDelay.TIM_Prescaler = 0;
	forDelay.TIM_Period = 2000 - 1;
	forDelay.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &forDelay);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void TIM2_IRQHandler()
{
	TIM_Cmd(TIM2, DISABLE);
}

void delay(int sec)
{
	for(int i = 0; i < sec; i++)
	{
		TIM_Cmd(TIM2, ENABLE);
	}
}
