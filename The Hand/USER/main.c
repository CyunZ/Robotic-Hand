
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

#define MAX_DATA_LEN 12
unsigned char flag = 0x00;
int dataLen =0;
char data[MAX_DATA_LEN];

union angle
 {
		 float data;
		 unsigned char dataByte[4];
 }angle1,angle2,angle3;

void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //TIM2 不重映射 PA0、PA1、PA2、PA3
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1,ENABLE);
	
	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	 
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);//TIM3部分重映像  PC6 PC7 PC8 PC9
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);//TIM3部分重映像  PB4 PB5 PB0 PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_4 | GPIO_Pin_0| GPIO_Pin_1; //TIM3_CH2 1 3 4
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7 | GPIO_Pin_8| GPIO_Pin_9; //TIM3_CH2 1 3 4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1; //TIM2_CH1 2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM3,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	
}




void vPWMTask(void * pvParameters)
{
	float result1,result2,result3;
	
	
	while(1)
	{
		
		if(dataLen == 12 && 0x33 == flag)
		{
			//处理数据		
			angle1.dataByte[0] = data[0];
			angle1.dataByte[1] = data[1];
			angle1.dataByte[2] = data[2];
			angle1.dataByte[3] = data[3];
			
			angle2.dataByte[0] = data[4];
			angle2.dataByte[1] = data[5];
			angle2.dataByte[2] = data[6];
			angle2.dataByte[3] = data[7];
			
			angle3.dataByte[0] = data[8];
			angle3.dataByte[1] = data[9];
			angle3.dataByte[2] = data[10];
			angle3.dataByte[3] = data[11];
				
			result1 =  angle1.data/180 * (197-176);    //176~197
			result2 =  angle2.data/180 * (193-177); 		//177~193
			result3 =  angle3.data/180 * (192-179); 	//179~192
			
			TIM_SetCompare2(TIM3,result1);
			TIM_SetCompare1(TIM3,result2);
			TIM_SetCompare3(TIM3,result3);
			
			//重置
			flag = 0x00;
			dataLen=0;
		}
		
		/**
		TIM_SetCompare2(TIM3,176);
		TIM_SetCompare1(TIM3,177);
		TIM_SetCompare3(TIM3,179);
		TIM_SetCompare4(TIM3,178);
				
		TIM_SetCompare1(TIM2,179);
		TIM_SetCompare2(TIM2,178);
		
		vTaskDelay(3000);
		
		TIM_SetCompare2(TIM3,194);
		TIM_SetCompare1(TIM3,193);
		TIM_SetCompare3(TIM3,192);
		TIM_SetCompare4(TIM3,191);
		
		TIM_SetCompare1(TIM2,190);
		TIM_SetCompare2(TIM2,189);
		vTaskDelay(3000);
		**/
		vTaskDelay(200);
	}
	
}

void vLEDToggleTask(void * pvParameters)
{
	while(1)
	{
		
		vTaskDelay(1000);
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		vTaskDelay(1000);
	}
}

int main(void)
{
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	//TIM3_PWM_Init(899,0);
	TIM3_PWM_Init(199,7199);
	//GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	 
  xTaskCreate( vPWMTask, "vPWMTask", 256, NULL, 2, NULL );
	
//	xTaskCreate( vLEDToggleTask, "vLEDToggleTask", 256, NULL, 3, NULL );
	
	vTaskStartScheduler();
	return 0;
}




void USART1_IRQHandler()
{
	u8 res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{
		res =USART_ReceiveData(USART1);    //读取接收到的数据 
		//判断帧头
		if(0x11 == res && 0x00 == flag) 
		{
			flag = 0x01;
		}
		if(0x01 == flag)
		{
			if(0x22 == res)	flag = 0x02;
			else flag = 0x00;
		}
		if(0x02 == flag) 
		{
			if(0x33 == res)	flag = 0x03;
			else flag = 0x00;
		}
		 //接收数据
		if(dataLen < MAX_DATA_LEN && 0x03 == flag)
		{
			data[dataLen++] = res;
		}
		//判断帧尾
		if(dataLen >= MAX_DATA_LEN && 0x03 == flag && 0x33 == res )
		{
			flag = 0x13;
		} else { flag = 0x00;dataLen=0;  }
		if(0x13 == flag) 
		{
			if(0x22 == res)	flag = 0x23;
			else { flag = 0x00;dataLen=0;  }
		}
		if(0x23 == flag) 
		{
			if(0x11 == res)		flag = 0x33;
			else { flag = 0x00;dataLen=0;  }
		}
		
	}

	USART_ClearFlag(USART1,USART_IT_RXNE); //一定要清除接收中断
}

