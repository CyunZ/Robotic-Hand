
#include "delay.h"
#include "sys.h"




#define MAX_DATA_LEN 12
unsigned char flag = 0x00;
int dataLen =0;
char data[MAX_DATA_LEN];

union angle
 {
		 float data;
		 unsigned char dataByte[4];
 }angle1,angle2,angle3;

void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    //PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//复位串口2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//停止复位
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //设置NVIC中断分组2:2位抢占优先级，2位响应优先级   0-3;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);   
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	 
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);//TIM3????????  PC6 PC7 PC8 PC9
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);//TIM3?????????  PB4 PB5 PB0 PB1
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_4 | GPIO_Pin_0| GPIO_Pin_1; //TIM3_CH2 1 3 4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7 | GPIO_Pin_8| GPIO_Pin_9; //TIM3_CH1 2 3 4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1 ; //TIM2_CH1 2     //TIM2 ??????? PA0??PA1??PA2??PA3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
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

 	
 int main(void)
{	 
	float result1,result2,result3;
	
	delay_init();	//延时初始化 
	delay_ms(1000);
	PWM_Init(199,7199);
	while(1)
	{
		if(dataLen == 12 && 0x33 == flag)
		{	
			GPIO_ResetBits(GPIOA,GPIO_Pin_8);
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
				
			result1 =  angle1.data/180 * (189-176) + 176;    //176~189
			if(result1 > 189) result1 = 189;
			if(result1 < 176) result1 = 176;
			
			result2 =  angle2.data/180 * (190-176) + 176; 		//176~190
			if(result2 > 190) result2 = 190;
			if(result2 < 176) result2 = 176;
			
			result3 =  angle3.data/180 * (189-176) + 176; 	//176~189
			if(result3 > 189) result3 = 189;
			if(result3 < 176) result3 = 176;
			

			TIM_SetCompare2(TIM3,(int)result1);
			TIM_SetCompare3(TIM3,(int)result2);
			TIM_SetCompare4(TIM3,(int)result3);
			
			flag = 0x00;
			dataLen=0;
			
		}
		/**
		if(flag == 0x00) TIM_SetCompare4(TIM3,178);
		if(flag == 0x01) TIM_SetCompare4(TIM3,189);
		if(flag == 0x02) TIM_SetCompare4(TIM3,189);
		if(flag == 0x03) TIM_SetCompare4(TIM3,189);
		if(flag == 0x13) TIM_SetCompare4(TIM3,189);
		if(flag == 0x23) TIM_SetCompare4(TIM3,189);
		if(flag == 0x33) TIM_SetCompare4(TIM3,189);
		**/
	
		
		/**
		TIM_SetCompare1(TIM3,176);
		TIM_SetCompare2(TIM3,176);
		TIM_SetCompare3(TIM3,176);
		TIM_SetCompare4(TIM3,176);
				
		TIM_SetCompare1(TIM2,175);
		TIM_SetCompare2(TIM2,180);
		
		vTaskDelay(3000);
	
		TIM_SetCompare1(TIM3,190);
		TIM_SetCompare2(TIM3,189);
		TIM_SetCompare3(TIM3,190);
		TIM_SetCompare4(TIM3,189);
		
		TIM_SetCompare1(TIM2,187);
		TIM_SetCompare2(TIM2,189);
		vTaskDelay(3000);
		**/
	}
	
}
 


void USART1_IRQHandler(void)
{
	u8 res;
	//flag = 0x01;
	USART_SendData(USART1,'K');
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{
		res =USART_ReceiveData(USART1);    
		//帧头
		if(0x11 == res && 0x00 == flag) 
		{
	//		USART_SendData(USART1,'A');
			flag = 0x01;
		}
		else if(0x01 == flag)
		{
		//	USART_SendData(USART1,'B');
			if(0x22 == res)	flag = 0x02;
			else flag = 0x00;
		}
		else if(0x02 == flag) 
		{
		//	USART_SendData(USART1,'C');
			if(0x33 == res)	flag = 0x03;
			else flag = 0x00;
		}
		 //接收数据
		else if(dataLen < MAX_DATA_LEN && 0x03 == flag)
		{
			data[dataLen++] = res;
	//		USART_SendData(USART1,'D');
		}
		//帧尾
		else if(dataLen >= MAX_DATA_LEN && 0x03 == flag && 0x33 == res )
		{
		//	USART_SendData(USART1,'E');
			flag = 0x13;
		}
		else if(0x13 == flag) 
		{
		//	USART_SendData(USART1,'F');
			if(0x22 == res)	flag = 0x23;
			else { flag = 0x00;dataLen=0;  }
		}
		else if(0x23 == flag) 
		{
		//	USART_SendData(USART1,'G');
			if(0x11 == res)		flag = 0x33;
			else { flag = 0x00;dataLen=0;  }	
		}
		
		
	}

	USART_ClearFlag(USART1,USART_IT_RXNE); 
}



void USART2_IRQHandler(void)
{
	u8 res;
	//flag = 0x01;
	//USART_SendData(USART1,'K');
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		res =USART_ReceiveData(USART2);    
		//帧头
		if(0x11 == res && 0x00 == flag) 
		{
	//		USART_SendData(USART1,'A');
			flag = 0x01;
		}
		else if(0x01 == flag)
		{
		//	USART_SendData(USART1,'B');
			if(0x22 == res)	flag = 0x02;
			else flag = 0x00;
		}
		else if(0x02 == flag) 
		{
		//	USART_SendData(USART1,'C');
			if(0x33 == res)	flag = 0x03;
			else flag = 0x00;
		}
		 //接收数据
		else if(dataLen < MAX_DATA_LEN && 0x03 == flag)
		{
			data[dataLen++] = res;
	//		USART_SendData(USART1,'D');
		}
		//帧尾
		else if(dataLen >= MAX_DATA_LEN && 0x03 == flag && 0x33 == res )
		{
		//	USART_SendData(USART1,'E');
			flag = 0x13;
		}
		else if(0x13 == flag) 
		{
		//	USART_SendData(USART1,'F');
			if(0x22 == res)	flag = 0x23;
			else { flag = 0x00;dataLen=0;  }
		}
		else if(0x23 == flag) 
		{
		//	USART_SendData(USART1,'G');
			if(0x11 == res)		flag = 0x33;
			else { flag = 0x00;dataLen=0;  }	
		}
		
	}

	USART_ClearFlag(USART2,USART_IT_RXNE); 
}


