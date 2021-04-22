#include "mpuiic2.h"
#include "delay.h"

 
 //MPU IIC 延时函数
void MPU_IIC_Delay2(void)
{
	delay_us(2);
}


//产生IIC起始信号
void MPU_IIC_Start2(void)
{
	MPU_SDA_OUT2();     //sda线输出
	MPU_IIC_SDA2=1;	  	  
	MPU_IIC_SCL2=1;
	MPU_IIC_Delay2();
 	MPU_IIC_SDA2=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop2(void)
{
	MPU_SDA_OUT2();//sda线输出
	MPU_IIC_SCL2=0;
	MPU_IIC_SDA2=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay2();
	MPU_IIC_SCL2=1; 
	MPU_IIC_SDA2=1;//发送I2C总线结束信号
	MPU_IIC_Delay2();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack2(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN2();      //SDA设置为输入  
	MPU_IIC_SDA2=1;MPU_IIC_Delay2();	   
	MPU_IIC_SCL2=1;MPU_IIC_Delay2();	 
	while(MPU_READ_SDA2)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop2();
			return 1;
		}
	}
	MPU_IIC_SCL2=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MPU_IIC_Ack2(void)
{
	MPU_IIC_SCL2=0;
	MPU_SDA_OUT2();
	MPU_IIC_SDA2=0;
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=1;
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=0;
}
//不产生ACK应答		    
void MPU_IIC_NAck2(void)
{
	MPU_IIC_SCL2=0;
	MPU_SDA_OUT2();
	MPU_IIC_SDA2=1;
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=1;
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte2(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT2(); 	    
    MPU_IIC_SCL2=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA2=(txd&0x80)>>7;
        txd<<=1; 	  
		    MPU_IIC_SCL2=1;
		    MPU_IIC_Delay2(); 
		    MPU_IIC_SCL2=0;	
		    MPU_IIC_Delay2();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte2(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN2();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL2=0; 
        MPU_IIC_Delay2();
		MPU_IIC_SCL2=1;
        receive<<=1;
        if(MPU_READ_SDA2)receive++;   
		MPU_IIC_Delay2(); 
    }					 
    if (!ack)
        MPU_IIC_NAck2();//发送nACK
    else
        MPU_IIC_Ack2(); //发送ACK   
    return receive;
}


























