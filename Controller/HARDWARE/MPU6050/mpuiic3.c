#include "mpuiic3.h"
#include "delay.h"

 
 //MPU IIC 延时函数
void MPU_IIC_Delay3(void)
{
	delay_us(2);
}


//产生IIC起始信号
void MPU_IIC_Start3(void)
{
	MPU_SDA_OUT3();     //sda线输出
	MPU_IIC_SDA3=1;	  	  
	MPU_IIC_SCL3=1;
	MPU_IIC_Delay3();
 	MPU_IIC_SDA3=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop3(void)
{
	MPU_SDA_OUT3();//sda线输出
	MPU_IIC_SCL3=0;
	MPU_IIC_SDA3=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay3();
	MPU_IIC_SCL3=1; 
	MPU_IIC_SDA3=1;//发送I2C总线结束信号
	MPU_IIC_Delay3();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack3(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN3();      //SDA设置为输入  
	MPU_IIC_SDA3=1;MPU_IIC_Delay3();	   
	MPU_IIC_SCL3=1;MPU_IIC_Delay3();	 
	while(MPU_READ_SDA3)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop3();
			return 1;
		}
	}
	MPU_IIC_SCL3=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MPU_IIC_Ack3(void)
{
	MPU_IIC_SCL3=0;
	MPU_SDA_OUT3();
	MPU_IIC_SDA3=0;
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=1;
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=0;
}
//不产生ACK应答		    
void MPU_IIC_NAck3(void)
{
	MPU_IIC_SCL3=0;
	MPU_SDA_OUT3();
	MPU_IIC_SDA3=1;
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=1;
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte3(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT3(); 	    
    MPU_IIC_SCL3=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA3=(txd&0x80)>>7;
        txd<<=1; 	  
		    MPU_IIC_SCL3=1;
		    MPU_IIC_Delay3(); 
		    MPU_IIC_SCL3=0;	
		    MPU_IIC_Delay3();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte3(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN3();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL3=0; 
        MPU_IIC_Delay3();
		MPU_IIC_SCL3=1;
        receive<<=1;
        if(MPU_READ_SDA3)receive++;   
		MPU_IIC_Delay3(); 
    }					 
    if (!ack)
        MPU_IIC_NAck3();//发送nACK
    else
        MPU_IIC_Ack3(); //发送ACK   
    return receive;
}


























