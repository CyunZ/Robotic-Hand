#ifndef __MPUIIC2_H
#define __MPUIIC2_H
#include "sys.h"

 	   		   
//IO方向设置
#define MPU_SDA_IN2()  {GPIOB->CRL&=0XF0FFFFFF;GPIOB->CRL|=8<<24;}
#define MPU_SDA_OUT2() {GPIOB->CRL&=0XF0FFFFFF;GPIOB->CRL|=3<<24;}

//IO操作函数	 
#define MPU_IIC_SCL2    PBout(5) 		//SCL
#define MPU_IIC_SDA2    PBout(6) 		//SDA	 
#define MPU_READ_SDA2  PBin(6) 		//输入SDA 

//IIC所有操作函数
void MPU_IIC_Delay2(void);				//MPU IIC延时函数
void MPU_IIC_Init2(void);                //初始化IIC的IO口				 
void MPU_IIC_Start2(void);				//发送IIC开始信号
void MPU_IIC_Stop2(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte2(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte2(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack2(void); 				//IIC等待ACK信号
void MPU_IIC_Ack2(void);					//IIC发送ACK信号
void MPU_IIC_NAck2(void);				//IIC不发送ACK信号

#endif
