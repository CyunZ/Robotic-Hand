#ifndef __MPUIIC3_H
#define __MPUIIC3_H
#include "sys.h"

 	   		   
//IO方向设置
#define MPU_SDA_IN3()  {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=8;}
#define MPU_SDA_OUT3() {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=3;}

//IO操作函数	 
#define MPU_IIC_SCL3    PBout(7) 		//SCL
#define MPU_IIC_SDA3    PBout(8) 		//SDA	 
#define MPU_READ_SDA3  PBin(8) 		//输入SDA 

//IIC所有操作函数
void MPU_IIC_Delay3(void);				//MPU IIC延时函数
void MPU_IIC_Init3(void);                //初始化IIC的IO口				 
void MPU_IIC_Start3(void);				//发送IIC开始信号
void MPU_IIC_Stop3(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte3(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte3(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack3(void); 				//IIC等待ACK信号
void MPU_IIC_Ack3(void);					//IIC发送ACK信号
void MPU_IIC_NAck3(void);				//IIC不发送ACK信号

#endif
