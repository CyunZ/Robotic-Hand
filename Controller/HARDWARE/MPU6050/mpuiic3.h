#ifndef __MPUIIC3_H
#define __MPUIIC3_H
#include "sys.h"

 	   		   
//IO��������
#define MPU_SDA_IN3()  {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=8;}
#define MPU_SDA_OUT3() {GPIOB->CRH&=0XFFFFFFF0;GPIOB->CRH|=3;}

//IO��������	 
#define MPU_IIC_SCL3    PBout(7) 		//SCL
#define MPU_IIC_SDA3    PBout(8) 		//SDA	 
#define MPU_READ_SDA3  PBin(8) 		//����SDA 

//IIC���в�������
void MPU_IIC_Delay3(void);				//MPU IIC��ʱ����
void MPU_IIC_Init3(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start3(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop3(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte3(u8 txd);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte3(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack3(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack3(void);					//IIC����ACK�ź�
void MPU_IIC_NAck3(void);				//IIC������ACK�ź�

#endif
