#ifndef __MPUIIC2_H
#define __MPUIIC2_H
#include "sys.h"

 	   		   
//IO��������
#define MPU_SDA_IN2()  {GPIOB->CRL&=0XF0FFFFFF;GPIOB->CRL|=8<<24;}
#define MPU_SDA_OUT2() {GPIOB->CRL&=0XF0FFFFFF;GPIOB->CRL|=3<<24;}

//IO��������	 
#define MPU_IIC_SCL2    PBout(5) 		//SCL
#define MPU_IIC_SDA2    PBout(6) 		//SDA	 
#define MPU_READ_SDA2  PBin(6) 		//����SDA 

//IIC���в�������
void MPU_IIC_Delay2(void);				//MPU IIC��ʱ����
void MPU_IIC_Init2(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start2(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop2(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte2(u8 txd);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte2(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack2(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack2(void);					//IIC����ACK�ź�
void MPU_IIC_NAck2(void);				//IIC������ACK�ź�

#endif
