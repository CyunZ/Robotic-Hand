#ifndef __MPU60503_H
#define __MPU60503_H
#include "mpuiic3.h"   		
#include "mpu6050ex.h"


u8 MPU_Init3(void); 								//初始化MPU6050
u8 MPU_Write_Len3(u8 addr,u8 reg,u8 len,u8 *buf);//IIC连续写
u8 MPU_Read_Len3(u8 addr,u8 reg,u8 len,u8 *buf); //IIC连续读 
u8 MPU_Write_Byte3(u8 reg,u8 data);				//IIC写一个字节
u8 MPU_Read_Byte3(u8 reg);						//IIC读一个字节

u8 MPU_Set_Gyro_Fsr3(u8 fsr);
u8 MPU_Set_Accel_Fsr3(u8 fsr);
u8 MPU_Set_LPF3(u16 lpf);
u8 MPU_Set_Rate3(u16 rate);
u8 MPU_Set_Fifo3(u8 sens);


short MPU_Get_Temperature3(void);
u8 MPU_Get_Gyroscope3(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer3(short *ax,short *ay,short *az);

#endif
