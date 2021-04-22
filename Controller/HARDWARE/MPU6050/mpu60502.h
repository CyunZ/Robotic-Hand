#ifndef __MPU60502_H
#define __MPU60502_H
#include "mpuiic2.h"   		
#include "mpu6050ex.h"


u8 MPU_Init2(void); 								//初始化MPU6050
u8 MPU_Write_Len2(u8 addr,u8 reg,u8 len,u8 *buf);//IIC连续写
u8 MPU_Read_Len2(u8 addr,u8 reg,u8 len,u8 *buf); //IIC连续读 
u8 MPU_Write_Byte2(u8 reg,u8 data);				//IIC写一个字节
u8 MPU_Read_Byte2(u8 reg);						//IIC读一个字节

u8 MPU_Set_Gyro_Fsr2(u8 fsr);
u8 MPU_Set_Accel_Fsr2(u8 fsr);
u8 MPU_Set_LPF2(u16 lpf);
u8 MPU_Set_Rate2(u16 rate);
u8 MPU_Set_Fifo2(u8 sens);


short MPU_Get_Temperature2(void);
u8 MPU_Get_Gyroscope2(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer2(short *ax,short *ay,short *az);

#endif


