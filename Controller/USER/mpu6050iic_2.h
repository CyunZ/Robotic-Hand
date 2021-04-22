// mpu6050iic_2

#include "sys.h"
#include "delay.h"
#include "mpu6050ex.h"

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



//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init2(void)
{ 
	u8 res;
	
	MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
  delay_ms(100);
	MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr2(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr2(0);					//加速度传感器,±2g
	MPU_Set_Rate2(50);						//设置采样率50Hz
	MPU_Write_Byte2(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte2(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte2(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte2(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte2(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte2(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate2(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr2(u8 fsr)
{
	return MPU_Write_Byte2(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr2(u8 fsr)
{
	return MPU_Write_Byte2(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF2(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte2(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate2(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte2(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF2(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature2(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len2(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope2(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len2(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer2(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len2(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len2(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack2())	//等待应答
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack2();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte2(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack2())		//等待ACK
		{
			MPU_IIC_Stop2();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop2();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len2(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack2())	//等待应答
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack2();		//等待应答
    MPU_IIC_Start2();
	MPU_IIC_Send_Byte2((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack2();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte2(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte2(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop2();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte2(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack2())	//等待应答
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack2();		//等待应答 
	MPU_IIC_Send_Byte2(data);//发送数据
	if(MPU_IIC_Wait_Ack2())	//等待ACK
	{
		MPU_IIC_Stop2();	 
		return 1;		 
	}		 
    MPU_IIC_Stop2();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte2(u8 reg)
{
	u8 res;
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack2();		//等待应答 
    MPU_IIC_Send_Byte2(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack2();		//等待应答
    MPU_IIC_Start2();
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack2();		//等待应答 
	res=MPU_IIC_Read_Byte2(0);//读取数据,发送nACK 
    MPU_IIC_Stop2();			//产生一个停止条件 
	return res;		
}
