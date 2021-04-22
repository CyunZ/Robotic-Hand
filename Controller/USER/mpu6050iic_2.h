// mpu6050iic_2

#include "sys.h"
#include "delay.h"
#include "mpu6050ex.h"

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

u8 MPU_Init2(void); 								//��ʼ��MPU6050
u8 MPU_Write_Len2(u8 addr,u8 reg,u8 len,u8 *buf);//IIC����д
u8 MPU_Read_Len2(u8 addr,u8 reg,u8 len,u8 *buf); //IIC������ 
u8 MPU_Write_Byte2(u8 reg,u8 data);				//IICдһ���ֽ�
u8 MPU_Read_Byte2(u8 reg);						//IIC��һ���ֽ�

u8 MPU_Set_Gyro_Fsr2(u8 fsr);
u8 MPU_Set_Accel_Fsr2(u8 fsr);
u8 MPU_Set_LPF2(u16 lpf);
u8 MPU_Set_Rate2(u16 rate);
u8 MPU_Set_Fifo2(u8 sens);


short MPU_Get_Temperature2(void);
u8 MPU_Get_Gyroscope2(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer2(short *ax,short *ay,short *az);



 //MPU IIC ��ʱ����
void MPU_IIC_Delay2(void)
{
	delay_us(2);
}


//����IIC��ʼ�ź�
void MPU_IIC_Start2(void)
{
	MPU_SDA_OUT2();     //sda�����
	MPU_IIC_SDA2=1;	  	  
	MPU_IIC_SCL2=1;
	MPU_IIC_Delay2();
 	MPU_IIC_SDA2=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay2();
	MPU_IIC_SCL2=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop2(void)
{
	MPU_SDA_OUT2();//sda�����
	MPU_IIC_SCL2=0;
	MPU_IIC_SDA2=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay2();
	MPU_IIC_SCL2=1; 
	MPU_IIC_SDA2=1;//����I2C���߽����ź�
	MPU_IIC_Delay2();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack2(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN2();      //SDA����Ϊ����  
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
	MPU_IIC_SCL2=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte2(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT2(); 	    
    MPU_IIC_SCL2=0;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte2(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN2();//SDA����Ϊ����
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
        MPU_IIC_NAck2();//����nACK
    else
        MPU_IIC_Ack2(); //����ACK   
    return receive;
}



//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init2(void)
{ 
	u8 res;
	
	MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
  delay_ms(100);
	MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr2(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr2(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate2(50);						//���ò�����50Hz
	MPU_Write_Byte2(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte2(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte2(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte2(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte2(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte2(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte2(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate2(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr2(u8 fsr)
{
	return MPU_Write_Byte2(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr2(u8 fsr)
{
	return MPU_Write_Byte2(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF2(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte2(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate2(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte2(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF2(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len2(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack2())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte2(buf[i]);	//��������
		if(MPU_IIC_Wait_Ack2())		//�ȴ�ACK
		{
			MPU_IIC_Stop2();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop2();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len2(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack2())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ��
    MPU_IIC_Start2();
	MPU_IIC_Send_Byte2((addr<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte2(0);//������,����nACK 
		else *buf=MPU_IIC_Read_Byte2(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop2();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte2(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack2())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop2();		 
		return 1;		
	}
    MPU_IIC_Send_Byte2(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ�� 
	MPU_IIC_Send_Byte2(data);//��������
	if(MPU_IIC_Wait_Ack2())	//�ȴ�ACK
	{
		MPU_IIC_Stop2();	 
		return 1;		 
	}		 
    MPU_IIC_Stop2();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte2(u8 reg)
{
	u8 res;
    MPU_IIC_Start2(); 
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|0);//����������ַ+д����	
	MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ�� 
    MPU_IIC_Send_Byte2(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ��
    MPU_IIC_Start2();
	MPU_IIC_Send_Byte2((MPU_ADDR<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack2();		//�ȴ�Ӧ�� 
	res=MPU_IIC_Read_Byte2(0);//��ȡ����,����nACK 
    MPU_IIC_Stop2();			//����һ��ֹͣ���� 
	return res;		
}
