#include "sys.h"
#include "delay.h"
#include "mpu6050ex.h"

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

u8 MPU_Init3(void); 								//��ʼ��MPU6050
u8 MPU_Write_Len3(u8 addr,u8 reg,u8 len,u8 *buf);//IIC����д
u8 MPU_Read_Len3(u8 addr,u8 reg,u8 len,u8 *buf); //IIC������ 
u8 MPU_Write_Byte3(u8 reg,u8 data);				//IICдһ���ֽ�
u8 MPU_Read_Byte3(u8 reg);						//IIC��һ���ֽ�

u8 MPU_Set_Gyro_Fsr3(u8 fsr);
u8 MPU_Set_Accel_Fsr3(u8 fsr);
u8 MPU_Set_LPF3(u16 lpf);
u8 MPU_Set_Rate3(u16 rate);
u8 MPU_Set_Fifo3(u8 sens);


short MPU_Get_Temperature3(void);
u8 MPU_Get_Gyroscope3(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer3(short *ax,short *ay,short *az);




 //MPU IIC ��ʱ����
void MPU_IIC_Delay3(void)
{
	delay_us(2);
}


//����IIC��ʼ�ź�
void MPU_IIC_Start3(void)
{
	MPU_SDA_OUT3();     //sda�����
	MPU_IIC_SDA3=1;	  	  
	MPU_IIC_SCL3=1;
	MPU_IIC_Delay3();
 	MPU_IIC_SDA3=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay3();
	MPU_IIC_SCL3=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop3(void)
{
	MPU_SDA_OUT3();//sda�����
	MPU_IIC_SCL3=0;
	MPU_IIC_SDA3=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay3();
	MPU_IIC_SCL3=1; 
	MPU_IIC_SDA3=1;//����I2C���߽����ź�
	MPU_IIC_Delay3();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack3(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN3();      //SDA����Ϊ����  
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
	MPU_IIC_SCL3=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte3(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT3(); 	    
    MPU_IIC_SCL3=0;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte3(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN3();//SDA����Ϊ����
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
        MPU_IIC_NAck3();//����nACK
    else
        MPU_IIC_Ack3(); //����ACK   
    return receive;
}



//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init3(void)
{ 
	u8 res;
	
	MPU_Write_Byte3(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
  delay_ms(100);
	MPU_Write_Byte3(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr3(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr3(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate3(50);						//���ò�����50Hz
	MPU_Write_Byte3(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte3(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte3(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte3(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte3(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte3(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte3(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate3(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr3(u8 fsr)
{
	return MPU_Write_Byte3(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr3(u8 fsr)
{
	return MPU_Write_Byte3(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF3(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte3(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate3(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte3(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF3(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature3(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len3(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope3(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len3(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
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
u8 MPU_Get_Accelerometer3(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len3(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
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
u8 MPU_Write_Len3(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start3(); 
	MPU_IIC_Send_Byte3((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack3())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop3();		 
		return 1;		
	}
    MPU_IIC_Send_Byte3(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte3(buf[i]);	//��������
		if(MPU_IIC_Wait_Ack3())		//�ȴ�ACK
		{
			MPU_IIC_Stop3();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop3();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len3(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start3(); 
	MPU_IIC_Send_Byte3((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack3())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop3();		 
		return 1;		
	}
    MPU_IIC_Send_Byte3(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ��
    MPU_IIC_Start3();
	MPU_IIC_Send_Byte3((addr<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte3(0);//������,����nACK 
		else *buf=MPU_IIC_Read_Byte3(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop3();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte3(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start3(); 
	MPU_IIC_Send_Byte3((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack3())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop3();		 
		return 1;		
	}
    MPU_IIC_Send_Byte3(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ�� 
	MPU_IIC_Send_Byte3(data);//��������
	if(MPU_IIC_Wait_Ack3())	//�ȴ�ACK
	{
		MPU_IIC_Stop3();	 
		return 1;		 
	}		 
    MPU_IIC_Stop3();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte3(u8 reg)
{
	u8 res;
    MPU_IIC_Start3(); 
	MPU_IIC_Send_Byte3((MPU_ADDR<<1)|0);//����������ַ+д����	
	MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ�� 
    MPU_IIC_Send_Byte3(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ��
    MPU_IIC_Start3();
	MPU_IIC_Send_Byte3((MPU_ADDR<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack3();		//�ȴ�Ӧ�� 
	res=MPU_IIC_Read_Byte3(0);//��ȡ����,����nACK 
    MPU_IIC_Stop3();			//����һ��ֹͣ���� 
	return res;		
}
