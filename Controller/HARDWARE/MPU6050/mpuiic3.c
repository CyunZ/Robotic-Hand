#include "mpuiic3.h"
#include "delay.h"

 
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


























