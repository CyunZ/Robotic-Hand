#ifndef _MPU6050I2C_h_
#define _MPU6050I2C_h_

class MPU6050I2C {
public:
    MPU6050I2C();

    //IIC���в�������
    void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
    void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
    void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
    void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
    void MPU_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
    u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
    u8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
    void MPU_IIC_Ack(void);					//IIC����ACK�ź�
    void MPU_IIC_NAck(void);				//IIC������ACK�ź�

    void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
    u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  

private:
    volatile unsigned long  *MPU_IIC_SCL;    		//SCL
    volatile unsigned long  *MPU_IIC_SDA ;    		//SDA	 
    volatile unsigned long  *MPU_READ_SDA ;   		//����SDA 
};

#endif