#include "sys.h"
#include "delay.h"
#include "usart.h"  
#include "mpu6050.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//MPU6050 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
//****************************************
// ??MPU6050????



typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;


//IO��������	 
#define MPU_IIC_SCL    PAout(2) //SCL
#define MPU_IIC_SDA    PAout(3) //SDA	 
#define MPU_READ_SDA   PAin(3)  //����SDA 


//��ʼ��IIC
void MPU_SDA_IN(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void MPU_SDA_OUT(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void Delay5us()
{ 
	u16 n=165;
	while(n--);	
}
void I2C_SendACK(u8 ack)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA = ack;                  //?????
	MPU_IIC_SCL = 1;                    //?????
	Delay5us();                 //??
	MPU_IIC_SCL = 0;                    //?????
	Delay5us();                 //??
}
//**************************************
//I2C??????
//**************************************
u8 I2C_RecvACK()
{
	u8 i;
	MPU_IIC_SCL = 1;
	MPU_SDA_IN();
	Delay5us();                 //??
	i = MPU_READ_SDA;                   //?????
	MPU_IIC_SCL = 0;                    //?????
	Delay5us();                 //??
	return i;
}

 

void I2C_SendByte(uchar txd)
{
  u8 t;   
	MPU_SDA_OUT(); 	    
  MPU_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {              
		MPU_IIC_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		MPU_IIC_SCL=1;
		Delay5us();    
		MPU_IIC_SCL=0;	
		Delay5us();    
  }	
}
uchar I2C_RecvByte()
{
	u8 i,receive=0;
	MPU_SDA_IN();
  for(i=0;i<8;i++ )
	{
    MPU_IIC_SCL=0; 
    Delay5us();   
		MPU_IIC_SCL=1;
    receive<<=1;
    if(MPU_READ_SDA)receive++;   
		Delay5us();   
  }					 
  return receive;
}
void I2C_Stop()
{
	MPU_SDA_OUT(); 	 
	MPU_IIC_SDA = 0;                    //?????
	MPU_IIC_SCL = 1;                    //?????
	Delay5us();                 //??
	MPU_IIC_SDA = 1;                    //?????
	Delay5us();                 //??
}
void I2C_Start()
{
	MPU_SDA_OUT(); 	 
	MPU_IIC_SDA = 1;                    //?????
	MPU_IIC_SCL = 1;                    //?????
	Delay5us();                 //??
	MPU_IIC_SDA = 0;                    //?????
	Delay5us();                 //??
	MPU_IIC_SCL = 0;                    //?????
}

u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA=1;Delay5us();   
	MPU_IIC_SCL=1;Delay5us();   
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
}

u8 Single_WriteI2C(uchar REG_Address,uchar REG_data)
{
	I2C_Start();                  //????
	I2C_SendByte(SlaveAddress);   //??????+???
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		I2C_Stop();		 
		return 1;		
	}
	I2C_SendByte(REG_Address);    //???????,
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	I2C_SendByte(REG_data);       //???????,
	if(MPU_IIC_Wait_Ack())	//�ȴ�ACK
	{
		I2C_Stop();   
		return 1;		 
	}	
	I2C_Stop();  
  return 0;		//??????
}


uchar Single_ReadI2C(uchar REG_Address)
{
	uchar REG_data;
	I2C_Start();                   //????
	I2C_SendByte(SlaveAddress);    //??????+???
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	I2C_SendByte(REG_Address);     //????????,?0??	
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	I2C_Start();                   //????
	I2C_SendByte(SlaveAddress+1);  //??????+???
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	REG_data=I2C_RecvByte();       //???????
	I2C_SendACK(0);                //??????
	I2C_Stop();                    //????
	return REG_data;
}

int GetData(u8 REG_Address)
{
	uchar H,L;
	short int temp;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	temp=(H<<8)+L;
	if(temp>18000)
	{
		return temp-65535;
	}
	else return temp;   //????
}

void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	//ʹ��GPIOBʱ��
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_2|GPIO_Pin_3); 	//PA2,PA3 �����
}



void MPU6050_Init(void)
{ 
	IIC_Init();
	MPU_IIC_SCL=1;
	MPU_IIC_SDA=1;
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//??????
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(MPU_CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, 0x18);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);
}



