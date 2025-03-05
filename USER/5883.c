
#include "iic.h"
#include "delay.h"

#define	SlaveAddress   0x3C	  //����������IIC�����еĴӵ�ַ


u8 QMC_buf[8];


void Single_Write_HMC5883(u8 REG_Address,u8 REG_data)
{
    IIC_Start();                  //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);   //�����豸��ַ+д�ź�
    IIC_Send_Byte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf 
    IIC_Send_Byte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
    IIC_Stop();                   //����ֹͣ�ź�
}
u8 HMC5883_RecvByte()
{
    u8 i;
    u8 dat = 0;
		SDA_OUT();
    IIC_SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
		SDA_IN();
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        IIC_SCL = 1;                //����ʱ����
        delay_us(5);             //��ʱ
        dat |= READ_SDA;             //������               
        IIC_SCL = 0;                //����ʱ����
        delay_us(5);             //��ʱ
    }
    return dat;
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void HMC5883_SendACK(u8 ack)
{
		SDA_OUT();
    IIC_SDA = ack;                  //дӦ���ź�
    IIC_SCL = 1;                    //����ʱ����
    delay_us(5);                 //��ʱ
    IIC_SCL = 0;                    //����ʱ����
    delay_us(5);                      //��ʱ
}


void Multiple_Read_HMC5883(void)
{   u8 i;
    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress);           //�����豸��ַ+д�ź�
    IIC_Send_Byte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
    IIC_Start();                          //��ʼ�ź�
    IIC_Send_Byte(SlaveAddress+1);         //�����豸��ַ+���ź�
	 for (i=0; i<6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        QMC_buf[i] = HMC5883_RecvByte();          //BUF[0]�洢����
       
				if (i == 5)
        {
           HMC5883_SendACK(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          HMC5883_SendACK(0);                //��ӦACK
       }
   }
    IIC_Stop();                          //ֹͣ�ź�
    delay_ms(3);
}


void Init_HMC5883()
{
     Single_Write_HMC5883(0x02,0x00);  //
}

