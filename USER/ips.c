#include "ips.h"
#include "delay.h"
#include "spi.h"

#define IPS_SPI  1


extern u8 lcd_dir;
extern u8 lcd_mirror;



void delay(int t)
{
	while(t--);
}

void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��A�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_14;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //��ʼ��GPIOB
	
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //PA9-->TX
	GPIO_Init(GPIOA, &GPIO_InitStructure);	  //��ʼ��GPIOA

	
#if IPS_SPI == 0	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //��ʼ��GPIOA
#endif	
	
}

#define MOSI PBout(15)
#define SCLK PBout(13)



u8 FLASH_SPI_ReadWriteByte(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(byte&0x80)MOSI=1;
		else 	MOSI=0;	
		SCLK=1;
		byte<<=1;
		//if(MISO)byte|=0x01;
		//else byte|=0x00;
		SCLK=0;
	}
	return byte;
}


void LCD_Writ_Bus(u8 dat) 
{	
	LCD_CS_Clr();
#if IPS_SPI == 0	
	FLASH_SPI_ReadWriteByte(dat);	
#else	
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);//�����ձ�־λ
	SPI_I2S_SendData(SPI2,dat);
	delay(6);
#endif
	LCD_CS_Set();
}

/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//д����
}


/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	
	LCD_WR_REG(0x2a);//�е�ַ����
	LCD_WR_DATA(x1+1);
	LCD_WR_DATA(x2+1);
	LCD_WR_REG(0x2b);//�е�ַ����
	LCD_WR_DATA(y1+26);
	LCD_WR_DATA(y2+26);
	LCD_WR_REG(0x2c);//������д
	
//	LCD_WR_REG(0x2a);//�е�ַ���� 
//	LCD_WR_DATA(x1+1);
//	LCD_WR_DATA(x2+1);
//	LCD_WR_REG(0x2b);//�е�ַ����
//	LCD_WR_DATA(y1+26);
//	LCD_WR_DATA(y2+26);
//	LCD_WR_REG(0x2c);//������д
}

//0--����   1--���·�ת     
void LCD_dir(u8 dir,u8 mirror)
{
	LCD_WR_REG(0x36);
	if(dir==0)
	{
		if(mirror==0)LCD_WR_DATA8(0x70);
		else         LCD_WR_DATA8(0x30); 
	}
	else
	{
		if(mirror==0)LCD_WR_DATA8(0xB0);
		else         LCD_WR_DATA8(0xF0); 	
	}
}


void LCD_Init(void)
{
	LCD_GPIO_Init();//��ʼ��GPIO
#if IPS_SPI == 1
	SPI2_Init();    //��ʼ��SPI1
#endif
	LCD_RES_Clr();  //��λ
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);
	
  //delay_ms(100);
	
	LCD_WR_REG(0x11);     //Sleep out
	delay_ms(120);                //Delay 120ms
	LCD_WR_REG(0xB1);     //Normal mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB2);     //Idle mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB3);     //Partial mode
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_DATA8(0x3C);   
	LCD_WR_REG(0xB4);     //Dot inversion
	LCD_WR_DATA8(0x03);   
	LCD_WR_REG(0xC0);     //AVDD GVDD
	LCD_WR_DATA8(0xAB);   
	LCD_WR_DATA8(0x0B);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_REG(0xC1);     //VGH VGL
	LCD_WR_DATA8(0xC5);   //C0
	LCD_WR_REG(0xC2);     //Normal Mode
	LCD_WR_DATA8(0x0D);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_REG(0xC3);     //Idle
	LCD_WR_DATA8(0x8D);   
	LCD_WR_DATA8(0x6A);   
	LCD_WR_REG(0xC4);     //Partial+Full
	LCD_WR_DATA8(0x8D);   
	LCD_WR_DATA8(0xEE);   
	LCD_WR_REG(0xC5);     //VCOM
	LCD_WR_DATA8(0x0F);   
	LCD_WR_REG(0xE0);     //positive gamma
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x10);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x02);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x25);   
	LCD_WR_DATA8(0x36);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_DATA8(0x10);   
	LCD_WR_REG(0xE1);     //negative gamma
	LCD_WR_DATA8(0x0A);   
	LCD_WR_DATA8(0x0D);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x02);   
	LCD_WR_DATA8(0x07);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x0F);   
	LCD_WR_DATA8(0x25);   
	LCD_WR_DATA8(0x35);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x04);   
	LCD_WR_DATA8(0x10);
		 
	LCD_WR_REG(0xFC);    
	LCD_WR_DATA8(0x80);  
		
	LCD_WR_REG(0x3A);     
	LCD_WR_DATA8(0x05); 

	LCD_dir(lcd_dir,lcd_mirror);
	
	LCD_WR_REG(0x20);     //Display inversion
	LCD_WR_REG(0x29);     //Display on
	
	LCD_WR_REG(0x2A);     //Set Column Address
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x1A);  //26  
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x69);   //105 
	
	LCD_WR_REG(0x2B);     //Set Page Address
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x01);    //1
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0xA0);    //160
	LCD_WR_REG(0x2C); 
	
	
}








