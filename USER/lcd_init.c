#include "lcd_init.h"
#include "delay.h"
#include "spi.h"

void delay(int t)
{
	while (t--)
		;
}

void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); // 使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_12 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速度50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			  // 初始化GPIOA

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // 初始化GPIOA
}

/******************************************************************************
	  函数说明：LCD串行数据写入函数
	  入口数据：dat  要写入的串行数据
	  返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat)
{
	LCD_CS_Clr();
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		; // 检查接收标志位
	SPI_I2S_SendData(SPI2, dat);
	delay(6);
	LCD_CS_Set();
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat >> 8);
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入命令
	  入口数据：dat 写入的命令
	  返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr(); // 写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set(); // 写数据
}

/******************************************************************************
	  函数说明：设置起始和结束地址
	  入口数据：x1,x2 设置列的起始和结束地址
				y1,y2 设置行的起始和结束地址
	  返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
	//if (USE_HORIZONTAL == 0)
	//{
	//	LCD_WR_REG(0x2a); // 列地址设置
	//	LCD_WR_DATA(x1 + 26);
	//	LCD_WR_DATA(x2 + 26);
	//	LCD_WR_REG(0x2b); // 行地址设置
	//	LCD_WR_DATA(y1 + 1);
	//	LCD_WR_DATA(y2 + 1);
	//	LCD_WR_REG(0x2c); // 储存器写
	//}
	//else if (USE_HORIZONTAL == 1)
	//{
	//	LCD_WR_REG(0x2a); // 列地址设置
	//	LCD_WR_DATA(x1 + 26);
	//	LCD_WR_DATA(x2 + 26);
	//	LCD_WR_REG(0x2b); // 行地址设置
	//	LCD_WR_DATA(y1 + 1);
	//	LCD_WR_DATA(y2 + 1);
	//	LCD_WR_REG(0x2c); // 储存器写
	//}
	//else if (USE_HORIZONTAL == 2)
	//{
	//	LCD_WR_REG(0x2a); // 列地址设置
	//	LCD_WR_DATA(x1 + 1);
	//	LCD_WR_DATA(x2 + 1);
	//	LCD_WR_REG(0x2b); // 行地址设置
	//	LCD_WR_DATA(y1 + 26);
	//	LCD_WR_DATA(y2 + 26);
	//	LCD_WR_REG(0x2c); // 储存器写
	//}
	//else
	//{
	//	LCD_WR_REG(0x2a); // 列地址设置
	//	LCD_WR_DATA(x1 + 1);
	//	LCD_WR_DATA(x2 + 1);
	//	LCD_WR_REG(0x2b); // 行地址设置
	//	LCD_WR_DATA(y1 + 26);
	//	LCD_WR_DATA(y2 + 26);
	//	LCD_WR_REG(0x2c); // 储存器写
	//}
	LCD_WR_REG(0x2a); // 列地址设置
	LCD_WR_DATA(x1 + 1);
	LCD_WR_DATA(x2 + 1);
	LCD_WR_REG(0x2b); // 行地址设置
	LCD_WR_DATA(y1 + 26);
	LCD_WR_DATA(y2 + 26);
	LCD_WR_REG(0x2c); // 储存器写
}

void LCD_Init(void)
{
	LCD_GPIO_Init(); // 初始化GPIO
	SPI2_Init();
	LCD_RES_Clr(); // 复位
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);

	LCD_BLK_Set(); // 打开背光
	delay_ms(100);

	LCD_WR_REG(0x11); // Sleep out
	delay_ms(120);	  // Delay 120ms
	LCD_WR_REG(0xB1); // Normal mode
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2); // Idle mode
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3); // Partial mode
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB4); // Dot inversion
	LCD_WR_DATA8(0x03);
	LCD_WR_REG(0xC0); // AVDD GVDD
	LCD_WR_DATA8(0xAB);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);	// VGH VGL
	LCD_WR_DATA8(0xC5); // C0
	LCD_WR_REG(0xC2);	// Normal Mode
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3); // Idle
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x6A);
	LCD_WR_REG(0xC4); // Partial+Full
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);
	LCD_WR_REG(0xC5); // VCOM
	LCD_WR_DATA8(0x0F);
	LCD_WR_REG(0xE0); // positive gamma
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
	LCD_WR_REG(0xE1); // negative gamma
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
	LCD_WR_REG(0x36);
	if (USE_HORIZONTAL == 0)
		LCD_WR_DATA8(0x08);
	else if (USE_HORIZONTAL == 1)
		LCD_WR_DATA8(0xC8);
	else if (USE_HORIZONTAL == 2)
		LCD_WR_DATA8(0x78);
	else
		LCD_WR_DATA8(0xA8);
	LCD_WR_REG(0x21); // Display inversion
	LCD_WR_REG(0x29); // Display on
	LCD_WR_REG(0x2A); // Set Column Address
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x1A); // 26
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x69); // 105
	LCD_WR_REG(0x2B);	// Set Page Address
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01); // 1
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xA0); // 160
	LCD_WR_REG(0x2C);
}
