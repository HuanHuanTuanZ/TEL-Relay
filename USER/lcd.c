#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "delay.h"
#include "common.h"

u16 gram[LCD_W * LCD_H]; // ����

/******************************************************************************
	  ����˵������ָ�����������ɫ
	  ������ݣ�xsta,ysta   ��ʼ����
				xend,yend   ��ֹ����
								color       Ҫ������ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
	u16 i, j;
	LCD_Address_Set(xsta, ysta, xend - 1, yend - 1); // ������ʾ��Χ
	for (i = ysta; i < yend; i++)
	{
		for (j = xsta; j < xend; j++)
		{
			LCD_WR_DATA(color);
		}
	}
}

/******************************************************************************
	  ����˵������ָ��λ�û���
	  ������ݣ�x,y ��������
				color �����ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_DrawPoint(int16_t x, int16_t y, u16 color)
{
	// LCD_Address_Set(x, y, x, y); // ���ù��λ��
	// LCD_WR_DATA(color);
	if (x < 0 || x >= LCD_W || y >= LCD_H || y < 0)
		return;
	gram[y * LCD_W + x] = color;
}

/******************************************************************************
	  ����˵��������
	  ������ݣ�x1,y1   ��ʼ����
				x2,y2   ��ֹ����
				color   �ߵ���ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
	u16 t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	delta_x = x2 - x1; // ������������
	delta_y = y2 - y1;
	uRow = x1; // �����������
	uCol = y1;
	if (delta_x > 0)
		incx = 1; // ���õ�������
	else if (delta_x == 0)
		incx = 0; // ��ֱ��
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0)
		incy = 1;
	else if (delta_y == 0)
		incy = 0; // ˮƽ��
	else
	{
		incy = -1;
		delta_y = -delta_y;
	}
	if (delta_x > delta_y)
		distance = delta_x; // ѡȡ��������������
	else
		distance = delta_y;
	for (t = 0; t < distance + 1; t++)
	{
		LCD_DrawPoint(uRow, uCol, color); // ����
		xerr += delta_x;
		yerr += delta_y;
		if (xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/******************************************************************************
	  ����˵����������
	  ������ݣ�x1,y1   ��ʼ����
				x2,y2   ��ֹ����
				color   ���ε���ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
}

/******************************************************************************
	  ����˵������Բ
	  ������ݣ�x0,y0   Բ������
				r       �뾶
				color   Բ����ɫ
	  ����ֵ��  ��
******************************************************************************/
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color)
{
	int a, b;
	a = 0;
	b = r;
	while (a <= b)
	{
		LCD_DrawPoint(x0 - b, y0 - a, color); // 3
		LCD_DrawPoint(x0 + b, y0 - a, color); // 0
		LCD_DrawPoint(x0 - a, y0 + b, color); // 1
		LCD_DrawPoint(x0 - a, y0 - b, color); // 2
		LCD_DrawPoint(x0 + b, y0 + a, color); // 4
		LCD_DrawPoint(x0 + a, y0 - b, color); // 5
		LCD_DrawPoint(x0 + a, y0 + b, color); // 6
		LCD_DrawPoint(x0 - b, y0 + a, color); // 7
		a++;
		if ((a * a + b * b) > (r * r)) // �ж�Ҫ���ĵ��Ƿ��Զ
		{
			b--;
		}
	}
}

/******************************************************************************
	  ����˵������ʾ���ִ�
	  ������ݣ�x,y��ʾ����
				*s Ҫ��ʾ�ĺ��ִ�
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ� ��ѡ 16 24 32
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	/*
	while (*s != 0)
	{
		if (sizey == 12)
			LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
		else if (sizey == 16)
			LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
		else if (sizey == 24)
			LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
		else if (sizey == 32)
			LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
		else
			return;
		s += 2;
		x += sizey;
	}

	*/

	if (sizey == 12)
		LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
	else if (sizey == 16)
		LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
	else if (sizey == 24)
		LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
	// else if (sizey == 32)
	//	LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
	else
		return;
}

/******************************************************************************
	  ����˵������ʾ����12x12����
	  ������ݣ�x,y��ʾ����
				*s Ҫ��ʾ�ĺ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese12x12(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u8 i, j;
	u16 k;
	u16 HZnum;		 // ������Ŀ
	u16 TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	u16 x0 = x;
	TypefaceNum = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;
	HZnum = sizeof(tfont12) / sizeof(typFNT_GB12); // ͳ�ƺ�����Ŀ
	for (k = 0; k < HZnum; k++)
	{
		if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1)))
		{
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (tfont12[k].Msk[i] & (0x01 << j))
						LCD_DrawPoint(x, y, fc);
					else if (!mode)
						LCD_DrawPoint(x, y, bc);
					x++;
					if ((x - x0) == 12)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
			break;
		}
	}
}

/******************************************************************************
	  ����˵������ʾ����16x16����
	  ������ݣ�x,y��ʾ����
				*s Ҫ��ʾ�ĺ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese16x16(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u8 i, j;
	u16 k;
	u16 HZnum;		 // ������Ŀ
	u16 TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	int16 x0 = x;
	TypefaceNum = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;
	HZnum = sizeof(tfont16) / sizeof(typFNT_GB16); // ͳ�ƺ�����Ŀ
	for (k = 0; k < HZnum; k++)
	{
		if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1)))
		{
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (tfont16[k].Msk[i] & (0x01 << j))
						LCD_DrawPoint(x, y, fc);
					else if (!mode)
						LCD_DrawPoint(x, y, bc);
					x++;
					if ((x - x0) == 16)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
			break;
		}
	}
}

/******************************************************************************
	  ����˵������ʾ����24x24����
	  ������ݣ�x,y��ʾ����
				*s Ҫ��ʾ�ĺ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese24x24(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u8 i, j, m = 0;
	u16 k;
	u16 HZnum;		 // ������Ŀ
	u16 TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	u16 x0 = x;
	TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
	HZnum = sizeof(tfont24) / sizeof(typFNT_GB24); // ͳ�ƺ�����Ŀ
	for (k = 0; k < HZnum; k++)
	{
		if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1)))
		{
			LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (!mode) // �ǵ��ӷ�ʽ
					{
						if (tfont24[k].Msk[i] & (0x01 << j))
							LCD_WR_DATA(fc);
						else
							LCD_WR_DATA(bc);
						m++;
						if (m % sizey == 0)
						{
							m = 0;
							break;
						}
					}
					else // ���ӷ�ʽ
					{
						if (tfont24[k].Msk[i] & (0x01 << j))
							LCD_DrawPoint(x, y, fc); // ��һ����
						x++;
						if ((x - x0) == sizey)
						{
							x = x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue; // ���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}

/******************************************************************************
	  ����˵������ʾ����32x32����
	  ������ݣ�x,y��ʾ����
				*s Ҫ��ʾ�ĺ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChinese32x32(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u8 i, j, m = 0;
	u16 k;
	u16 HZnum;		 // ������Ŀ
	u16 TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	u16 x0 = x;
	TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
	HZnum = sizeof(tfont32) / sizeof(typFNT_GB32); // ͳ�ƺ�����Ŀ
	for (k = 0; k < HZnum; k++)
	{
		if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1)))
		{
			LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (!mode) // �ǵ��ӷ�ʽ
					{
						if (tfont32[k].Msk[i] & (0x01 << j))
							LCD_WR_DATA(fc);
						else
							LCD_WR_DATA(bc);
						m++;
						if (m % sizey == 0)
						{
							m = 0;
							break;
						}
					}
					else // ���ӷ�ʽ
					{
						if (tfont32[k].Msk[i] & (0x01 << j))
							LCD_DrawPoint(x, y, fc); // ��һ����
						x++;
						if ((x - x0) == sizey)
						{
							x = x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue; // ���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}

/******************************************************************************
	  ����˵������ʾ�����ַ�
	  ������ݣ�x,y��ʾ����
				num Ҫ��ʾ���ַ�
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	/*
	u8 temp, sizex, t, m = 0;
	u16 i, TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	u16 x0 = x;
	sizex = sizey / 2;
	TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
	num = num - ' ';									 // �õ�ƫ�ƺ��ֵ
	LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1); // ���ù��λ��
	for (i = 0; i < TypefaceNum; i++)
	{
		if (sizey == 12)
			temp = ascii_1206[num][i]; // ����6x12����
		else if (sizey == 16)
			temp = ascii_1608[num][i]; // ����8x16����
		else if (sizey == 24)
			temp = ascii_2412[num][i]; // ����12x24����
		else if (sizey == 32)
			temp = ascii_3216[num][i]; // ����16x32����
		else
			return;
		for (t = 0; t < 8; t++)
		{
			if (!mode) // �ǵ���ģʽ
			{
				if (temp & (0x01 << t))
					LCD_WR_DATA(fc);
				else
					LCD_WR_DATA(bc);
				m++;
				if (m % sizex == 0)
				{
					m = 0;
					break;
				}
			}
			else // ����ģʽ
			{
				if (temp & (0x01 << t))
					LCD_DrawPoint(x, y, fc); // ��һ����
				x++;
				if ((x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
		}
	}
	*/
	u16 i, TypefaceNum; // һ���ַ���ռ�ֽڴ�С
	u8 x0 = x, temp, t;
	u8 sizex = sizey / 2;
	TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
	num = num - ' '; // �õ�ƫ�ƺ��ֵ
	for (i = 0; i < TypefaceNum; i++)
	{
		if (sizey == 12)
			temp = ascii_1206[num][i]; // ����6x12����
		else if (sizey == 16)
			temp = ascii_1608[num][i]; // ����8x16����
		else if (sizey == 24)
			temp = ascii_2412[num][i]; // ����12x24����
		else if (sizey == 32)
			temp = ascii_3216[num][i]; // ����16x32����
		else
			return;
		for (t = 0; t < 8; t++)
		{
			if (!mode) // �ǵ���ģʽ
			{
				if (temp & (0x01 << t))
					LCD_DrawPoint(x, y, fc); // gram[y*160+x]=color1;
				else
					LCD_DrawPoint(x, y, bc);
				x++;
				if ((x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
			else // ����ģʽ
			{
				if (temp & (0x01 << t))
					LCD_DrawPoint(x, y, fc);
				x++;
				if ((x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
		}
	}
}

/******************************************************************************
	  ����˵������ʾ�ַ���
	  ������ݣ�x,y��ʾ����
				*p Ҫ��ʾ���ַ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	while (*p != '\0')
	{
		LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
		x += sizey / 2;
		p++;
	}
}

/******************************************************************************
	  ����˵������ʾ����
	  ������ݣ�m������nָ��
	  ����ֵ��  ��
******************************************************************************/
u32 mypow(u8 m, u8 n)
{
	u32 result = 1;
	while (n--)
		result *= m;
	return result;
}

/******************************************************************************
	  ����˵������ʾ��������
	  ������ݣ�x,y��ʾ����
				num Ҫ��ʾ��������
				len Ҫ��ʾ��λ��
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey)
{
	u8 t, temp;
	u8 enshow = 0;
	u8 sizex = sizey / 2;
	for (t = 0; t < len; t++)
	{
		temp = (num / mypow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
				continue;
			}
			else
				enshow = 1;
		}
		LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
	}
}

/******************************************************************************
	  ����˵������ʾ��λС������
	  ������ݣ�x,y��ʾ����
				num Ҫ��ʾС������
				len Ҫ��ʾ��λ��
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
	u8 t, temp, sizex;
	u16 num1;
	sizex = sizey / 2;
	num1 = num * 100;
	for (t = 0; t < len; t++)
	{
		temp = (num1 / mypow(10, len - t - 1)) % 10;
		if (t == (len - 2))
		{
			LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
			t++;
			len += 1;
		}
		LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
	}
}

/******************************************************************************
	  ����˵������ʾͼƬ
	  ������ݣ�x,y�������
				length ͼƬ����
				width  ͼƬ���
				pic[]  ͼƬ����
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[])
{
	u16 i, j;
	u32 k = 0;
	LCD_Address_Set(x, y, x + length - 1, y + width - 1);
	for (i = 0; i < length; i++)
	{
		for (j = 0; j < width; j++)
		{
			LCD_WR_DATA8(pic[k * 2]);
			LCD_WR_DATA8(pic[k * 2 + 1]);
			k++;
		}
	}
}

/******************************************************************************
	  ����˵������ʾ�ַ���(�Զ�������Ӣ��)
	  ������ݣ�x,y��ʾ����
				*str Ҫ��ʾ���ַ���
				fc �ֵ���ɫ
				bc �ֵı���ɫ
				sizey �ֺ�
				mode:  0�ǵ���ģʽ  1����ģʽ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowStr(u16 x, u16 y, u8 *str, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u16 x0 = x;
	u8 bHz = 0;		  // �ַ���������
	while (*str != 0) // ����δ����
	{
		if (!bHz)
		{
			if (*str > 0x80)
				bHz = 1; // ����
			else		 // �ַ�
			{
				if (*str == 13) // ���з���
				{
					y += sizey;
					x = x0;
					str++;
				}
				else
					LCD_ShowChar(x, y, *str, fc, bc, sizey, mode); // ��Ч����д��
				str++;
				x += sizey / 2; // �ַ�,Ϊȫ�ֵ�һ��
			}
		}
		else // ����
		{
			bHz = 0;										 // �к��ֿ�
			LCD_ShowChinese(x, y, str, fc, bc, sizey, mode); // ��ʾ�������,������ʾ
			str += 2;
			x += sizey; // ��һ������ƫ��
		}
	}
}

/******************************************************************************
	  ����˵��������д��
	  ������ݣ���
	  ����ֵ��  ��
******************************************************************************/
void LCD_Reflash()
{
	u16 i = 0;
	int sum = LCD_W * LCD_H;
	LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1);
	LCD_CS_Clr();
	for (i = 0; i < sum; i++)
	{
		// SPI_I2S_SendData(SPI2, color8_index[gram[i]] >> 8);
		// SPI_I2S_SendData(SPI2, gram[i] >> 8);
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		// SPI_I2S_SendData(SPI2, color8_index[gram[i]]);
		// SPI_I2S_SendData(SPI2, gram[i]);
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		//__nop();
		LCD_WR_DATA(gram[i]);
	}
	LCD_CS_Set();
}

/******************************************************************************
	  ����˵�������渲д
	  ������ݣ�color ��д��ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_Clear(u16 color)
{
	u16 *xs = gram;
	u32 count = LCD_W * LCD_H;
	while (count--)
		*xs++ = color;
}

/******************************************************************************
	  ����˵��������ʵ�ľ���
	  ������ݣ�xsta,ysta   ��ʼ����
				xend,yend   ��ֹ����
								color       Ҫ������ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_FillRect(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
	u16 x,y;
	for (y = ysta; y <= yend; y++)
	{
		for (x = xsta; x <= xend; x++)
		{
			LCD_DrawPoint(x, y, color);
		}
	}
}

/******************************************************************************
	  ����˵��������ʵ��Բ�Ǿ���
	  ������ݣ�xsta,ysta   ��ʼ����
				xend,yend   ��ֹ����
				r 	Բ�ǰ뾶
				color       Ҫ������ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowFillRoundRect(u16 xsta, u16 ysta, u16 xend, u16 yend, u8 r, u16 color)
{
	int a;
	int di;
	u16 x, y;
	x = xend - r;
	y = yend - r;
	a = 0;
	di = 3 - (r << 1);
	while (a <= r)
	{
		LCD_DrawLine(xsta + a + x, ysta - r - y, xsta + a + x, ysta + r + y, color);
		LCD_DrawLine(xsta + r + x, ysta - a - y, xsta + r + x, ysta + a + y, color);
		LCD_DrawLine(xsta - a - x, ysta - r - y, xsta - a - x, ysta + r + y, color);
		LCD_DrawLine(xsta - r - x, ysta - a - y, xsta - r - x, ysta + a + y, color);
		a++;

		if (di < 0)
			di += 4 * a + 6;
		else
		{
			di += 10 + 4 * (a - r);
			r--;
		}
	}
	LCD_FillRect(xsta - x, ysta - yend, xsta + x, ysta + yend, color);
}

/******************************************************************************
	  ����˵��������Բ�Ǿ���
	  ������ݣ�xsta,ysta   ��ʼ����
				xend,yend   ��ֹ����
				r 	Բ�ǰ뾶
				color       ��ɫ
	  ����ֵ��  ��
******************************************************************************/
void LCD_ShowRoundRect(u16 xsta, u16 ysta, u16 xend, u16 yend, u8 r, u16 color)
{
	int a;
	int di;
	u16 x, y;
	x = xend - r;
	y = yend - r;
	a = 0;
	di = 3 - (r << 1);
	while (a <= r)
	{
		LCD_DrawPoint(xsta + a + x, ysta - r - y, color);
		LCD_DrawPoint(xsta + r + x, ysta - a - y, color);
		LCD_DrawPoint(xsta + r + x, ysta + a + y, color);
		LCD_DrawPoint(xsta + a + x, ysta + r + y, color);
		LCD_DrawPoint(xsta - a - x, ysta + r + y, color);
		LCD_DrawPoint(xsta - r - x, ysta + a + y, color);
		LCD_DrawPoint(xsta - a - x, ysta - r - y, color);
		LCD_DrawPoint(xsta - r - x, ysta - a - y, color);
		a++;
		if (di < 0)
			di += 4 * a + 6;
		else
		{
			di += 10 + 4 * (a - r);
			r--;
		}
	}
	LCD_DrawLine(xsta - x, ysta - yend, xsta + x, ysta - yend, color);
	LCD_DrawLine(xsta - x, ysta + yend, xsta + x, ysta + yend, color);
	LCD_DrawLine(xsta - xend, ysta - y, xsta - xend, ysta + y, color);
	LCD_DrawLine(xsta + xend, ysta - y, xsta + xend, ysta + y, color);
}
