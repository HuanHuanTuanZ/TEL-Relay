#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 


#define ppm             PBout(10)

#define menu_up         PBin(5)
#define menu_down       PBin(4)
#define menu_left       PBin(6)
#define menu_right      PAin(15)
#define menu_ok         PBin(3)
#define menu_esc        PBin(2)  //PB12 ---> PB2


//#define ch7        PBin(2)
//#define ch8        PCin(15)
//#define ch9        PCin(14)
//#define ch10       PCin(13)


#define sport_dir       PBout(7)


void Gpio_Init(void);//��ʼ��
void ch7_init(void);
		 				    
#endif
