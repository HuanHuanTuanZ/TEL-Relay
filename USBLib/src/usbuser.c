

#include "xuan.h"

#if USB

/*************************************************************
*	name: usb_hw.c
*	mail:findstr@sina.com
* 	(c) findstr	2012 3/10
*************************************************************/
#include "stm32f10x.h"
#include "usbreg.h"
#include "usbcfg.h"
#include "usbcore.h"

#include "usbuser.h" 
#include "stdio.h"
#include "usb_hw.h"

#include "key.h"

/*ʹ�ô˺����Զ����ɶ˵㺯��
 *������һ��ֵ��ʧ�����ǣ�USB_EP_EVENT Ϊ0x07
 *������n>3ʱ���ͻ�ֱ�ӷ���ΪNULL����֪��keil�Ŀ�������Ϊ
 */
//#define	P_EP(n)	((USB_EP_EVENT & (1<< (n))) ? USB_EndPoint##n:NULL)
#define	P_EP(n)	((USB_EP_EVENT & (n)||(!n)) ? USB_EndPoint##n : NULL)

/*ʹ��P_EP(n)�����ɶ˵�ĺ�����	\
 *ֻ�ö˵�0 -->����ö��
 *�Ͷ˵�1   -->������������
 */
void (* const USB_P_EP[16])(DWORD event)=
{
	P_EP(0),
	P_EP(1)
} ;

void USB_EndPoint1 (DWORD event)
{
	if (event == USB_EVT_IN) {
//		printf("�˵�1USB_EVT_IN\r\n");
// 		if (key_press_status()) {
// 			u8 buf[2]={0}; 
// 			USB_WriteEP(0x81,buf,2);
// 		}
	}
	else
	{
		u8 buf[4]={0};  
//		printf("�˵�1USB_EVT_OUT\r\n");
		USB_ReadEP(0x01,buf,4);	
	}
}

#endif


