/*************************************************************
*	name: usb_hw.c
*	mail:findstr@sina.com
* 	(c) findstr	2012 3/15
*************************************************************/
#include "stm32f10x.h"
#include "usbreg.h"

#ifndef 	  	_USB_USER_H__
#define	  		_USB_USER_H__

//�궨��
/* USB Endpoint Callback Events */
#define USB_EVT_SETUP       1   /* Setup Packet */
#define USB_EVT_OUT         2   /* OUT Packet */
#define USB_EVT_IN          3   /*  IN Packet */
#define USB_EVT_OUT_NAK     4   /* OUT Packet - Not Acknowledged */
#define USB_EVT_IN_NAK      5   /*  IN Packet - Not Acknowledged */
#define USB_EVT_OUT_STALL   6   /* OUT Packet - Stalled */
#define USB_EVT_IN_STALL    7   /*  IN Packet - Stalled */
#define USB_EVT_OUT_DMA_EOT 8   /* DMA OUT EP - End of Transfer */
#define USB_EVT_IN_DMA_EOT  9   /* DMA  IN EP - End of Transfer */
#define USB_EVT_OUT_DMA_NDR 10  /* DMA OUT EP - New Descriptor Request */
#define USB_EVT_IN_DMA_NDR  11  /* DMA  IN EP - New Descriptor Request */
#define USB_EVT_OUT_DMA_ERR 12  /* DMA OUT EP - Error */
#define USB_EVT_IN_DMA_ERR  13  /* DMA  IN EP - Error */

//��������
extern void (* const USB_P_EP[16])(DWORD event)	;


void USB_EndPoint1 (DWORD event);
#endif


