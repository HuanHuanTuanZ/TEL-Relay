#include "Ctrl.h"

#if USB
/*************************************************************
*	name: usb_hw.c
*	mail:findstr@sina.com
* 	(c) findstr	2012 3/9
*************************************************************/
#include "stm32f10x.h"

#include "usbreg.h"
#include "usbuser.h"
#include "usbcore.h"
#include "usb_hw.h"
#include "stdio.h"
#include "usbcfg.h"

/*�˵㻺�����Ŀ�ʼ��ַ
 *��Ϊÿ������鶼��Ҫһ���˵�������
 *�����еĶ˵���������ڣ�USB���������ײ�
 *�˵�ַ�������USB�������ĵ�ַ������Ϊ����Offset����Щ
 *����ʹ��2���˵�
 *�˵�0��˵�1
 *��ʱEP_BUF_ADDRָ�򻺳���������
 */
#define	EP_BUF_ADDR	(sizeof(EP_BUF_DSCR)*USB_EP_NUM)

/*USB�������׵�ַ����������������,���Ե�ַ*/
EP_BUF_DSCR	* pBUF_DSCR = (EP_BUF_DSCR *) USB_PMA_ADDR;

/*�˵���л�������ַ 
 *����ָʾĿǰΪֹUSB�������л�û�з���Ŀ��е�ַ���׵�ַ*/
WORD	FreeBufAddr;

	
/*����:���ڳ�ʼ��USB��ʱ�ӵȲ���
 *����:��
 *����ֵ����
 */
void USB_Init(void)
{
	//printf("����USB_Init�����г�ʼ��\r\n");
	//ʹ��USBʱ��
	RCC->APB1ENR |= (1<<23);

	//ʹ��USB�ж�
	/*��ΪUSB�����ȼ��жϵ��жϺ�Ϊ20����NVIC����IPRX
	 *�Ĵ�������λ���洢�ж����ȼ�������20/4��5 ,
	 *Ȼ��ʹ�ܵ�20λ�ж�*/
	NVIC->IP[5] |=0x10;
	NVIC->ISER[0]|=(1<<20);
}
/*����:���ڸ�λUSBģ��	  
 *����:��
 *����ֵ����
 */
/*�������ҵ�ˮƽ���㲻��˫����Ϊ��������Ȳ���^-^*/
/*һЩ���ϣ�
  *USB�����ȼ��ж�(ͨ��20):��������USB�¼�����(��ȷ����,USB��λ��).
  *USB�����ȼ��ж�(ͨ��19):������ͬ����˫�������������¼�����,Ŀ���Ǳ�֤���Ĵ�������.
  *USB�����ж�(ͨ��42)����USB����ģʽ�Ļ����¼�����.  OTG_FS_WKUP����
  *
  *��λҪִ�е����ݿ��Բμ�rm0008 21.4.2��
  */
extern const BYTE USB_DeviceDescriptor[];
void USB_Reset(void)
{
	//printf("USB_Reset\r\n");
	/*��λ�����������ǰ�������ж϶�û�����ˣ��������£�*/
	ISTR=0;

	/*ͨ������CNTR������stm32��USBģ��Ĺ�����ʽ
	 *���е�USB�¼��ж϶����ڵ����ȼ��ж�(ͨ��20)�ϴ����
	 *�ðɾ���ʹ����ô��ɣ�����������˵��
	 */
	CNTR= 		CNTR_CTRM	|	// ʹ����ȷ�����ж�
		 	CNTR_RESETM 	|	//ʹ�ܸ�λ�ж�
			CNTR_SUSPM	|	//ʹ�ܹ����ж�
			CNTR_WKUPM	;	//ʹ�ܻ����ж�
	
	FreeBufAddr	=	EP_BUF_ADDR; //��ʱFreeBuffָ���һ���������׵�ַ(��������������)����Ե�ַ

	BTABLE	= 0x00;				 //���û������������λ��������Ե�ַ
	
	/*Ϊ�˵�0���û����������ֿ���λ*/
	pBUF_DSCR->ADDR_TX = FreeBufAddr;
	FreeBufAddr += USB_MAX_PACKET0;		//�˵�0����Ϊ8���ֽڣ�һ���������Ϊ8���ֽ�
	pBUF_DSCR->ADDR_RX = FreeBufAddr;
	FreeBufAddr += USB_MAX_PACKET0;
	
	/*��count_Rx�ֶ���10~14bit������ʾ�������ֽڵĿ���
	 *��15bit������ʾ��Ĵ�С
	 *0---2byte
	 *1---1byte
	 *��������ʹ����8���ֽڣ�bit15Ϊ0,����Ӧ��((8<<10)>>1)��8<<9;
	 *����count_Rx�����ڷ���ʱ������ֵ
	 */
	pBUF_DSCR->COUNT_RX= USB_MAX_PACKET0 << 9;	
	/*���ö˵�0Ϊ���ƶ˵㣬���ջ�������Ч
	 *����λ����˵��ַ
	 */
	EPxREG(0) = EP_CONTROL | EP_RX_VALID;

	/*ʹ��USBģ��,������USB��ַΪ0������Ӧö��*/
	DADDR = DADDR_EF | 0;
	//printf("Leave USB_Reset\r\n");
}

/*����:��λһ���˵�
 *����:	�˵��
 *				EPNum:bit3~bit0	----> �˵��
 * 				EPNum:bit7		----> �˵㷽��
 *
 *����ֵ:��
 */
 /*��ʵ���ǽ���һ��Ҫ���͵����ݰ����DATA0*/
 void	EP_Reset(DWORD	EPNum)
 {
 	DWORD	num,var;
   	//printf("EP_Reset\r\n");
	/*��ö˵��,����λΪ�˵��*/
	num = EPNum & 0x0F;
	var = EPxREG(num);
	/*���bit7Ϊ1�򽫴˶˵�ķ���toggle��Ϊ0��
	 *���򽫴˶˵�Ľ���toggle��Ϊ0
	 *��Ϊ�������Ǵ�data0���ݰ���ʼ���͵�
	 */
	 if(EPNum & 0x80)
	 	EPxREG(num) = var & (EP_MASK | EP_DTOG_TX);/*����˵�*/
	 else
	 	EPxREG(num) = var & (EP_MASK | EP_DTOG_RX);/*����˵�*/
	 
 }
 /*���ܣ����ӻ�Ͽ�USB����
  *������true -->����USB
  *      false-->�ر�USB
  *����ֵ:��
  */
void USB_Connect(BOOL turnon)
{
	/*��Ҫע��һ����£����е�USB�Ĵ��������ã�����Ҫ������
	 *�ڱ���ֲ�������ȷ�������������ܻᵼ����һЩ����*/
	//printf("��������USB����\r\n");
	 /*��USBǿ�Ƹ�λ*/
	CNTR = CNTR_FRES;
	//printf("test1\r\n");
	/*��Ϊ����������Ӧ�ø�������һ����������û�д�����жϸ������*/
	ISTR=0;
	//printf("test2\r\n");
	if(turnon)
	{
		//printf("test3\r\n");
		/*ʹ��GPIOA,Ȼ��PA.8�õ�,ʹUSB
		*��D+��1.5K�Ͻӵ���,ʹUSB������ʶ���˸����豸
		*����������USBʶ��
		*/
		RCC->APB2ENR |= (1 << 2);                 	 /* enable clock for GPIOA */
		GPIOA->CRH &= ~(0x0f << 0 * 4);                /* clear port PA8 */
		GPIOA->CRH |=  (0x03 << 0 * 4);                /* PA6 General purpose output open-drain, max speed 50 MHz */
		GPIOA->BRR  =  (   1 << 8    );                /* reset PA8  (set to low) */	 	
		/*�������Է��֣�������ı����ǣ���λUSBģ��
		*Ȼ���ڴ�ʹ��CNTR_RESETM����λ�жϱ�־
		*���ڶ˵�0�ĳ�ʼ������USB�����ȼ��н��д���
		*��Ȼ������Ҳֻ���˵����ȼ��ж�^_^!*/
		CNTR = CNTR_RESETM;	 /*�˴�ֻʹ���˸�λ�жϣ�*/	 
	}
	else
		CNTR = CNTR_FRES | CNTR_PDWN ;/*��λ���ر�*/
	

}
/*����:���ö˵�״̬
 *����:EPnum --->�˵��
 *     stat  --->Ҫ���õ�״ֵ̬
 *����ֵ:��
 */
void USB_ConfigEP (USB_ENDPOINT_DESCRIPTOR * pEPD)
{
 	DWORD num,val;
	
	//ȡ�ö˵��
	num = pEPD->bEndpointAddress & 0xf;

	val = pEPD->wMaxPacketSize;
	//�����IN�˵�
	if(pEPD->bEndpointAddress & USB_ENDPOINT_DIRECTION_MASK)
	{
		//�˴���ֻ��˵pBUF_DSCR��ָ��,ʣ�µľ����﷨������
		(pBUF_DSCR + num)->ADDR_TX = FreeBufAddr;
		/*ȡ2�ı���,��Ϊ�����������ֶ����,ע��˴��������1023������˷�����
		 *��ΪUSB_COUNTn_TXֻ�ܽ���bit0~bit9
		 */
		val = (val + 1)& ~1;
	}
	//����˵�
	else
	{
		(pBUF_DSCR + num)->ADDR_RX = FreeBufAddr;
		/*��ΪUSB_COUNTn_RX�д洢ֻ��bit10~bit14,���BLSIZE=0(�����СΪ2�ֽ�),��ôֻ����0~62���ֽ�
		 *�����������62,��Ӧ�����С����ΪBLSIZE=1(��32���ֽ�)
		 */
		if(val > 62	)
		{
			//���СΪ32,���СӦ��Ϊ32�ı���
			val = (val +31)&~31;
			/*���ڴ˼��㹫ʽ,�μ�rm0008,21,5.3��
			 *(val >> 5)<<10 == val <<5
			 */
			(pBUF_DSCR + num)->COUNT_RX = ((val << 5)-1) | 0x8000;
		}
		else
		{
			val = (val + 1) & ~1;
			(pBUF_DSCR + num)->COUNT_RX = val << 9;
		}
	}
	//���������������ʼ��ַ
	FreeBufAddr += val ;

	switch(pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK)
	{
		//���ƶ˵�
		case USB_ENDPOINT_TYPE_CONTROL:
			val = EP_CONTROL;
			break;
		//ͬ���˵�
		case USB_ENDPOINT_TYPE_ISOCHRONOUS:
			val = EP_ISOCHRONOUS;
			break;
		//�鴫��
		case USB_ENDPOINT_TYPE_INTERRUPT:
			val = EP_INTERRUPT;
			break;
		default:
			//printf("������,δʶ��Ķ˵�����\r\n");
			break;
	}
	val |= num;
	//���ö˵�Ĵ���
	EPxREG(num) = val;
 }
/*����:���ö˵�״̬
 *����:EPnum --->�˵��
 *     stat  --->Ҫ���õ�״ֵ̬
 *����ֵ:��
 */
 void EP_Status(DWORD EPNum,DWORD stat)
 {
 	DWORD num,var;
	
	/*ȡ�ö˵��*/
	num = EPNum & 0x0f;
	var = EPxREG(num);
	/*�˴�����һ��С����,��Ϊ�˵�Ĵ�����д1��ת,������������Ӧ��ֵֻ��ʹ�����*/
	if(EPNum & 0x80)	//����˵�
		EPxREG(num)=(var ^ (stat & EP_STAT_TX)) & (EP_MASK | EP_STAT_TX);
	else				//����˵�
		EPxREG(num)=(var ^ (stat & EP_STAT_RX)) & (EP_MASK | EP_STAT_RX);

 }
/*����:��λ�˵�
 *������EPNum 	bit0~bit3Ϊ�˵��
 			  	bit7	 Ϊ�˵㷽��
 *����ֵ:��
 */
void USB_ResetEP(DWORD EPNum)
{
	EP_Reset(EPNum);
}
/*����:���ö˵�Ϊstall
 *������EPNum 	bit0~bit3Ϊ�˵��
 			  	bit7	 Ϊ�˵㷽��
 *����ֵ:��
 */
void USB_SetStallEP(DWORD	EPNum)
{
	EP_Status(EPNum,EP_TX_STALL | EP_RX_STALL);
}
/*����:���ö˵�Ϊenable
 *������EPNum 	bit0~bit3Ϊ�˵��
 			  	bit7	 Ϊ�˵㷽��
 *����ֵ:��
 */
void USB_EnableEP(DWORD EPNum)
{
	EP_Status(EPNum,EP_TX_VALID | EP_RX_VALID);
}

/*����:���ö˵�Ϊdisable
 *������EPNum 	bit0~bit3Ϊ�˵��
 			  	bit7	 Ϊ�˵㷽��
 *����ֵ:��
 */
void USB_DisableEP(DWORD EPNum)
{
	EP_Status(EPNum,EP_TX_DIS | EP_RX_DIS);
}

/*����:����˵��stall״̬
 *������EPNum 	bit0~bit3Ϊ�˵��
 			  	bit7	 Ϊ�˵㷽��
 *����ֵ:��
 */
void USB_ClrStallEP(DWORD	EPNum)
{
	EP_Status(EPNum,EP_TX_VALID | EP_RX_VALID);
}
/*����:����USB��ַ
 *������addrҪ���õĵ�ַ
 *����ֵ:��
 */
void USB_SetAddress(DWORD	addr)
{
	//DADDR�ĸ�1λ������ʹ��USBģ���
	DADDR = DADDR_EF | addr;
}
/*����:���ڶ��˵㻺����
 *����:EPnum --->�˵��
 *     pData --->���ڽ��մӶ˵㻺�����ж���������
 *		�˺�����Щ����,Ӧ�ü���һ��������ʾ�������ж��
 *����ֵ:�������ֽ���
 */
DWORD USB_ReadEP(DWORD EPnum,BYTE * pData,DWORD cnt)
{

	//DWORD num,cnt,*pv,n;
DWORD num,*pv,n;	
	//�õ��˵��    
	num = EPnum & 0xf;

	//ȡ��Щ�˵�Ļ�������ַ
	pv=(DWORD *)(USB_PMA_ADDR + 2* ((pBUF_DSCR + num)->ADDR_RX));
	//COUNT_RX��10λ��ŵ��ǻ�����������
	cnt=(pBUF_DSCR + num)->COUNT_RX & EP_COUNT_MASK;
	for(n=0;n<(cnt+1)/2;n++)
	{
		/*pakced�ؼ������ڵ��ֽڶ���,����USB���ݽṹ�еĽṹ������Ϊ��Ҫ
		 *��Ϊstm32����ʱʹ��32λ,��USB����ʱʹ��16λ,����pDataΪWORD,��
		 *pvΪDWORD��ָ��
		 */
	 	*((__packed WORD *)pData)=*pv++;
		/*����pDataΪ���ֽ�ָ������,���Ǽ�2�����Ǽ�4*/
		pData+=2;
	}
	/*OK,�������ǵĶ˵��ֿ��Խ���������,����ΪVALID*/
	EP_Status (EPnum,EP_RX_VALID);
   	return cnt;
}

/*����:����д�˵㻺����
 *����:EPNum --->�˵��
 *     pData --->ָ��Ҫ���͵����ݻ�����
 *	   cnt	 --->Ҫд����ֽ���
 *����ֵ:д����ֽ���
 */
 DWORD USB_WriteEP(DWORD EPNum , BYTE * pData,DWORD cnt)
 {
 	DWORD num,*pv,n;

	num = EPNum & 0x0f;

  	//pv=(DWORD *)(USB_PMA_ADDR + 2*((pBUF_DSCR+num)->ADDR_TX));
	 pv=(DWORD *)(USB_PMA_ADDR + 2*((pBUF_DSCR+num)->ADDR_TX));

	/*�˴�Ӧ���ж�Ҫд��������Ƿ�����,���ܻ��һ������bug*/
   	for(n=0;n<(cnt + 1)/2;n++)
	{
		*pv++=*((__packed WORD*)pData);
		pData+=2;
	}
	//OK,����USB���ͻ��������Ѿ��ж�����,������Ӧ������
	(pBUF_DSCR+num)->COUNT_TX=cnt;
	EP_Status(EPNum,EP_TX_VALID);
	return 	cnt;
 }

/*����:USB����
 *��������
 *����ֵ:��
 */
void USB_Suspend(void)
{
	//printf("��������ж�\r\n");
	//ǿ�ƹ���
	CNTR |= CNTR_FSUSP;	
	//����͹���ģʽ
	CNTR |= CNTR_LPMODE;
}

/*����:USB����
 *��������
 *����ֵ:��
 */
void USB_WakeUp(void)
{
 	GPIOE->BRR |=1 ;            /* Turn On Suspend LED */
	//printf("���뻽���ж�\r\n");
	//������,��Ȼ�ð���һλ������
	CNTR &=  ~CNTR_FSUSP;	
	//USB�Ļ����¼��Ḵλ��λ,�������ﲻ�ù�
	//CNTR &= ~CNTR_LPMODE;
}
/*����:USB�����ȼ��жϷ������
 *��������
 *����ֵ:��
 */
 void USB_LP_CAN1_RX0_IRQHandler(void)
 {
 	DWORD	istr;
	DWORD 	num,var;

	istr=ISTR;	//ȡ���жϱ�־λ
	/*USB��λ�жϵĴ���*/
	if(istr & ISTR_RESET)
	{
		//��λ�豸
		USB_Reset();
		//��λ��USBЭ���йص�����
		USB_ResetCore();
		ISTR = ~ISTR_RESET;	/*�Ѿ������긴λ�ж��ˣ������λ�жϱ�־*/
	}
	/*USB�����ж�*/
/**********************************************************************/
	if (istr & ISTR_SUSP) {
		USB_Suspend();
		ISTR = ~ISTR_SUSP;
	}

	/* USB Wakeup */
	if (istr & ISTR_WKUP) {
		USB_WakeUp();
		ISTR = ~ISTR_WKUP;
	}

//******************************************************************
	/*�˵���з����жϴ���*/
	while((istr = ISTR) & ISTR_CTR) 
	{
		//����жϱ�־λ
		ISTR = ~ISTR_CTR;
		//ȡ�÷����жϵĶ˵��
		num=istr & ISTR_EP_ID;
		//ȡ�ö˵�Ĵ�����ֵ		
		var = EPxREG(num);
		//��ȷ�Ľ��մ���
		if(var & EP_CTR_RX )
		{
			//��0��ȷ���ձ�־λ
		//	printf("�˵��Ϊ:");
		//	printhex((u8)num);
			EPxREG(num) = var & ~EP_CTR_RX & EP_MASK;
			//������Ӧ�Ķ˵���д���
			if(USB_P_EP[num])
			{
				//����ǿ��ƴ���,��ʹ��USG_EVT_SETUP
				if(var & EP_SETUP)
					USB_P_EP[num](USB_EVT_SETUP);
				else
					//���������ͨ�������
					USB_P_EP[num](USB_EVT_OUT);
			}
		}
		//�������Ƿ��ͳɹ��ж�
		if(var & EP_CTR_TX)
		{
			//����жϱ�־λ
			EPxREG(num) = var & ~EP_CTR_TX & EP_MASK;
			//���ö�Ӧ���жϺ������д���
			if(USB_P_EP[num])
			{
				//USB���ͳɹ�
				USB_P_EP[num](USB_EVT_IN);	
			}
		}

	}
 }
#endif
