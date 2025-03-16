
#include "Ctrl.h"

#if USB

/*************************************************************
 *	name: usb_hw.c
 *	mail:findstr@sina.com
 * 	(c) findstr	2012 3/15
 *************************************************************/
#include "stm32f10x.h"
#include "usbreg.h"
#include "usbcore.h"
#include "usbuser.h"
#include "usbcfg.h"
#include "usb.h"
#include "usb_hw.h"
#include "usbdesc.h"
#include "hid.h"
#include "hiduser.h"

#include "stdio.h"

#pragma diag_suppress 111, 1441

// ����ָʾUSB�豸��״̬
static WORD USB_DeviceStatus;
// �����洢�豸�ĵ�ַ
static BYTE USB_DeviceAddress;
// �����洢USB��ǰʹ�õ��豸������
BYTE USB_Configuration;
// ������ʹ�ö˵�
static DWORD USB_EndPointMask;
// ���ڱ�־�˶˵��Ƿ��Ѿ���ֹͣ0~15���δ���15���˵�
static DWORD USB_EndPointHalt;
// ������ʹ�õĽӿ���
static BYTE USB_NumInterfaces;
// ÿ���ӿڿ��õĵ�ǰ�ӿڿ��滻ֵ
// static BYTE  USB_AltSetting[USB_IF_NUM];

/*������ʱ�洢���ƴ���ʱ�����ݰ�*/
USB_SETUP_PACKET SetupPacket;

/*�����������������ݵ�EP0�����ݽṹ*/
USB_EP_DATA EP0Data;
/*EP0Buf������USB��������ʱ�õĻ�����*/
BYTE EP0Buf[USB_MAX_PACKET0];

/*����:��λUSB��һЩ���ݱ�־
 *����:��
 *����ֵ:��
 */
void USB_ResetCore(void)
{
	// Ĭ��Ϊ���߹���,��Ϊ���ǵ�USB���ڲ����ǲ��ڵ����ϲ��ܹ�����ô&^_^
	USB_DeviceStatus = USB_POWER;
	// ��ö��֮����ַ��Ȼ��0
	USB_DeviceAddress = 0;
	// �����������ı�ʶ��1��ʼ,����Ҳ����Ϊ0
	USB_Configuration = 0;
	// Ŀǰʹ�õ��Ƕ˵㣰
	USB_EndPointMask = 0x00010001;
	// û��ֹͣ�Ķ˵�
	USB_EndPointHalt = 0x00000000;
}

/*����:�����׶�,��ȡ�������ݰ�
 *����:��
 *����ֵ:��
 */
void USB_SetupStage(void)
{
	USB_ReadEP(0x00, (BYTE *)&SetupPacket, 0);
}
/*����:�����׶�,In���ְ�
 *����:��
 *����ֵ:��
 */
void USB_StatusInStage(void)
{
	USB_WriteEP(0x80, NULL, 0);
}
/*����:�����׶�,Out���ְ�
 *����:��
 *����ֵ:��
 */
void USB_StatusOutStage(void)
{
	//	 USB_ReadEP(0x00,EP0Buf);
	USB_ReadEP(0x00, EP0Buf, 0);
}
/*����:����In�׶�
 *����:��
 *����ֵ:��
 */
void USB_DataInStage(void)
{
	DWORD cnt;
	// �ȼ�������Ҫ���Ͷ�������
	if (EP0Data.Count > USB_MAX_PACKET0)
		cnt = USB_MAX_PACKET0;
	else
		cnt = EP0Data.Count;
	// ����д�˵�ȴ������,��Dirλ��1
	cnt = USB_WriteEP(0x80, EP0Data.pData, cnt);
	EP0Data.pData += cnt;
	EP0Data.Count -= cnt;
}
/*����:����Out�׶�
 *����:��
 *����ֵ:��
 */
void USB_DataOutStage(void)
{
	DWORD cnt;

	//	 cnt	=	USB_ReadEP(0x00,EP0Data.pData);
	cnt = USB_ReadEP(0x00, EP0Data.pData, 0);
	EP0Data.pData += cnt;
	EP0Data.Count -= cnt;
}
/*����:��ȡUSB�豸��״̬
 *����:��
 *����ֵ:TRUE	--->�ɹ�
 *		 FALSE	--->����
 */
static __inline BOOL USB_GetStatus(void)
{
	DWORD n, m;

	switch (SetupPacket.bmRequestType.BM.Recipient)
	{
	// ���ն����豸
	case REQUEST_TO_DEVICE:
		// �����豸״̬����
		EP0Data.pData = (BYTE *)&USB_DeviceStatus;
		// ��״̬��Ϣ���͸�����
		USB_DataInStage();
		break;
	// ���ն��ǽӿ�
	case REQUEST_TO_INTERFACE:
		/*�����������ı�ʶ��1��ʼ,��������ĽӿںŲ��ܴ��ڽӿڵ���Ŀ,��Ϊ�ӿ���Ŀ��0��ʼ
		 *��Ϊ���ǽӿ��������еĽӿں���һ���ֽ���������wIndex�е������е��ֽ���Ч
		 */
		if ((USB_Configuration != 0) && (SetupPacket.wIndex.WB.L < USB_NumInterfaces))
		{
			// ��0�����ֽ�,��Ϊ����USBЭ��˴����뷵��0
			*((__packed WORD *)EP0Buf) = 0;
			EP0Data.pData = EP0Buf;
			// ���͸�����
			USB_DataInStage();
		}
		// �˽ӿڳ����˴���
		else
			return FALSE;
		break;
	case REQUEST_TO_ENDPOINT:
		// �˵�Ÿ�1λ0����,��4λΪ�˵��
		n = SetupPacket.wIndex.WB.L & 0x8f;
		// m�ĸ�16λ����in�˵�ı�־,��16λ����out�˵�ı�־
		m = (n & 0x80) ? (1 << 16) << (n & 0x0f) : (1 << n);
		// ������ñ�ʶ��Ϊ0,�����ñ�ʶΪ0,�����Ƕ˵�0ʱ,��������
		if ((USB_Configuration != 0) || ((n & 0x0f) == 0) && (USB_EndPointMask & m))
		{
			// �鿴�˶˵��Ƿ��Ѿ�ͣ��
			*((__packed WORD *)EP0Buf) = (USB_EndPointHalt & m) ? 1 : 0;
			EP0Data.pData = EP0Buf;
			// �����ݷ��͸�����
			USB_DataInStage();
		}
		// ˵��������������������
		else
			return FALSE;
		break;
	default:
		return FALSE;
	}
	return TRUE;
}
/*����:����/���USB�豸������
 *����:sc
			0----->���
			1----->����
 *����ֵ:TRUE	--->�ɹ�
 *		 FALSE	--->����
 */
static __inline BOOL USB_SetClrFeature(DWORD sc)
{
	DWORD n, m;
	switch (SetupPacket.bmRequestType.BM.Recipient)
	{
	// ���ն����豸,������������豸������
	case REQUEST_TO_DEVICE:
		if (SetupPacket.wValue.W == USB_FEATURE_REMOTE_WAKEUP)
		{
			if (sc)
			{
				printf("�����豸Զ�̻�������\r\n");
				// ����USB״̬Ϊʹ��Զ�̻���
				USB_DeviceStatus |= USB_GETSTATUS_REMOTE_WAKEUP;
				/*stm32Ӳ�������֧��Զ�̻���,����Ͳ���������
				 *��Ȼ,���֧��֧�����ڶ��ж�����λ������
				 */
			}
			else
			{
				printf("����豸Զ�̻�������\r\n");
				USB_DeviceStatus &= ~USB_GETSTATUS_REMOTE_WAKEUP;
				/*stm32Ӳ�������֧��Զ�̻���,����Ͳ���������
				 *��Ȼ,���֧��֧�����ڶ��ж�����λ������
				 */
			}
		}
		// ��֧�ֵ�����
		else
		{
			printf("��֧�ֵ�����\r\n");
			return FALSE;
		}
		break;
	// ���ն��ǽӿ�
	case REQUEST_TO_INTERFACE:
		// �ӿڲ�֧���������
		printf("������ýӿ���������USB2.0֧��\r\n");
		return FALSE;
	// ���ն��Ƕ˵�
	case REQUEST_TO_ENDPOINT:
		// ȡ�ö˵��
		n = SetupPacket.wIndex.WB.L & 0x8F;
		//			printf("����Ķ˵���:");
		//			printhex(n);
		//			printf("USB_Configuration:");
		//			printhex(USB_Configuration);
		//			printf("USB_EndPointMask");
		//			printhex(USB_EndPointMask);
		//			printf("\r\n");
		// ���˵�ת������EndPointMaskһ���ĸ�ʽ
		m = (n & 0x80) ? ((1 << 16) << (n & 0x0f)) : (1 << n);
		/*����USBЭ����˵,
		 *��ַ״̬��ֻ�ж˵�0����ʹ��,
		 *����״̬�¿���
		 *��֪��������ǲ���bug
		 *ԭ�ļ�����:
		 *This request is valid when the device is in the Address state;
		 *references to interfaces or to endpoints other than endpoint zero
		 *shall cause the device to respond with a Request Error
		 */
		/*	�˴���Ϊֻ������״̬�µķǣ��˵����������� �����ڵ�ַ״̬��û�д���
		 *
		 */
		if ((USB_Configuration != 0) && ((n & 0x0f) != 0) && (USB_EndPointMask & m))
		{
			// ����/��� ĳ���˵�stall״̬
			if (SetupPacket.wValue.W == USB_FEATURE_ENDPOINT_STALL)
			{
				// ����
				if (sc)
				{
					printf("���ö˵�STALL\r\n");
					// ��������Ķ˵�Ϊstall
					USB_SetStallEP(n);
					// ����USB��״̬λ
					USB_EndPointHalt |= m;
				}
				// ���
				else
				{
					printf("����˵�%dSTALL\r\n", n);
					USB_ClrStallEP(n);
					USB_EndPointHalt &= ~m;
				}
			}
			// δ���������
			else
			{
				printf("����ȷ�Ķ˵�����\r\n");
				return FALSE;
			}
		}
		// ����ȷ������ʱ��
		else
		{
			printf("����ȷ������ʱ��\r\n");
			return FALSE;
		}
		break;
	// δ����Ľ��ն�����
	default:
		printf("δ����Ľ��ն�����\r\n");
		printf("���ն�Ϊ:");
		printf("%x", SetupPacket.bmRequestType.BM.Recipient);
		printf("\r\n");
		return FALSE;
	}
	return TRUE;
}
/*����:��������������������
 *����:��
 *����ֵ:TRUE	--->�ɹ�
 *		 FALSE	--->�����˴���
 */
static __inline BOOL USB_GetDescriptor()
{
	BYTE *pD;
	DWORD len, n;

	switch (SetupPacket.bmRequestType.BM.Recipient)
	{
		// �����豸������
	case REQUEST_TO_DEVICE:
		switch (SetupPacket.wValue.WB.H)
		{
		// ��ȡ�豸������
		case USB_DEVICE_DESCRIPTOR_TYPE:
			printf("��ȡ�豸������\r\n");
			EP0Data.pData = (BYTE *)USB_DeviceDescriptor;
			len = USB_DEVICE_DESC_SIZE;
			break;
		// ��ȡ����������
		case USB_CONFIGURATION_DESCRIPTOR_TYPE:
			printf("��ȡUSB����������\r\n");
			pD = (BYTE *)USB_ConfigDescriptor;
			/* ����USBЭ��:The range of values used for a
			 * descriptor index is from 0 to one less than
			 * the number of descriptors of that type implemented by the device
			 * �����������еı�ʶ�Ǵ�1��ʼ�Ƶ�,����,�����е�ֵ�Ǳ�ʶֵΪ����������
			 */
			/*���ڿ����м����������������Կ�����Ҫ����һ��ָ��*/
			for (n = 0; n != SetupPacket.wValue.WB.L; n++)
			{
				if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bLength != 0)
				{
					pD += ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
				}
			}
			// ��������������Ϊ���򷵻�stall
			if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bLength == 0)
				return FALSE;
			// ����Ҫ���͵�ָ��
			EP0Data.pData = pD;
			// ��Ȼ�����������ж���һ���ֽ�,����wTotalLength��û������,�������ﲻ�ù�,ֱ���þ�����
			len = ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
			break;
		// �ַ���������
		case USB_STRING_DESCRIPTOR_TYPE:
			printf("��ȡ�ַ���������\r\n");
			EP0Data.pData = (BYTE *)USB_StringDescriptor + SetupPacket.wValue.WB.L;
			len = ((USB_STRING_DESCRIPTOR *)EP0Data.pData)->bLength;
			break;
		default:
			return FALSE;
		}
		break;
	// ������ն˽ӿ�
	case REQUEST_TO_INTERFACE:
		printf("������ն˽ӿ�\r\n");
		switch (SetupPacket.wValue.WB.H)
		{
		// Hid ������
		case HID_HID_DESCRIPTOR_TYPE:
			printf("����HID��\r\n");
			break;
		case HID_REPORT_DESCRIPTOR_TYPE:
			printf("���󱨸�������\r\n");
			// ����ֻ��һ������������,�����������������Ǵ�0��ʼ��,��������һ����,���ǲ���0�򷵻�Stall
			if (SetupPacket.wIndex.WB.L != 0)
				return FALSE;
			EP0Data.pData = (BYTE *)HID_ReportDescriptor;
			printf("����������������ֽ�:");
			printf("%x", EP0Data.Count);
			printf("���������������ĳ���:");
			printf("%x", HID_ReportDescSize);
			printf("\r\n");
			len = HID_ReportDescSize;
			break;
		default:
			printf("δʶ�������!!!###,������:");
			printf("%x", SetupPacket.wValue.WB.H);
			printf("\r\n");
			break;
		}
		break;
	default:
		printf("USB_GetDescriptor false\r\n");
		return FALSE;
	}

	printf("��ȡ�������ɹ�\r\n");
	// �����ݷ��͸�����
	if (EP0Data.Count > len)
		EP0Data.Count = len;
	USB_DataInStage();
	return TRUE;
}
/*����:������������
 *����:��
 *����ֵ:TRUE 	--- >�ɹ�
 *		 FALSE	--- >ʧ��
 */
static __inline BOOL USB_SetConfiguration(void)
{
	USB_COMMON_DESCRIPTOR *pD;
	DWORD alt, n, m;
	printf("��������\r\n");
	// ����ֵ����Ϊ0
	if (SetupPacket.wValue.WB.L)
	{
		pD = (USB_COMMON_DESCRIPTOR *)USB_ConfigDescriptor;
		// ѭ���������������е����һ���˵�������
		while (pD->bLength)
		{
			switch (pD->bDescriptorType)
			{
			// ��ʱ������������
			case USB_CONFIGURATION_DESCRIPTOR_TYPE:
				// ���������������������Ҫ���õ�����������
				if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bConfigurationValue == SetupPacket.wValue.WB.L)
				{
					// �������ʱ,�豸���õ�����ֵ
					USB_Configuration = SetupPacket.wValue.WB.L;
					// �������ʱ���豸���õ����õĽӿ���Ŀ
					USB_NumInterfaces = ((USB_CONFIGURATION_DESCRIPTOR *)pD)->bNumInterfaces;
					// �声ÿ���ӿڿ��滻����ֵ
					for (n = 0; n < USB_IF_NUM; n++)
					{
						// USB_AltSetting[n]=0;
					}
					// ���ǽ����е���ǰ����ʹ�õĶ˵���ã������Ҫ���ڶ˵�����������ʹ�ܾ�����
					// ����������������þͱ�֤��,����ʹ�õ���ǰ����ʹ��,����������û��ʹ�õĶ˵�
					// ������Ϊ�˷�ֹ����,�µ�^-^!
					for (n = 1; n < 16; n++)
					{
						if (USB_EndPointMask & (1 << n))
							USB_DisableEP(n);
						if (USB_EndPointMask & ((1 << 16) << n))
							USB_DisableEP(n | 0x80);
					}
					// ��û��ö�ٵ��˵��������ֻʹ�õ��Ƕ˵�0
					USB_EndPointMask = 0x00010001;
					// ������,��Ȼû��ֹͣ�Ķ˵㰡,ȫ�µ�&_^
					USB_EndPointHalt = 0x00000000;
					/*�ڸ�λʱ,����Ĭ�������߹���,���ڿ�ʼ��ʽ������ǰҪ��Ϊ������������ǵ���
					 *�����������е���,һ���������������غ�,��������������Ч��,�����������������ʵ��
					 *����������
					 */
					if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bmAttributes & USB_CONFIG_SELF_POWERED)
						USB_DeviceStatus |= USB_GETSTATUS_SELF_POWERED;
					else
						USB_DeviceStatus &= ~USB_GETSTATUS_SELF_POWERED;
				}
				// �����Թ�,��һ������������
				else
				{
					(BYTE *)pD += ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
					continue;
				}
				break;
			// �����ǽӿ�������
			case USB_INTERFACE_DESCRIPTOR_TYPE:
				alt = ((USB_INTERFACE_DESCRIPTOR *)pD)->bAlternateSetting;
				break;
			case USB_ENDPOINT_DESCRIPTOR_TYPE:
				// ���ǽ�����Ҫ�������ӽӿ��еĵ�һ��������,(������ӽӿڵĻ�,û�еĻ��պ�Ҳ��0)
				if (alt == 0)
				{
					// �õ��˶˵�
					n = ((USB_ENDPOINT_DESCRIPTOR *)pD)->bEndpointAddress & 0x8f;
					// ���˵��ת����bit�Դ洢��USB_EndPointMask��ȥ��������ʹ���˴˶˵�
					m = (n & 0x80) ? ((1 << 16) << (n & 0x0f)) : (1 << n);
					USB_EndPointMask |= m;
					// ���ö˵�
					USB_ConfigEP((USB_ENDPOINT_DESCRIPTOR *)pD);
					// ʹ�ܶ˵�
					USB_EnableEP(n);
					// ��λ�˵�,ʹ�˵㴦��ȷ���Ķ˵�
					USB_ResetEP(n);
				}
				break;
			}
			(BYTE *)pD += pD->bLength;
		}
	}
	// ������������Ϊ0,�����ǽ����е����ûָ�����ַ״̬ʱ������
	else
	{
		USB_Configuration = 0;
		/*���ﲻ����0�˵�*/
		for (n = 1; n < 16; n++)
		{
			// �����һ������ʹ���˴˶˵�,��disable it
			if (USB_EndPointMask & (1 << n)) // OUT�˵�
				USB_DisableEP(n);
			if (USB_EndPointMask & (1 << 16) << n) // IN�˵�
				USB_DisableEP(n | 0x80);
		}
		// ���ʹ�õ�ȫ���˵�,(��Ȼ0�˵㻹�ǳ����)
		USB_EndPointMask = 0x00010001;
		USB_EndPointHalt = 0x00000000;
	}
	// ���Ǹ���USB_Configuration��ֵ���鿴���ǵ���ʱ���ҵ�������Ҫ��ֵ
	if (USB_Configuration == SetupPacket.wValue.WB.L)
	{
		printf("�������óɹ�\r\n");
		return TRUE;
	}
	else
	{
		printf("��������ʧ��\r\n");
		return FALSE;
	}
}

/*����:���ڴ���˵�0����������������
 *����: USB_EVT_SETUP ö��ʱ��λ�����͵����ݰ�
 *      USB_EVT_OUT	  ��ͨ����ʱ,��λ�����͵����ݰ�
 *		USB_EVT_IN	  ��ͨ����ʱ,��λ������������
 *����ֵ:��
 */
void USB_EndPoint0(DWORD event)
{
	//	printf("����˵�0������\r\n");
	switch (event)
	{
	// ���ƴ���
	case USB_EVT_SETUP:
		//	printf("���뽨���׶�\r\n");
		// ���ȶ�ȡ��������������
		USB_SetupStage();
		// ����EP0�����ݽṹ
		EP0Data.Count = SetupPacket.wLength;
		// ʶ���Ǻ�����
		//			printf("������");
		//			printhex(SetupPacket.bRequest);
		//			printf("    ���������:");
		//			printhex(SetupPacket.bmRequestType.B);
		//			printf("\r\n");
		// bRequestType.bit5~bit6������������
		switch (SetupPacket.bmRequestType.BM.Type)
		{
		// ��׼����
		case REQUEST_STANDARD:
			switch (SetupPacket.bRequest)
			{
			// ��ȡUSB״̬
			case USB_REQUEST_GET_STATUS:
				// ���״̬ʧ����ֹͣ0x81�˵�
				if (!USB_GetStatus())
				{
					printf("��ȡ״̬ʧ��\r\n");
					goto stall_i;
				}
				break;
			case USB_REQUEST_CLEAR_FEATURE:
				// ����������ʧ����ֹͣ0x81�˵�
				if (!USB_SetClrFeature(0))
				{
					printf("�������ʧ��\r\n");
					goto stall_i;
				}
				// �ɹ�����һ����OK
				USB_StatusInStage();
				break;
			case USB_REQUEST_SET_FEATURE:
				// ���״̬ʧ����ֹͣ0x81�˵�
				if (!USB_SetClrFeature(1))
				{
					printf("��������ʧ��\r\n");
					goto stall_i;
				}
				// �ɹ�����һ����OK
				USB_StatusInStage();
				break;
			case USB_REQUEST_SET_ADDRESS:
				switch (SetupPacket.bmRequestType.BM.Recipient)
				{
				// ֻ֧�����豸���õ�ַ
				/*USBЭ��ָ�������ڸĵ�ַǰ��ɼ�Ⲣ�Դ���Ϣ��������Ӧ
				 *��USB�����õ�ַ�׶α������������յ�0�������ݰ�����ܽ���
				 *���������յ�״̬�׶ε�0���ݰ�ʱ������������ж�
				 *���Էŵ���USB_EVT_IN��*/
				case REQUEST_TO_DEVICE:
					printf("���õ�ַ\r\n");
					USB_DeviceAddress = 0x80 | SetupPacket.wValue.WB.L;
					USB_StatusInStage();
					break;
				default:
					printf("δʶ��Ľ��ն�,���ն���");
					printf("%x", SetupPacket.bmRequestType.BM.Recipient);
					printf("\r\n");
					goto stall_i;
				}
				break;
			// ��ȡ����������
			case USB_REQUEST_GET_DESCRIPTOR:
				if (!USB_GetDescriptor())
					goto stall_i;
				break;
			// ������������
			case USB_REQUEST_SET_CONFIGURATION:
				switch (SetupPacket.bmRequestType.BM.Recipient)
				{
				// ���豸
				case REQUEST_TO_DEVICE:
					if (!USB_SetConfiguration())
					{
						// ���ʧ�������������stall
						goto stall_i;
					}
					// ���������,�ո���OK
					USB_StatusInStage();
					break;
				// ��֧�ֶ�������������
				default:
					goto stall_i;
				}
				break;
			case USB_REQUEST_GET_INTERFACE:
				printf("����ӿ�\r\n");
				break;
			default:
				printf("δʶ�������,���������");
				printf("%x", SetupPacket.bRequest);
				printf("\r\n");
				break;
			}
			break;
		// ������
		case REQUEST_CLASS:
			printf("������:");
			printf("%x", SetupPacket.bmRequestType.B);
			printf("\r\n");
			switch (SetupPacket.bmRequestType.BM.Recipient)
			{
			// �Խӿ�����
			case REQUEST_TO_INTERFACE:
				//							printhex( SetupPacket.wIndex.WB.L);
				if (SetupPacket.wIndex.WB.L == 0x00)
				{
					switch (SetupPacket.bRequest)
					{
					case HID_REQUEST_GET_REPORT:
						printf("��ȡ����������\r\n");
						break;
					case HID_REQUEST_SET_REPORT:
						printf("���ñ���������\r\n");
						break;
					case HID_REQUEST_GET_IDLE:
						printf("��ȡ������\r\n");
						break;
					case HID_REQUEST_SET_IDLE:
						printf("���ÿ���\r\n");
						break;
					case HID_REQUEST_GET_PROTOCOL:
						printf("��ѯ����Э���Ƿ��ڻ\r\n");
						break;
					case HID_REQUEST_SET_PROTOCOL:
						printf("SetReprot\r\n");
						break;
					default:
						printf("��֧�ֵ�HID����\r\n");
						break;
					}
				}
				break;
			default:
				printf("����REQUEST_TO_INTERFACE\r\n");
				break;
			}
			break;
		// ��������
		case REQUEST_VENDOR:
			printf("REQUEST_VENDOR\r\n");
			break;
		// δ��������
		default:
			printf("δ��������\r\n");
		stall_i:
			USB_SetStallEP(0x80);
			break;
		}
		break;
	// ���ճɹ�
	case USB_EVT_OUT:
		printf("USB_EVT_OUT\r\n");
		if (SetupPacket.bmRequestType.BM.Dir == 0)
		{
			printf(" ������:");
			printf("%x", SetupPacket.bRequest);
			printf("\r\n");
		}
		// ��������
		else
		{
			USB_StatusOutStage();
		}
		break;
	// ���ͳɹ�
	case USB_EVT_IN:
		//	printf("USB_EVT_IN\r\n");
		if (SetupPacket.bmRequestType.BM.Dir == 1)
			USB_DataInStage();
		/*USBЭ��ָ�������ڸĵ�ַǰ��ɼ�Ⲣ�Դ���Ϣ��������Ӧ
		 *��USB�����õ�ַ�׶α������������յ�0�������ݰ�����ܽ���
		 *���������յ�״̬�׶ε�0���ݰ�ʱ������������ж�
		 *���Էŵ���USB_EVT_IN��*/
		// ԭkeil��û�м�if�ж�,����Ϊbug,�����ж��Ƿ����������õ�ַ��״̬�׶γɹ����������ж�
		else // if(SetupPacket.bRequest == USB_REQUEST_SET_ADDRESS)
		{
			if (USB_DeviceAddress & 0x80)
			{
				USB_DeviceAddress &= 0x7f;
				USB_SetAddress(USB_DeviceAddress);
			}
		}
		break;
	}
}

#endif
