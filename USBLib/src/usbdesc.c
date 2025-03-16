#include "Ctrl.h"
#if USB

#include "stm32f10x.h"
#include "usbreg.h"
#include "usbuser.h"
#include "usbcore.h"
#include "usb_hw.h"
#include "usbcfg.h"
#include "hid.h"

#include "usbdesc.h"

/* USB Standard Device Descriptor */
const BYTE USB_DeviceDescriptor[] = {
  USB_DEVICE_DESC_SIZE,              /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(0x0110), /* 1.10 */          /* bcdUSB */
  0x00,                              /* bDeviceClass */
  0x00,                              /* bDeviceSubClass */
  0x00,                              /* bDeviceProtocol */
  USB_MAX_PACKET0,                   /* bMaxPacketSize0 */
  WBVAL(0x055),                     /* idVendor */
  WBVAL(0x0219),                     /* idProduct */
  WBVAL(0x0120), /* 1.20 */          /* bcdDevice */
  0x04,                              /* iManufacturer */
  0x20,                              /* iProduct */
  0x4A,                              /* iSerialNumber */
  0x01                               /* bNumConfigurations */
};

#define HID_INPUT_REPORT_BYTES       1              /* size of report in Bytes */
#define HID_OUTPUT_REPORT_BYTES      1              /* size of report in Bytes */
#define HID_FEATURE_REPORT_BYTES     1              /* size of report in Bytes */

const u8 HID_ReportDescriptor[] = {
	0x05, 0x01,                    // 	USAGE_PAGE (Generic Desktop)
	//    0x09, 0x04,              // 	USAGE (Joystick)
  0x09, 0x05,                    // 	����Ϊ��Ϸ�ֱ�
  0xa1, 0x01,                    // 	COLLECTION (Application)
  0xa1, 0x02,                    // 	COLLECTION (Logical)
 
	//ʹ��X���Y��
	0x09, 0x30,                    //   ʹ��X��
	0x09, 0x31,                    //   ʹ��Y��
	0x15, 0x00,                    //   �߼���Сֵ(0)
	0x26, 0xff, 0x00,              //   �߼����ֵ (255)
	0x35, 0x00,                    //   ������Сֵ (0)
	0x46, 0xff, 0x00,              //   �������ֵ (255)
	0x75, 0x08,                    //   ����λ�� (8)
	0x95, 0x02,                    //   �����ֽ��� (2)
	0x81, 0x02,                    //   ������� (Data,Var,Abs)28
	
	//ʹ��Z��
	0x09, 0x32,                    //   ʹ��Z�� (Z)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
	0x46, 0xff, 0x7f,              //   PHYSICAL_MAXIMUM (32767)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)18
	
	//ʹ��X��ת
	0x09, 0x33,                    //   ʹ��X��ת
	0x15, 0x00,                    //   �߼���Сֵ (0)
	0x26, 0xff, 0x00,              //   �߼����ֵ (255)
	0x35, 0x00,                    //   ������Сֵ (0)
	0x46, 0xff, 0x7f,              //   �������ֵ (32767)
	0x75, 0x08,                    //   ����λ�� (8)
	0x95, 0x01,                    //   �����ֽ��� (1)
	0x81, 0x02,                    //   ������� (Data,Var,Abs)46   18

	//ʹ�û���
	0x09, 0x36,                    //   ʹ�û��� (Slider)
	0x15, 0x00,                    //   �߼���Сֵ (0)
	0x26, 0xff, 0x00,              //   �߼����ֵ (255)
	0x35, 0x00,                    //   ������Сֵ (0)
	0x46, 0xff, 0x7f,              //   �������ֵ (32767)
	0x75, 0x08,                    //   ����λ�� (8)
	0x95, 0x01,                    //   �����ֽ��� (1)
	0x81, 0x02,                    //   ������� (Data,Var,Abs)18

	//ʹ��Y��ת
	0x09, 0x34,                    //   ʹ��Y��ת (Y)
	0x15, 0x00,                    //   �߼���Сֵ (0)
	0x26, 0xff, 0x00,              //   �߼����ֵ (255)
	0x35, 0x00,                    //   ������Сֵ (0)
	0x46, 0xff, 0x7f,              //   �������ֵ (32767)
	0x75, 0x08,                    //   ����λ�� (8)
	0x95, 0x01,                    //   �����ֽ��� (1)
	0x81, 0x02,                    //   ������� (Data,Var,Abs)18

	//ʹ��Z��ת
	0x09, 0x35,                    	//   ʹ��Z��ת (z)
	0x15, 0x00,                    	//   �߼���Сֵ (0)
	0x26, 0xff, 0x00,              	//   �߼����ֵ (255)
	0x35, 0x00,                    	//   ������Сֵ (0)
	0x46, 0xff, 0x7f,              	//   �������ֵ (32767)
	0x75, 0x08,                    	//   ����λ�� (8)
	0x95, 0x01,                    	//   �����ֽ��� (1)
	0x81, 0x02,                    	//   ������� (Data,Var,Abs)46   18

	//ʹ�û���
  0x09, 0x36,                    	//   ʹ�û��� (Slider)
	0x15, 0x00,                    	//   �߼���Сֵ (0)
	0x26, 0xff, 0x00,              	//   �߼����ֵ (255)
	0x35, 0x00,                    	//   ������Сֵ (0)
	0x46, 0xff, 0x7f,              	//   �������ֵ (32767)
	0x75, 0x08,                    	//   ����λ�� (8)
	0x95, 0x01,                    	//   �����ֽ��� (1)
	0x81, 0x02,                    	//   ������� (Data,Var,Abs)18

	//ʹ���Ӿ�ͷ��
	0x09, 0x39,                    	//   USAGE (Hat switch)
	0x15, 0x00,                    	//   LOGICAL_MINIMUM (0)
	0x25, 0x07,                    	//   LOGICAL_MAXIMUM (3)
	0x35, 0x00,                    	//   PHYSICAL_MINIMUM (0)
	0x46, 0x3b, 0x01,              	//   PHYSICAL_MAXIMUM (270)
	0x65, 0x14,                    	//   UNIT (Eng Rot:Angular Pos)
	0x75, 0x08,                    	//   REPORT_SIZE (8)
	0x95, 0x01,                    	//   REPORT_COUNT (1)
	0x81, 0x02,                    	//   INPUT (Data,Var,Abs)    19
		
	//ʹ�ð���64��
	0x05, 0x09,                    	//   	USAGE_PAGE (Button)
	0x19, 0x01,                    	//   	USAGE_MINIMUM (Button 1)
	0x29, 0x20,                    	// 		USAGE_MAXIMUM (Button 64)
	0x15, 0x00,                    	//    LOGICAL_MINIMUM (0)
	0x25, 0x01,                    	//    LOGICAL_MAXIMUM (1)
	0x95, 0x20, 										// 		REPORT_COUNT (64)
	0x75, 0x01, 										// 		REPORT_SIZE (1)
	0x81, 0x02, 										// 		INPUT (Data,Var,Abs)
	
	//������
	0xc0,                          	//    END_COLLECTION
	0xc0                           	// 		END_COLLECTION
};


const u8 HID_ReportDescriptor_222222[] = {
	/* USER CODE BEGIN 0 */
	0x05,0x01,          /*Usage Page(Generic Desktop)*/ 
	0x09,0x04,          /*Usage(joystick)*/
	0xA1,0x01,          /*Collection(Application)*/
	//24
	0x05, 0x01,         /*USAGE_PAGE (Generic Desktop)*/
	0x09, 0x01,         /*USAGE (Pointer)*/
	0xa1, 0x00,         /*COLLECTION (Physical)*/
	0x09, 0x30,         /*USAGE (Rx)*/
	0x09, 0x31,         /*USAGE (Ry)*/
	0x09, 0x32,         /*USAGE (Rz)*/
	0x15, 0x00,         /*LOGICAL_MINIMUM (0)*/
	0x26, 0x00,0x08,   /*LOGICAL_MAXIMUM (2048)*/
	0x75, 0x10,         /*REPORT_SIZE (16)*/
	0x95, 0x03,         /*REPORT_COUNT (3)*/
	0x81, 0x02,         /*INPUT (Data,Var,Abs)*/
	0xc0,               /*End Collection*/
	//24
	0x05,0x01,          /*USAGE_PAGE (Generic Desktop)*/
	0x09,0x01,          /*USAGE (Pointer)*/
	0xa1,0x00,          /*COLLECTION (Physical)*/
	0x09,0x33,          /*Usage(X axis)*/
	0x09,0x34,          /*Usage(Y axis)*/ 
	0x09,0x35,          /*Usage(Z axis)*/ 
	0x15,0x00,          /*Logical Minimum(0)*/
	0x26,0x00,0x08,     /*Logical Maximum(2048)*/
	0x95,0x03,          /*REPORT_COUNT(3)*/
	0x75,0x10,          /*REPORT_SIZE (16)*/
	0x81,0x02,          /*INPUT (Data,Var,Abs)*/
	0xc0,               /*End Collection*/
	//20
	0x05,0x01,          /*USAGE_PAGE (Generic Desktop*/
	0x09,0x36,			/*USAGE (Pointer)*/
	0xA1,0x00,			/*COLLECTION (Physical)*/
	0x09,0x36,          /*USAGE (Slider)*/
	0x15,0x00,			/*LOGICAL_MINIMUM (0)*/
	0x26,0x00,0x08,	    /*LOGICAL_MAXIMUM (2048)*/
	0x75,0x10,          /*REPORT_SIZE (16)*/
	0x95,0x01,          /*REPORT_COUNT (3)*/
	0x81,0x02,          /*INPUT (Data,Var,Abs)*/
	0xc0,               /*End Collection*/
	//20
	0x05,0x01,          /*USAGE_PAGE (Generic Desktop*/
	0x09,0x36,			/*USAGE (Pointer)*/
	0xA1,0x00,			/*COLLECTION (Physical)*/
	0x09,0x36,          /*USAGE (Slider)*/
	0x15,0x00,			/*LOGICAL_MINIMUM (0)*/
	0x26,0x00,0x08,	    /*LOGICAL_MAXIMUM (2048)*/
	0x75,0x10,          /*REPORT_SIZE (16)*/
	0x95,0x01,          /*REPORT_COUNT (3)*/
	0x81,0x02,          /*INPUT (Data,Var,Abs)*/
	0xc0,               /*End Collection*/
		
//	0x09, 0x01,                    //   USAGE (Vendor Usage 1) 每个功能的一个卷标志
//	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)    表示每个传输数据限定�??0
//	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)    表示每个传输数据的最大�?�限定为255
//	0x95, 0x40,                    //   REPORT_COUNT (64) 每次接收的数据长度，这里�??64�??
//	0x75, 0x08,                    //   REPORT_SIZE (8)        传输字段的宽度为8bit，表示每个传输的数据范围�??0~ffff ffff
//	0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) 表示USB设备要接收PC的数据的功能

	/* USER CODE END 0 */
	0xC0    /*     END_COLLECTION	             */
};


const u16 HID_ReportDescSize = sizeof(HID_ReportDescriptor);



/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
/* USB???????(?????????????)(Configuration, Interface, Endpoint, Class, Vendor */
const u8 USB_ConfigDescriptor[] = {
/* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,       /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
  WBVAL
	(                             /* wTotalLength */
    USB_CONFIGUARTION_DESC_SIZE +
    USB_INTERFACE_DESC_SIZE     +
    HID_DESC_SIZE               +
    USB_ENDPOINT_DESC_SIZE     
  ),
  0x01,                              /* bNumInterfaces */
  0x01,                              /* bConfigurationValue: 0x01 is used to select this configuration */
  0x00,                              /* iConfiguration: no string to describe this configuration */
  USB_CONFIG_BUS_POWERED /*|*/       /* bmAttributes */
/*USB_CONFIG_REMOTE_WAKEUP*/,
  USB_CONFIG_POWER_MA(100),          /* bMaxPower, device power consumption is 100 mA */

/* Interface 0, Alternate Setting 0, HID Class */
  USB_INTERFACE_DESC_SIZE,           /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
  0x00,                              /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x01,                              /* bNumEndpoints */
  USB_DEVICE_CLASS_HUMAN_INTERFACE,  /* bInterfaceClass */
  0,                 		     /* bInterfaceSubClass */
  0,              		     /* bInterfaceProtocol */
  0x04,                              /* iInterface */
/* HID Class Descriptor */
/* HID_DESC_OFFSET = 0x0012 */
  HID_DESC_SIZE,                     /* bLength */
  HID_HID_DESCRIPTOR_TYPE,           /* bDescriptorType */
  WBVAL(0x0110), /* 1.10 */          /* bcdHID */
  0x21,                              /* bCountryCode */
  0x01,                              /* bNumDescriptors */
  HID_REPORT_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(HID_REPORT_DESC_SIZE),       /* wDescriptorLength */
/* Endpoint, HID Interrupt In */

  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_IN(1),                /* bEndpointAddress */
  USB_ENDPOINT_TYPE_INTERRUPT,       /* bmAttributes */
  WBVAL(0x0010),                     /* wMaxPacketSize */
	0x00,
  0x20,          /* 32ms */          /* bInterval */
	/*terimate*/
};


/* USB String Descriptor (optional) */
const u8 USB_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
  0x04,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(0x0409), /* US English */    /* wLANGID */
/* Index 0x01: Manufacturer */
  (13*2 + 2),                        /* bLength (13 Char + Type + lenght) */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'X',0,
  'R',0,
  'C',0,
  ' ',0,
  'J',0,
  'O',0,
  'Y',0,
  'S',0,
  'T',0,
  'I',0,
  'C',0,
  'K',0,
  ' ',0,
/* Index 0x02: Product */
  (16*2 + 2),                        /* bLength (16 Char + Type + lenght) */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'X',0,
  'R',0,
  'C',0,
  ' ',0,
  'J',0,
  'O',0,
  'Y',0,
  'S',0,
  'T',0,
  'I',0,
  'C',0,
  'K',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
/* Index 0x03: Serial Number */
  (12*2 + 2),                        /* bLength (12 Char + Type + lenght) */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  '0',0,                             /* allowed characters are       */
  '0',0,                             /*   0x0030 - 0x0039 ('0'..'9') */
  '0',0,                             /*   0x0041 - 0x0046 ('A'..'F') */
  '1',0,                             /*   length >= 26               */
  'A',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
/* Index 0x04: Interface 0, Alternate Setting 0 */
  ( 3*2 + 2),                        /* bLength (3 Char + Type + lenght) */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'H',0,
  'I',0,
  'D',0,
};

#endif


