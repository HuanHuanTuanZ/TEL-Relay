#include "Ctrl.h"
#include "delay.h"
#include "common.h"
#include "timer.h"
#include "lcd.h"
#include "lcd_init.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "usart3.h"
#include "pic.h"
/////////////////////////////////ϵͳ����///////////////////////////////////
u8 ui_1hz = 0, ui_10hz = 0, ui_20hz = 0, ui_100hz = 0;				// ����ˢ��Ƶ�ʱ�־
u8 tsk = 0, tsk_1hz = 0, tsk_10hz = 0, tsk_20hz = 0, tsk_100hz = 0; // ������Ƶ�ʱ�־

u8 time3 = 0;				// ���涨ʱ��3������
u8 time1 = 0;				// ����ʱ��1������
u32 second = 0;				// ������ʱ������
u16 battery = 5000;			// ��ص�ѹ-mV
u32 time_10ms = 0;			// ��ʱ��1��10ms����
u8 time_use1 = 0;			// ��ʱ��1ʹ�ñ�־
u32 time_10ms2 = 0;			// ��ʱ��2��10ms����
u8 time_use2 = 0;			// ��ʱ��2ʹ�ñ�־
u32 time_10ms3 = 0;			// ��ʱ��3��10ms����
u8 time_use3 = 0;			// ��ʱ��3ʹ�ñ�־
int16 param[2][16];			// ���ý���ͨ�ò�������
u32 time_100ms = 0;			// ��ʱ��100ms������
u8 ppm_connected = 0;		// PPM���ӱ�־
u8 sbus_connected = 0;		// SBUS���ӱ�־
u8 crsf_connected = 0;		// crsf���ӱ�־
u8 elrs_connected = 0;		// ELRS��Ƶͷ���ӱ�־
u8 elrs_back_connected = 0; // ELRS�ش����ӱ�־
u8 led_time = 10;			// ��Ļ���������
u8 led_time0 = 10;			// ��Ļ����Ϩ��ʱ��
u8 led_enable = 0;			// ��Ļ�Զ�����ʹ��
u8 adc_enable = 0;			// ADC����ʹ�ܿ���
u8 input_type = 0;			// �������ź����� SBUS 0/CRSF 1
u8 beep_flag = 0;			// ��������������ʱ�������
u8 usb_flag = 0;			// USB�����־
u8 out_mode_temp = 0;		// �ź�������ʷ��¼
u8 beep_enable = 1;			// ������ʹ��
int16 bat_vol = 680;		// ��ص�ѹУ׼����

///////////////////////////////����UI����///////////////////////////////////
u8 UP, DOWN, LEFT, RIGHT, ESC, OK; // �˵��������±�־
u8 ESC_release, OK_release;		   // �˵������ͷű�־
u8 fps = 0;						   // ����ˢ��֡��
u8 fps_temp = 0;				   // ����ˢ��֡�ʼ�����
u8 first_splash = 0;			   // UI�״�ˢ�±�־
u8 ui_flag[3] = {0, 0, 0};		   // ����UI��־
u8 ui_current = 0;				   // ���浱ǰUI��־
u8 ui_menu_edit = 0;			   // ���ý����־
u8 ui_menu_flag = 0;			   // ���ý��浱ǰUI��־
u8 ui_select_flag = 0;			   // ���ý���UIѡ�б�־
// u8 PC = WHITE8;					   // ǰ��ɫ��������ɫ
// u8 BC = BLACK8;					   // ����ɫ
// u8 RC = GRAY8;					   // ����Bar��״̬����ɫ
// u8 SC = BLUE8;					   // ��Ŀѡ����ɫ
// u8 FC = RED8;					   // ��Ŀ�۽���ɫ
u8 theme = 0;			 // ��������
u8 save_flag = 0;		 // ��������ͨ�ñ�־
u8 save_time = 0;		 // ����������ʱ
u8 bkg_type = 0;		 // UI��������
u8 lcd_dir = 0;			 // ��Ļ��ת
u8 lcd_mirror = 0;		 // ��Ļ����
u16 mavlink_buad = 1152; // Mavlink������

//////////////////////////////////ң�ز���///////////////////////////////////
u8 out_mode = 1; // ���ģʽUSB/CRSF
u8 ch_map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
int16 ch_min[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int16 ch_mid[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16 ch_max[16] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
u8 ch_rev[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u16 CH_input[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // ͨ������ֵ

u16 CH_out[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // ͨ�����ֵ
u8 output_map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
u8 output_max[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
u8 output_min[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

/////////////////////////////////�ش�����///////////////////////////////////
u8 tele_flag = 0; // �������Ӽ�����

int16 hud_roll = 0;	 // �����
int16 hud_pitch = 0; // ������
int16 hud_yaw = 0;
int16 hud_alt = 0;
int16 hud_groundspeed = 0;
int16 hud_airspeed = 0;
u8 hud_gps = 0;
int16 hud_bat = 0;
int16 hud_cur = 0;
u8 hud_signal = 0;
int hud_lantitude;
int hud_longitude;

u8 hud_data_type = 0;
u8 gyro_connected = 0;

u8 tx_signal = 0; // �����ź�
u8 rx_signal = 0; // �����ź�
u8 mavlink_flag = 0;
u8 mavlink_connected = 0;

int16 Ax, Ay, Az;
////////////////////////////////MAVLINK����///////////////////////////////////
u8 mavlink_enable = 1; // MAVLINK���ʹ��

u8 ips_type = 0; // 0:0.96   1:1.8

int main(void)
{
	SystemInit();									// ϵͳ��ʼ��72M
	delay_init();									// ��ʱ������ʼ��
	delay_ms(100);									// �ϵ���ʱ��ȷ����Դ�ȶ�
	Gpio_Init();									// ��ʼ���������
	BEEP_Init(999, CPU_SPEED - 1);					// ��������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	Adc_Init();										// ADC��ʼ��

	if (input_type)
	{
		uart_init(420000); // ����1��ʼ��ΪCRSF
	}
	else
	{
		uart_init(100000); // ����1��ʼ��ΪSBUS
	}

	usart2_init(mavlink_buad * 100); // ����2��ʼ��
	usart3_init(400000);			 // CRSF���
	mavlink_init();

#if USB
	USB_Init();		   // ��ʼ��USB
	USB_Connect(TRUE); // ʹ��USB
#endif

	LCD_Init();								 // LCD��ʼ��
	LCD_Fill(0, 0, LCD_W, LCD_H, LIGHTBLUE); // ȫ���ǻ��渲д
	LCD_BLK_Set();							 // �򿪱���
	ppm_Cap_Init(0xffff, CPU_SPEED - 1);	 // ��1Mhz��Ƶ�ʼ���

	TIM3_Int_Init(1999, CPU_SPEED * 10 - 1); // ��ʱ��3 50HZ 20ms�ж�
	TIM1_Int_Init(999, CPU_SPEED * 10 - 1);	 // ��ʱ��1 100HZ 10ms�ж�

	TIM_Cmd(TIM3, (FunctionalState)1); // ������ʱ��
	TIM_Cmd(TIM1, (FunctionalState)1); // ������ʱ��

	// LCD_ShowPicture(0, 0, 160, 80, gImage_1);
	// beep_enable = read16(25); // ��������ʹ��״̬
	// ע���������������������ע��
	beep_queue_regedit(1400, 5);
	beep_queue_regedit(0, 2);
	beep_queue_regedit(1200, 5);
	beep_queue_regedit(0, 2);
	beep_queue_regedit(1000, 5);
	while (1)
	{
		// LCD_Clear(WHITE);
		// LCD_ShowIntNum(10, 26, time3, 4, BLACK, YELLOW, 16);
		if (ui_menu_edit == 0)
		{
			switch (ui_current)
			{
			case 0:
				switch (ui_flag[ui_current])
				{
					break;
				}
				break;
			case 1:
				switch (ui_flag[ui_current])
				{
					break;
				}
				break;
			case 2:
				switch (ui_flag[ui_current])
				{
					break;
				}
				break;
			}
		}
		else
		{
		}
		// message_reflash(); // ˢ����ʾ��Ϣ
		// LCD_Reflash();
	}
}

/*---------------------------------UI��ʱ��-----------------------------------------------------------------------*/
void TIM3_IRQHandler(void) // TIM3		20ms		50HZ
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) // �ж�
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // ����жϱ�־
		tsk++;
		menu_check();	   // ��ť���
		if (tsk % 10 == 0) // 5HZ 200ms �������
		{

			if (message_time)
				message_time--;
		}
		if (tsk == 50) // 1HZ 1S �������
		{
			u8 i;
			tsk = 0;
			time3++;
			if (USB_Configuration)
			{
				if (usb_flag == 0)
				{
					message("USB������", 10);
					beep_positive();
					out_mode = 2;
				}
				usb_flag = 10;
			}
			else if (usb_flag)
				usb_flag--;
			else if (out_mode == 2)
			{
				message("USB�ѶϿ�", 10);
				beep_negative();
				out_mode = 1;
			}
			for (i = 0; i < 15; i++)
			{
				CRSF_rate[i] = CRSF_rate_temp[i];
				CRSF_rate_temp[i] = 0;
			}
			link_status_check();
		}

		beep_queue_operation();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*---------------------------------������ʱ��-----------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void) // TIM1		10ms		100HZ
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // 1MS�ж�
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // ����жϱ�־
		time1++;
		input(); // ��ȡͨ������+ͨ������+��������
		// output(); // �������
		switch (out_mode)
		{
		case 1:
			CRSF_out();
			break;
		case 2:
			USB_out();
			break;
		default:
			break;
		}
		if (time_use1) // ��ʱ������
			time_10ms += 3;

		if (time1 == 100)
		{
			time1 = 0;
		}
	}
}
