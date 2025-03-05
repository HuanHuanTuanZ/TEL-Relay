#include "xuan.h"

#include "common.h"
#include "ips.h"
// #include "lcd.h"
#include "XUI.h"
#include "usart3.h"
#include "timer.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "bkg.h"
#include "mpu6050.h"

/////////////////////////////////ϵͳ����///////////////////////////////////
u8 ui_1hz = 0, ui_10hz = 0, ui_20hz = 0, ui_100hz = 0;	   // ����ˢ��Ƶ�ʱ�־
u8 tsk_1hz = 0, tsk_10hz = 0, tsk_20hz = 0, tsk_100hz = 0; // ������Ƶ�ʱ�־

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
u32 hardware_id = 0;		// Ӳ��ID
u8 ppm_connected = 0;		// PPM���ӱ�־
u8 sbus_connected = 0;		// SBUS���ӱ�־
u8 elrs_connected = 0;		// ELRS��Ƶͷ���ӱ�־
u8 elrs_back_connected = 0; // ELRS�ش����ӱ�־
u8 led_time = 10;			// ��Ļ���������
u8 led_time0 = 10;			// ��Ļ����Ϩ��ʱ��
u8 led_enable = 0;			// ��Ļ�Զ�����ʹ��
u8 admin = 0;				// ע���־
u8 adc_enable = 0;			// ADC����ʹ�ܿ���
u8 input_type = 0;			// �������ź�����
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
u8 ui_flag[4] = {0, 0, 0};		   // ����UI��־
u8 ui_current = 0;				   // ���浱ǰUI��־
u8 ui_menu_edit = 0;			   // ���ý����־
u8 ui_menu_flag = 0;			   // ���ý��浱ǰUI��־
u8 ui_select_flag = 0;			   // ���ý���UIѡ�б�־
u8 PC = WHITE8;					   // ǰ��ɫ��������ɫ
u8 BC = BLACK8;					   // ����ɫ
u8 RC = GRAY8;					   // ����Bar��״̬����ɫ
u8 SC = BLUE8;					   // ��Ŀѡ����ɫ
u8 FC = RED8;					   // ��Ŀ�۽���ɫ
u8 theme = 0;					   // ��������
u8 save_flag = 0;				   // ��������ͨ�ñ�־
u8 save_time = 0;				   // ����������ʱ
u8 bkg_type = 0;				   // UI��������
u8 lcd_dir = 0;					   // ��Ļ��ת
u8 lcd_mirror = 0;				   // ��Ļ����
u8 usart2_function = 0;			   // ����2���� �̶�MAVLINK
u16 mavlink_buad = 1152;		   // Mavlink������

//////////////////////////////////ң�ز���///////////////////////////////////
u8 out_mode = 1; // ���ģʽ
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

/*
PPM����      ->  PA0����     4Mhz����

SBUS����     ->  ����1->RX   1000000bps
SBUS���     ->  ����3->TX   1000000bps  ��CRSF����������ţ�������ͬʱ��������źţ����Բ���ͻ

MAVLINK���  ->  ����2->TX   57600bps
MAVLINK����  ->  ����2->RX   57600bps

CRSF���     ->  ����3->TX   4000000bps
CRSF����     ->  ����3->RX   4000000bps

ADC1         ->  PA1
ADC2         ->  PA4
ADC3         ->  PA5
ADC4         ->  PA6
ADC5         ->  PA7
ADC6         ->  PB0
BAT_ADC      ->  PB1

*/

int main(void)
{
	SystemInit();									// ϵͳ��ʼ��72M
	delay_init();									// ��ʱ������ʼ��
	delay_ms(100);									// �ϵ���ʱ��ȷ����Դ�ȶ�
	Gpio_Init();									// ��ʼ���������
	BEEP_Init(999, CPU_SPEED - 1);					// ��������ʼ��
	LCD_BLK_Clr();									// �򿪱���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(100000);								// ����1��ʼ��ΪSBUS
	Adc_Init();										// ADC��ʼ��
	if (read16(0) != aux_version)
		factory(); // �״����г�ʼ��


	//usart2_function = read16(30); //�̶����ڶ�����ΪMAV

	//if (usart2_function == 0) // ���ڹ���ΪMAVLINK
	//{
		usart2_init(mavlink_buad * 100); // ����2��ʼ��
		mavlink_init();					 // ��ʼ��Mavlink
		request_mavlink();				 // ����MAVLINK����
	//}
	//else if (usart2_function == 1) // ���ڹ���ΪMPU6050
	//{
	//	MPU6050_Init();
	//}

#if USB
	USB_Init();		   // ��ʼ��USB
	USB_Connect(TRUE); // ʹ��USB
#endif
	//////////////////////////////////////////////////////////////////////

	lcd_dir = read16(27);		// ��ȡ��Ļ��������
	lcd_mirror = read16(28);	// ��ȡ��Ļ��������
	LCD_Init();					// LCD��ʼ��
	LCD_Fill(0, 0, 159, 79, 0); // �����Ļ

#if DMA
	XUI_dma_init(); // ��ĻDMA��ʼ��
#endif

	bat_adc = Get_Adc(9); // ��ȡ��ص�ѹֵ
	/////////////////////////////////////////////////////////////////////////
	out_mode = 1;
	ppm_Cap_Init(0xffff, CPU_SPEED - 1); // ��1Mhz��Ƶ�ʼ���
	if (out_mode == 1)
		usart3_init(400000); // CRSF���

	TIM3_Int_Init(999, CPU_SPEED * 10 - 1); // ��ʱ��10ms�ж�
	TIM1_Int_Init(999, CPU_SPEED * 10 - 1); // ��ʱ��1ms�ж�

	TIM_Cmd(TIM3, (FunctionalState)1); // ������ʱ��
	TIM_Cmd(TIM1, (FunctionalState)1); // ������ʱ��

	beep_enable = read16(25); // ��������ʹ��״̬
	// ע���������������������ע��
	beep_queue_regedit(1400, 10);
	beep_queue_regedit(0, 4);
	beep_queue_regedit(1200, 10);
	beep_queue_regedit(0, 4);
	beep_queue_regedit(1000, 10);

	logo(); // ����LOGO
	// ����Ĭ�Ͻ���
	if (boot_menu == 0)
	{
	}
	else if (boot_menu == 1)
	{
		ui_current = 0;
		ui_flag[ui_current] = 1;
	}
	else if (boot_menu == 2)
	{
		ui_current = 1;
		ui_flag[ui_current] = 3;
	}
	else if (boot_menu == 3)
	{
		ui_current = 1;
		ui_flag[ui_current] = 0;
	}
	else if (boot_menu == 4)
	{
		ui_current = 1;
		ui_flag[ui_current] = 1;
	}
	else if (boot_menu == 5)
	{
		ui_current = 2;
		ui_flag[ui_current] = 0;
	}
	else if (boot_menu == 6)
	{
		ui_menu_edit = 1;
		ui_menu_flag = 0;
	}

	// ��ѭ��
	while (1)
	{
		background();
		ui_change();		   // �ı�UI����
		if (ui_menu_edit == 0) // ��ǰΪ����UI
		{
			switch (ui_current)
			{
			///////////////ң�ؽ���
			case 0:
				switch (ui_flag[ui_current])
				{
				case 0:
					XUI_desk1();
					break;
				case 1:
					XUI_desk2();
					break;
				case 2:
					XUI_desk3();
					break;
				case 3:
					XUI_desk4();
					break;
				case 4:
					XUI_desk5();
					break;
				case 5:
					system_status();
					break;
				}
				break;
			///////////////���ݻش�
			case 1:
				switch (ui_flag[ui_current])
				{
				case 0:
					hud();
					break;
				case 1:
					hud2();
					break;
				case 2:
					hud3();
					break;
				case 3:
					XUI_data1();
					break;
				case 4:
					XUI_data2();
					break;
				case 5:
					XUI_data3();
					break;
				case 6:
					crsf_test();
					break;
				}
				break;
			///////////////ʵ�ù���
			case 2:
				switch (ui_flag[ui_current])
				{
				case 0:
					XUI_tool1();
					break;
				case 1:
					XUI_tool2();
					break;
				case 2:
					XUI_tool3();
					break;
					//						case 3:clock();break;
					//						case 4:triangle();break;
				}
				break;
			///////////////Mavlink
			case 3:
				switch (ui_flag[ui_current])
				{
				case 0:
					mavlink_data1();
					break;
				case 1:
					mavlink_data2();
					break;
				case 2:
					mavlink_data3();
					break;
				case 3:
					mavlink_data4();
					break;
				case 4:
					mavlink_test();
					break;
				}
				break;
			}
		}
		else /// ��ǰΪ���ý���UI
		{
			statusbar();
			switch (ui_menu_flag)
			{
			case 0:
				XUI_setup_menu();
				break;
			case 1:
				model_setup();
				break;
			case 2:
				input_setup();
				break;
			case 3:
				mix_setup();
				break;
			case 4:
				output_setup();
				break;
			case 5:
				logic_setup();
				break;
			case 6:
				system_setup();
				break;
			case 7:
				adc_adjust();
				break;
			case 8:
				version_info();
				break;
			}
		}
		message_reflash(); // ˢ����ʾ��Ϣ
		XUI_reflash();	   // ˢ����Ļ
		fps_temp++;		   // ��Ļˢ�¼�����+1

#if USB
		if (USB_Configuration)
		{
			if (usb_flag == 0)
			{
				message("USB������", 20);
				out_mode_temp = out_mode;
				out_mode = 2;
			}
			usb_flag = 10;
		}
		else if (usb_flag)
			usb_flag--;
		else if (out_mode == 2)
		{
			out_mode = out_mode_temp;
			message("USB�ѶϿ�", 20);
		}
#endif
		if (tsk_100hz)
		{
			tsk_100hz = 0;
#if MAVLINK
			if (mavlink_enable && usart2_function == 0)
			{
				update();
			}
#endif
		}
		if (tsk_10hz)
		{
			tsk_10hz = 0;
#if MAVLINK
			if (mavlink_enable)
			{
				if (mavlink_connected && usart2_function == 0)
				{
					send_gps_aat();
					send_position_aat();
				}
			}
#endif
		}
		if (tsk_1hz)
		{
			tsk_1hz = 0;
#if MAVLINK
			if (mavlink_enable)
			{
				if (mavlink_connected)
					send_heartbeat_aat();
			}
#endif
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*---------------------------------UI���涨ʱ��-----------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TIM3_IRQHandler(void) // TIM3    20ms
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //??TIM3?????"??
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //??TIMx??????
		tsk_100hz = 1;
		ui_100hz = 1;
		beep_queue_operation();
		time3++;
		if (time3 % 5 == 0)
		{
			tsk_20hz = 1;
			ui_20hz = 1;
			menu_check(); // �˵��������
			if (message_time)
				message_time--;
			link_status_check();
			hud_data_prepare();

			if (usart2_function == 1)
			{
				if (gyro_connected == 1)
				{
					Ax = GetData(ACCEL_XOUT_H) * 180 / 32768;
					Ay = GetData(ACCEL_YOUT_H) * 180 / 32768;
					Az = GetData(ACCEL_ZOUT_H) * 180 / 32768;
					if (Az < 0)
					{
						if (Ax >= 0)
							Ax = 180 - Ax;
						else
							Ax = -180 - Ax;
					}
					if (Ax == 0 && Ay == 0 && Az == 0)
					{
						gyro_connected = 0;
						message("Mpu6050�ѶϿ�", 20);
						beep_negative();
					}
				}
			}
		}
		if (time3 % 10 == 0)
		{
			tsk_10hz = 1;
			ui_10hz = 1;
			time_100ms++;
		}
		if (time3 == 100)
		{
			u8 i;
			time3 = 0;
			fps = fps_temp;
			fps_temp = 0;
			second++;
			ui_1hz = 1;
			tsk_1hz = 1;
			if (led_time && led_enable)
				led_time--;
			else if (led_enable)
				LCD_BLK_Clr();

			if (tele_flag)
				tele_flag--; // �������ӱ�־
			if (mavlink_flag)
				mavlink_flag--; // �������ӱ�־
#if CRSF_DEBUG
			for (i = 0; i < 15; i++)
			{
				CRSF_rate[i] = CRSF_rate_temp[i];
				CRSF_rate_temp[i] = 0;
			}
#endif

#if MAVLINK_DEBUG
			for (i = 0; i < 30; i++)
			{
				msg_rate[i] = msg_rate_temp[i];
				msg_rate_temp[i] = 0;
			}
#endif
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*---------------------------------������ʱ��-----------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void) // 0.5MS��ѯһ��
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // 0.5�ж�
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // ����жϱ�־
		time1++;
		input();		   // ��ȡͨ������+ͨ������+��������
		mix();			   // �������
		logic_operation(); // �߼���������
		output();		   // �������
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
		if (time_use2)
			time_10ms2 += 3;
		if (time_use3)
			time_10ms3 += 3;

		if (time1 == 100)
		{
			time1 = 0;
		}
	}
}
