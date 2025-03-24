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
/////////////////////////////////系统参数///////////////////////////////////
u8 ui_1hz = 0, ui_10hz = 0, ui_20hz = 0, ui_100hz = 0;				// 界面刷新频率标志
u8 tsk = 0, tsk_1hz = 0, tsk_10hz = 0, tsk_20hz = 0, tsk_100hz = 0; // 任务处理频率标志

u8 time3 = 0;				// 界面定时器3计数器
u8 time1 = 0;				// 任务定时器1计数器
u32 second = 0;				// 开机总时间秒数
u16 battery = 5000;			// 电池电压-mV
u32 time_10ms = 0;			// 计时器1的10ms计数
u8 time_use1 = 0;			// 计时器1使用标志
u32 time_10ms2 = 0;			// 计时器2的10ms计数
u8 time_use2 = 0;			// 计时器2使用标志
u32 time_10ms3 = 0;			// 计时器3的10ms计数
u8 time_use3 = 0;			// 计时器3使用标志
int16 param[2][16];			// 设置界面通用参数缓存
u32 time_100ms = 0;			// 定时器100ms计数器
u8 ppm_connected = 0;		// PPM连接标志
u8 sbus_connected = 0;		// SBUS连接标志
u8 crsf_connected = 0;		// crsf连接标志
u8 elrs_connected = 0;		// ELRS高频头连接标志
u8 elrs_back_connected = 0; // ELRS回传连接标志
u8 led_time = 10;			// 屏幕背光计数器
u8 led_time0 = 10;			// 屏幕背光熄屏时间
u8 led_enable = 0;			// 屏幕自动背光使能
u8 adc_enable = 0;			// ADC输入使能开关
u8 input_type = 0;			// 主输入信号类型 SBUS 0/CRSF 1
u8 beep_flag = 0;			// 蜂鸣器声音持续时间计数器
u8 usb_flag = 0;			// USB插入标志
u8 out_mode_temp = 0;		// 信号输入历史记录
u8 beep_enable = 1;			// 蜂鸣器使能
int16 bat_vol = 680;		// 电池电压校准参数

///////////////////////////////界面UI参数///////////////////////////////////
u8 UP, DOWN, LEFT, RIGHT, ESC, OK; // 菜单按键按下标志
u8 ESC_release, OK_release;		   // 菜单按键释放标志
u8 fps = 0;						   // 界面刷新帧率
u8 fps_temp = 0;				   // 界面刷新帧率计数器
u8 first_splash = 0;			   // UI首次刷新标志
u8 ui_flag[3] = {0, 0, 0};		   // 桌面UI标志
u8 ui_current = 0;				   // 桌面当前UI标志
u8 ui_menu_edit = 0;			   // 设置界面标志
u8 ui_menu_flag = 0;			   // 设置界面当前UI标志
u8 ui_select_flag = 0;			   // 设置界面UI选中标志
// u8 PC = WHITE8;					   // 前景色，字体颜色
// u8 BC = BLACK8;					   // 背景色
// u8 RC = GRAY8;					   // 条形Bar，状态栏颜色
// u8 SC = BLUE8;					   // 项目选中颜色
// u8 FC = RED8;					   // 项目聚焦颜色
u8 theme = 0;			 // 界面主题
u8 save_flag = 0;		 // 参数保存通用标志
u8 save_time = 0;		 // 参数保存延时
u8 bkg_type = 0;		 // UI背景类型
u8 lcd_dir = 0;			 // 屏幕翻转
u8 lcd_mirror = 0;		 // 屏幕镜像
u16 mavlink_buad = 1152; // Mavlink波特率

//////////////////////////////////遥控参数///////////////////////////////////
u8 out_mode = 1; // 输出模式USB/CRSF
u8 ch_map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
int16 ch_min[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int16 ch_mid[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16 ch_max[16] = {-100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
u8 ch_rev[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u16 CH_input[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 通道输入值

u16 CH_out[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // 通道输出值
u8 output_map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
u8 output_max[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
u8 output_min[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

/////////////////////////////////回传参数///////////////////////////////////
u8 tele_flag = 0; // 数传连接计数器

int16 hud_roll = 0;	 // 横滚角
int16 hud_pitch = 0; // 俯仰角
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

u8 tx_signal = 0; // 发射信号
u8 rx_signal = 0; // 接收信号
u8 mavlink_flag = 0;
u8 mavlink_connected = 0;

int16 Ax, Ay, Az;
////////////////////////////////MAVLINK参数///////////////////////////////////
u8 mavlink_enable = 1; // MAVLINK输出使能

u8 ips_type = 0; // 0:0.96   1:1.8

int main(void)
{
	SystemInit();									// 系统初始化72M
	delay_init();									// 延时函数初始化
	delay_ms(100);									// 上电延时，确保电源稳定
	Gpio_Init();									// 初始化输出引脚
	BEEP_Init(999, CPU_SPEED - 1);					// 蜂鸣器初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	Adc_Init();										// ADC初始化

	if (input_type)
	{
		uart_init(420000); // 串口1初始化为CRSF
	}
	else
	{
		uart_init(100000); // 串口1初始化为SBUS
	}

	usart2_init(mavlink_buad * 100); // 串口2初始化
	usart3_init(400000);			 // CRSF输出
	mavlink_init();

#if USB
	USB_Init();		   // 初始化USB
	USB_Connect(TRUE); // 使能USB
#endif

	LCD_Init();								 // LCD初始化
	LCD_Fill(0, 0, LCD_W, LCD_H, LIGHTBLUE); // 全屏非缓存覆写
	LCD_BLK_Set();							 // 打开背光
	ppm_Cap_Init(0xffff, CPU_SPEED - 1);	 // 以1Mhz的频率计数

	TIM3_Int_Init(1999, CPU_SPEED * 10 - 1); // 定时器3 50HZ 20ms中断
	TIM1_Int_Init(999, CPU_SPEED * 10 - 1);	 // 定时器1 100HZ 10ms中断

	TIM_Cmd(TIM3, (FunctionalState)1); // 打开任务定时器
	TIM_Cmd(TIM1, (FunctionalState)1); // 打开任务定时器

	// LCD_ShowPicture(0, 0, 160, 80, gImage_1);
	// beep_enable = read16(25); // 检测蜂鸣器使能状态
	// 注册蜂鸣器开机声音，反向注册
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
		// message_reflash(); // 刷新提示信息
		// LCD_Reflash();
	}
}

/*---------------------------------UI定时器-----------------------------------------------------------------------*/
void TIM3_IRQHandler(void) // TIM3		20ms		50HZ
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) // 中断
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志
		tsk++;
		menu_check();	   // 按钮检测
		if (tsk % 10 == 0) // 5HZ 200ms 任务计数
		{

			if (message_time)
				message_time--;
		}
		if (tsk == 50) // 1HZ 1S 任务计数
		{
			u8 i;
			tsk = 0;
			time3++;
			if (USB_Configuration)
			{
				if (usb_flag == 0)
				{
					message("USB已连接", 10);
					beep_positive();
					out_mode = 2;
				}
				usb_flag = 10;
			}
			else if (usb_flag)
				usb_flag--;
			else if (out_mode == 2)
			{
				message("USB已断开", 10);
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
/*---------------------------------主任务定时器-----------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void) // TIM1		10ms		100HZ
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // 1MS中断
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除中断标志
		time1++;
		input(); // 获取通道输入+通道运算+按键输入
		// output(); // 输出运算
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
		if (time_use1) // 计时器计数
			time_10ms += 3;

		if (time1 == 100)
		{
			time1 = 0;
		}
	}
}
