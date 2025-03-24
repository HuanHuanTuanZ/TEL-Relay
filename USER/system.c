#include "common.h"
#include "stmflash.h"
#include "usart.h"
#include "lcd.h"
#include "string.h"

void save16(u16 addr, u16 v)
{
	u16 tmp[1];
	tmp[0] = v;
	STMFLASH_Write(FLASH_SAVE_ADDR + addr * 2, tmp, 1);
}

void save32(u16 addr, u32 v)
{
	u16 tmp[2];
	tmp[0] = v >> 16;
	tmp[1] = v;
	STMFLASH_Write(FLASH_SAVE_ADDR + addr * 2, tmp, 2);
}

u16 read16(u16 addr)
{
	return Read_StmflashData(FLASH_SAVE_ADDR + addr * 2);
}

u32 read32(u16 addr)
{
	u16 v1 = Read_StmflashData(FLASH_SAVE_ADDR + addr * 2);
	u16 v2 = Read_StmflashData(FLASH_SAVE_ADDR + (addr + 1) * 2);
	return v1 << 16 | v2;
}

#define model_param_max 172
// void model_reset()
//{
//	u8 i;
//	u16 temp[model_param_max];
//	u32 addr = FLASH_SAVE_ADDR + model_current * model_size * 2 + 200;
//	save16(model_addr + model_size - 1, 1);
//	for (i = 0; i < 16; i++) // 通道输入参数   64   0-63
//	{
//		temp[i] = i;
//		temp[i + 16] = 0;
//		temp[i + 32] = 0;
//		temp[i + 48] = 0;
//	}
//	for (i = 0; i < 4; i++) // 混控参数   16   64-79
//	{
//		temp[i + 64] = 0;
//		temp[i + 68] = 0;
//		temp[i + 72] = 0;
//		temp[i + 76] = 0;
//	}
//	for (i = 0; i < 16; i++) // 通道输出参数  64   80-143
//	{
//		temp[i + 80] = i;
//		temp[i + 96] = 100;
//		temp[i + 112] = 0;
//		temp[i + 128] = 100;
//	}
//	for (i = 0; i < 4; i++) // 逻辑开关参数  20   144-163
//	{
//		temp[i + 144] = 0;
//		temp[i + 148] = 0;
//		temp[i + 152] = 0;
//		temp[i + 156] = 0;
//		temp[i + 160] = 0;
//	}
//	STMFLASH_Write(addr, temp, model_param_max);
//	sprintf(model_name, "Model%02d", model_current + 1);
//	model_name_save(model_addr + 164); // 164-171
// }

// void model_read() // 通道数据读取
//{
//	u8 i;
//	if (read16(model_addr + model_size - 1) != 1)
//		model_reset();
//	for (i = 0; i < 16; i++)
//	{
//		ch_map[i] = read16(model_addr + i);
//		ch_min[i] = read16(model_addr + 16 + i);
//		ch_mid[i] = read16(model_addr + 32 + i);
//		ch_max[i] = read16(model_addr + 48 + i);
//	}
//	for (i = 0; i < 4; i++)
//	{
//		mixer_type[i] = read16(model_addr + 64 + i);
//		mixer_in1[i] = read16(model_addr + 68 + i);
//		mixer_in2[i] = read16(model_addr + 72 + i);
//		mixer_rate[i] = read16(model_addr + 76 + i);
//	}
//	for (i = 0; i < 16; i++)
//	{
//		output_map[i] = read16(model_addr + 80 + i);
//		output_min[i] = read16(model_addr + 96 + i);
//		ch_rev[i] = read16(model_addr + 112 + i);
//		output_max[i] = read16(model_addr + 128 + i);
//	}
//	for (i = 0; i < 4; i++)
//	{
//		logic_condition[i] = read16(model_addr + 144 + i);
//		logic_a[i] = read16(model_addr + 148 + i);
//		logic_x_value[i] = read16(model_addr + 152 + i);
//		logic_last_time[i] = read16(model_addr + 156 + i);
//		logic_mode[i] = read16(model_addr + 160 + i);
//	}
//	model_name_read(model_addr + 164); // 164-171
// }

#define system_param_max 35
void factory() // 初始化
{
	u8 i;
	u16 temp[system_param_max];
	temp[0] = aux_version; // 软件版本
	temp[1] = 5;		   // 界面主题
	temp[2] = 0;		   // 模型选择
	temp[3] = 0;		   // 远程遥控
	temp[4] = 0;		   // 默认界面
	temp[5] = 0;		   // 自动背光
	temp[6] = 60;		   // 背光时间
	temp[7] = read16(7);   /////////////////////7
	temp[8] = 0;		   // ADC输入
	temp[9] = 9;

	for (i = 0; i < 4; i++)
	{
		temp[10 + i] = 0;
		temp[14 + i] = 2047;
		temp[18 + i] = 4095;
	}
	temp[22] = 2;	// 输入信号类型
	temp[23] = 1;	// MAV输出
	temp[24] = 499; // 电压校准
	temp[25] = 1;	// 蜂鸣器
	temp[26] = 0;	// UI背景类型
	temp[27] = 0;	// 屏幕翻转
	temp[28] = 0;	// 屏幕镜像
	temp[29] = 0;	// HUD类型
	temp[30] = 0;	// 串口2功能
	temp[31] = 0;	// HUD TYPE
	temp[32] = 0;
	temp[33] = 0;
	temp[34] = 1152; // MAVLINK波特率
	STMFLASH_Write(FLASH_SAVE_ADDR, temp, system_param_max);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*---------------------------------读取设置---------------------------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void system_read()
//{
//	u8 i;
//	hardware_id = *(vu32 *)(0x1FFFF7E8) + *(vu32 *)(0x1FFFF7EC) + *(vu32 *)(0x1FFFF7F0);
//	if (read16(0) != aux_version)
//		factory(); // 首次运行初始化
//	theme = read16(1);
//	model_current = read16(2);
//	remote_ctrl = read16(3);
//	boot_menu = read16(4);
//	led_enable = read16(5);
//	led_time0 = read16(6);
//	admin = read16(7);
//	adc_enable = read16(8);
//	elrs_tlm = read16(9);
//
//	for (i = 0; i < 4; i++) // 10-21
//	{
//		adc_min[i] = read16(10 + i);
//		adc_mid[i] = read16(14 + i);
//		adc_max[i] = read16(18 + i);
//	}
//	input_type = read16(22);
//	mavlink_enable = read16(23);
//	bat_vol = read16(24);
//	beep_enable = read16(25);
//	bkg_type = read16(26);
//	lcd_dir = read16(27);
//	lcd_mirror = read16(28);
//	hud_data_type = read16(29);
//	usart2_function = read16(30);
//	hud_type = read16(31);
//	hud_type2 = read16(32);
//	hud_type3 = read16(33);
//	mavlink_buad = read16(34);
//
//	theme_change(theme);
//	model_read();
// }

// void model_name_save(int16 addr)
//{
//	u16 temp[8];
//	u8 i;
//	for (i = 0; i < 8; i++)
//		temp[i] = model_name[2 * i] << 8 | model_name[2 * i + 1];
//	STMFLASH_Write(FLASH_SAVE_ADDR + addr * 2, temp, 8);
// }
//
// void model_name_read(int16 addr)
//{
//	u16 temp[8];
//	u8 i;
//	STMFLASH_Read(FLASH_SAVE_ADDR + addr * 2, temp, 8);
//	for (i = 0; i < 8; i++)
//	{
//		model_name[2 * i] = temp[i] >> 8;
//		model_name[2 * i + 1] = temp[i];
//	}
// }

u8 message_time = 0;
char message_str[32];

void message(char *p, u8 time)
{
	message_time = time;
	strcpy(message_str, p);
}

void message_reflash()
{
	if (message_time)
	{
		LCD_ShowFillRoundRect(80, 40, 56, 8, 4, YELLOW);
		LCD_ShowRoundRect(80, 40, 56, 8, 4, BLACK);
		LCD_ShowStr(80 - strlen(message_str) * 3, 34, (u8 *)message_str, BLACK, YELLOW, 12, 1);
	}
}

void link_status_check()
{
	if (sbus_status)
	{
		sbus_status--;
		if (sbus_connected == 0)
		{
			sbus_connected = 1;
			message("SBUS已连接", 10);
			beep_positive();
		}
	}
	else
	{
		if (sbus_connected)
		{
			sbus_connected = 0;
			message("SBUS已断开", 10);
			beep_negative();
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	if (ppm_status)
	{
		ppm_status--;
		if (ppm_connected == 0)
		{
			ppm_connected = 1;
			message("PPM已连接", 10);
			beep_positive();
		}
	}
	else
	{
		if (ppm_connected)
		{
			ppm_connected = 0;
			message("PPM已断开", 10);
			beep_negative();
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	if (CRSF_rate[14])
	{
		if (elrs_connected == 0)
		{
			crsf_param_buf = elrs_tlm - 1;
			crsf_param_flag = 2;
			message("ELRS-TX已连接", 10);
			beep_positive();
		}
		elrs_connected = 2;
	}
	else
	{
		if (elrs_connected)
		{
			elrs_connected--;
			if (elrs_connected == 0)
			{
				message("ELRS-TX已断开", 10);
				beep_negative();
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////////////////
	if (tele_flag)
	{
		if (elrs_back_connected == 0)
		{
			elrs_back_connected = 1;
			first_splash = 1;
			message("ELRS回传已连接", 10);
			beep_positive();
		}
	}
	else
	{
		if (elrs_back_connected)
		{
			elrs_back_connected = 0;
			message("ELRS回传已断开", 10);
			beep_negative();
		}
	}
}
