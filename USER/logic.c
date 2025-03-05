#include "common.h"
// #include "lcd.h"
#include "XUI.h"
#include "math.h"
#include "usart.h"

#define logic_x_count 6 // �˵�������
#define logic_y_count 4 // �˵�������

#define logic_max 999
#define logic_mode_value 11000
#define logic_ok_value 22000

u8 logic_condition[logic_count];	 // �߼���������
int16 logic_a[logic_count];			 // �߼�����ͨ��
int16 logic_x_value[logic_count];	 // �߼������ж�ֵX
int16 logic_x[logic_count];			 // �߼������ж�ֵXͨ��
int16 logic_last_time[logic_count];	 // ����ʱ��
u32 logic_current_time[logic_count]; // ��ǰʱ��
u8 logic_flag[logic_count];			 // ״̬��־
u8 logic_status[logic_count];		 // ����״̬
u8 logic_status_temp[logic_count];	 // ״̬����
u8 logic_status_last[logic_count];	 // �ϴο���״̬
u8 logic_mode[logic_count];			 // ����ģʽ

#define logic_condtion_count 15
const char logic_condtion_index[][4] =
	{
		" -- ", // 0
		"  A ",
		" A=X",
		"A!=X",
		" A<X",
		" A>X",
		" A*X",
		"A|>X",
		"A|<X",
		" AND",
		"  OR",
		"NAND",
		" NOR",
		" XOR",
		" NOT",
		//	" ��ʱ ",
		//	" ���� ",
		//	" ���� ",//19
};

#define logic_source_count 49
const char logic_source_index[][4] =
	{
		" -- ",
		"CH01",
		"CH02",
		"CH03",
		"CH04",
		"CH05",
		"CH06",
		"CH07",
		"CH08",
		"CH09",
		"CH10",
		"CH11",
		"CH12",
		"CH13",
		"CH14",
		"CH15",
		"CH16",
		"ADC1",
		"ADC2",
		"ADC3",
		"ADC4",
		"ADC5",
		"ADC6",
		"ADC7",
		"ADC8",
		"PPM1",
		"PPM2",
		"PPM3",
		"PPM4",
		"PPM5",
		"PPM6",
		"PPM7",
		"PPM8",
		"SBS1",
		"SBS2",
		"SBS3",
		"SBS4",
		"SBS5",
		"SBS6",
		"SBS7",
		"SBS8",
		"SBS9",
		"SB10",
		"SB11",
		"SB12",
		"SB13",
		"SB14",
		"SB15",
		"SB16",
};

int16 logic_source(u8 seq)
{
	if (seq == 0)
		return 0;
	else if (seq <= 16)
		return (CH_input[seq - 1] - 1500) / 5;
	else if (seq <= 24)
		return (hardware_value[seq - 17]) / 5;
	else if (seq <= 32)
		return (ppm_value[seq - 25]) / 5;
	else if (seq <= 48)
		return (sbus_value[seq - 33]) / 5;
	else
		return 0;
}

u8 aaaa(int16 a)
{
	if (a)
		return 1;
	else
		return 0;
}

u8 a_same_x(int16 a, int16 x)
{
	if (a == x)
		return 1;
	else
		return 0;
}
u8 a_notsame_x(int16 a, int16 x)
{
	if (a != x)
		return 1;
	else
		return 0;
}
u8 a_less_x(int16 a, int16 x)
{
	if (a < x)
		return 1;
	else
		return 0;
}
u8 a_than_x(int16 a, int16 x)
{
	if (a > x)
		return 1;
	else
		return 0;
}
u8 ax(int16 a, int16 x)
{
	if (a * x)
		return 1;
	return 0;
}
u8 abs_a_than_x(int16 a, int16 x)
{
	if (abs(a) > x)
		return 1;
	else
		return 0;
}
u8 abs_a_less_x(int16 a, int16 x)
{
	if (abs(a) < x)
		return 1;
	else
		return 0;
}
u8 a_and_x(int16 a, int16 x)
{
	if (a && x)
		return 1;
	return 0;
}
u8 a_or_x(int16 a, int16 x)
{
	if (a || x)
		return 1;
	return 0;
}
u8 a_nand_x(int16 a, int16 x)
{
	if (a && x)
		return 0;
	else
		return 1;
}
u8 a_nor_x(int16 a, int16 x)
{
	if ((a || x) == 0)
		return 1;
	else
		return 0;
}
u8 a_xor_x(int16 a, int16 x)
{
	if (a && x)
		return 0;
	else if ((!a) && (!x))
		return 0;
	else
		return 1;
}
u8 not_a(int16 a)
{
	if (a)
		return 0;
	return 1;
}

u8 logic_condtion(u8 seq)
{
	int16 temp;

	if (logic_x_value[seq] > logic_mode_value)
		temp = logic_source(logic_x[seq]);
	else
		temp = logic_x_value[seq];

	switch (logic_condition[seq])
	{
	case 0:
		return 0;
	case 1:
		return aaaa(logic_source(logic_a[seq]));
	case 2:
		return a_same_x(logic_source(logic_a[seq]), temp);
	case 3:
		return a_notsame_x(logic_source(logic_a[seq]), temp);
	case 4:
		return a_less_x(logic_source(logic_a[seq]), temp);
	case 5:
		return a_than_x(logic_source(logic_a[seq]), temp);
	case 6:
		return ax(logic_source(logic_a[seq]), temp);
	case 7:
		return abs_a_than_x(logic_source(logic_a[seq]), temp);
	case 8:
		return abs_a_less_x(logic_source(logic_a[seq]), temp);
	case 9:
		return a_and_x(logic_source(logic_a[seq]), temp);
	case 10:
		return a_or_x(logic_source(logic_a[seq]), temp);
	case 11:
		return a_nand_x(logic_source(logic_a[seq]), temp);
	case 12:
		return a_nor_x(logic_source(logic_a[seq]), temp);
	case 13:
		return a_xor_x(logic_source(logic_a[seq]), temp);
	case 14:
		return not_a(logic_source(logic_a[seq]));
	}
	return 0;
}

void logic_operation()
{
	u8 i;
	for (i = 0; i < logic_count; i++)
	{
		if (logic_status_temp[i]) // ����߼���
		{
			if (logic_flag[i] == 0)
			{
				logic_flag[i] = 1;
				logic_current_time[i] = time_100ms;
			}
			if (logic_flag[i] == 1)
			{
				if (logic_last_time[i] == -1)
				{
					logic_status[i] = !logic_status[i];
					logic_flag[i] = 2;
				}
				else if (logic_last_time[i] == -2)
				{
					logic_status[i] = 1;
					logic_flag[i] = 100;
				}
				else
				{
					logic_status[i] = 1;
					logic_flag[i] = 2;
				}
			}
			if (logic_last_time[i] > 0) // ����ʱ��Ϊ�㣬��Ϊ����
			{
				if (logic_flag[i] == 2) // �Ѿ���ʼ�������ʱ�����
				{
					logic_flag[i] = 3;
					logic_current_time[i] = time_100ms; // ��¼��ǰʱ��
				}
				if (logic_flag[i] == 3) // ��ʼ����ʱ������ȴ�
				{
					if ((time_100ms - logic_current_time[i]) >= logic_last_time[i]) // ʱ�䵽��
					{
						logic_status[i] = 0; // �߼���0
						if (logic_mode[i] == 1)
							logic_flag[i] = 0; // ���Ϊ�ظ���ʽ�����־��0����ʼ��һ��ѭ��
						else
							logic_flag[i] = 4; // �����־��4������ʼ��һ��ѭ��
					}
				}
			}
		}
		else
		{
			if (logic_flag[i] == 1 || logic_flag[i] == 4)
				logic_flag[i] = 0;

			if (logic_last_time[i] > 0) // ����ʱ��
			{
				if (logic_flag[i] == 2)
				{
					logic_flag[i] = 3;
					logic_current_time[i] = time_100ms;
				}
				if (logic_flag[i] == 3)
				{
					if ((time_100ms - logic_current_time[i]) >= logic_last_time[i])
					{
						logic_status[i] = 0;
						logic_flag[i] = 0;
					}
				}
			}
			else if (logic_last_time[i] == 0) // ����
			{
				logic_status[i] = 0;
				logic_flag[i] = 0;
			}
			else if (logic_last_time[i] == -1)
				logic_flag[i] = 0; // �л�
		}
	}
	for (i = 0; i < logic_count; i++)
		logic_status_temp[i] = logic_condtion(i);
}

void logic_reset_param(u8 seq)
{
	logic_a[seq] = 0;
	logic_x_value[seq] = 0;
	logic_x[seq] = 0;
	logic_last_time[seq] = 0;
	logic_mode[seq] = 0;
}

// void logic_init()
//{
//	u8 i=0;
//	for(i=0;i<logic_count;i++)
//	{
//		L[i].a=0;
//		L[i].x_value=0;
//		L[i].x=0;
//		L[i].last_time=0;
//		L[i].mode=0;
//	}
// }

void draw_logic_info(u8 i)
{
	u8 back_color = BC, point_color = PC;
	u8 seq = menu_y + i;
	u8 mode = 1;
	if (py == seq)
	{
		if (param_edit == 0)
		{
			back_color = SC;
			point_color = PC;
		}
		else
		{
			back_color = PC;
			point_color = BC;
		}
		mode = 0;
	}
	XUI_str(0, 32 + i * 12, 24, 12, (u8 *)"LG0", 12, px == 0 ? mode : 1, px == 0 ? point_color : PC, px == 0 ? back_color : BC);
	if (seq < 9)
		XUI_num(18, 32 + i * 12, 1 + seq, 1, 12, px == 0 ? mode : 1, px == 0 ? point_color : PC, px == 0 ? back_color : BC);
	else
		XUI_num(12, 32 + i * 12, 1 + seq, 2, 12, px == 0 ? mode : 1, px == 0 ? point_color : PC, px == 0 ? back_color : BC);
	XUI_str(27, 32 + i * 12, 24, 12, (u8 *)logic_condtion_index[logic_condition[seq]], 12, px == 1 ? mode : 1, px == 1 ? point_color : PC, px == 1 ? back_color : BC);
	if (logic_condition[seq])
	{
		XUI_str(54, 32 + i * 12, 24, 12, (u8 *)logic_source_index[logic_a[seq]], 12, px == 2 ? mode : 1, px == 2 ? point_color : PC, px == 2 ? back_color : BC);

		if (logic_x_value[seq] > logic_mode_value)
			XUI_str(81, 32 + i * 12, 24, 12, (u8 *)logic_source_index[logic_x[seq]], 12, px == 3 ? mode : 1, px == 3 ? point_color : PC, px == 3 ? back_color : BC);
		else
			XUI_num(93 - len(logic_x_value[seq]) * 3, 32 + i * 12, logic_x_value[seq], len(logic_x_value[seq]), 12, px == 3 ? mode : 1, px == 3 ? point_color : PC, px == 3 ? back_color : BC);
		if (logic_last_time[seq] == 0)
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)"����", 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC);
		else if (logic_last_time[seq] == -1)
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)"�л�", 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC); // �ﵽ�����򴥷���ʧȥ�������֣��ٴδ�����ָ�
		else if (logic_last_time[seq] == -2)
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)"����", 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC);
		else if (logic_last_time[seq] < 100)
		{
			char temp[10];
			sprintf(temp, "%1.1fS", logic_last_time[seq] / 10.0);
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)temp, 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC);
		}
		else if (logic_last_time[seq] < 1000)
		{
			char temp[10];
			sprintf(temp, " %dS", logic_last_time[seq] / 10);
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)temp, 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC);
		}
		else if (logic_last_time[seq] < 10000)
		{
			char temp[10];
			sprintf(temp, "%dS", logic_last_time[seq] / 10);
			XUI_str(108, 32 + i * 12, 24, 12, (u8 *)temp, 12, px == 4 ? mode : 1, px == 4 ? point_color : PC, px == 4 ? back_color : BC);
		}
		// XUI_fill(137,32+i*12,157,32+i*12+11,px==5?back_color:BC);
		if (logic_mode[seq]) // �ظ�����
		{
			if (logic_status[seq])
				XUI_bar(147, 32 + i * 12 + 6, 8, 3, 3, GREEN8);
			else
				XUI_bar(147, 32 + i * 12 + 6, 8, 3, 3, RED8);
		}
		else // ���δ���
		{
			if (logic_status[seq])
				XUI_fill(139, 32 + i * 12 + 3, 155, 32 + i * 12 + 9, GREEN8);
			else
				XUI_fill(139, 32 + i * 12 + 3, 155, 32 + i * 12 + 9, RED8);
		}
	}
}

void logic_reset_status(u8 seq)
{
	logic_flag[seq] = 0;
	logic_status[seq] = 0;
}

void logic_setup()
{
	u8 i;
	if (first_splash)
	{
		first_splash = 0;
		if (logic_count > logic_y_count)
			menu_len = logic_y_count;
		else
			menu_len = logic_count;
		px = py = 0;
		menu_y = 0;
	}
	XUI_str(56, 4, 64, 12, (u8 *)"�߼�����", 12, 1, PC, BC);
	XUI_str(0, 20, 24, 12, (u8 *)"���", 12, 1, RC, BC);
	XUI_str(27, 20, 24, 12, (u8 *)"����", 12, 1, RC, BC);
	XUI_str(54, 20, 24, 12, (u8 *)"����", 12, 1, RC, BC);
	XUI_str(81, 20, 24, 12, (u8 *)"��ֵ", 12, 1, RC, BC);
	XUI_str(108, 20, 24, 12, (u8 *)"����", 12, 1, RC, BC);
	XUI_str(135, 20, 24, 12, (u8 *)"״̬", 12, 1, RC, BC);
	for (i = 0; i < menu_len; i++) // ȫ��ˢ��
		draw_logic_info(i);
	if (LEFT)
	{
		LEFT = 0;
		if (px)
			px--;
		else
			px = logic_x_count - 1;

		if (logic_condition[py] == 0)
		{
			if (px > 1)
				px = 1;
		}
	}
	if (RIGHT)
	{
		RIGHT = 0;
		if (px < (logic_x_count - 1))
			px++;
		else
			px = 0;
		if (logic_condition[py] == 0)
		{
			if (px > 1)
				px = 0;
		}
	}
	if (param_edit)
	{
		if (ESC == 1 || OK == 1)
		{
			ESC = 0;
			OK = 0;
			param_edit = 0;

			save16(model_addr + 144 + py, logic_condition[py]);
			save16(model_addr + 148 + py, logic_a[py]);
			save16(model_addr + 152 + py, logic_x_value[py]);
			save16(model_addr + 156 + py, logic_last_time[py]);
			save16(model_addr + 160 + py, logic_mode[py]);
			// �������ĳ���
		}
		if (UP)
		{
			if (px)
				logic_reset_status(py); // �߼�״̬��λ
			if (px == 1)
			{
				if (logic_condition[py] < (logic_condtion_count - 1))
					logic_condition[py]++;
				else
					logic_condition[py] = 0;
			}
			if (logic_condition[py])
			{
				switch (px)
				{
				case 2:
					if (logic_a[py] < (logic_source_count - 1))
						logic_a[py]++;
					else
						logic_a[py] = 0;
					break;
				case 3:
					if (logic_x_value[py] > logic_mode_value)
					{
						if (logic_x[py] < logic_source_count)
							logic_x[py]++;
						else
							logic_x[py] = 0;
					}
					else if (logic_x_value[py] < logic_max) // ��ֵģʽ
					{
						logic_x_value[py] += UP;
						if (logic_x_value[py] >= logic_max)
							logic_x_value[py] = logic_max;
					}
					break;
				case 4:
					if (logic_last_time[py] < 100)
						logic_last_time[py] += UP;
					else if (logic_last_time[py] < 9990)
						logic_last_time[py] += UP * 10;
					if (logic_last_time[py] > 9990)
						logic_last_time[py] = 9990;
					break;
				case 5:
					logic_mode[py] = !logic_mode[py];
					break;
				}
			}
			UP = 0;
		}
		if (DOWN)
		{
			if (px)
				logic_reset_status(py); // �߼�״̬��λ
			if (px == 1)
			{
				if (logic_condition[py])
					logic_condition[py]--;
				else
					logic_condition[py] = logic_condtion_count - 1;
			}
			if (logic_condition[py])
			{
				switch (px)
				{
				case 2:
					if (logic_a[py])
						logic_a[py]--;
					else
						logic_a[py] = logic_source_count - 1;
					break;
				case 3:
					if (logic_x_value[py] > logic_mode_value) // ӳ��ģʽ
					{
						if (logic_x[py])
							logic_x[py]--;
						else
							logic_x[py] = logic_source_count - 1;
					}
					else if (logic_x_value[py] > -logic_max) // ��ֵģʽ
					{
						logic_x_value[py] -= DOWN;
						if (logic_x_value[py] <= -logic_max)
							logic_x_value[py] = -logic_max;
					}
					break;
				case 4:
					if (logic_last_time[py] > 100)
						logic_last_time[py] -= DOWN * 10;
					else if (logic_last_time[py] >= DOWN)
						logic_last_time[py] -= DOWN;
					else
						logic_last_time[py]--;
					if (logic_last_time[py] <= -2)
						logic_last_time[py] = -2;
					break;
				case 5:
					logic_mode[py] = !logic_mode[py];
					break;
				}
			}
			DOWN = 0;
		}
	}
	else
	{
		if (UP)
		{
			UP = 0;
			if (py)
				py--;
			else
			{
				py = logic_count - 1;
				if (logic_count > logic_y_count)
					menu_y = logic_count - logic_y_count;
				else
					menu_y = 0;
			}
			if (py < menu_y)
				menu_y--;
			if (mixer_type[py] == 0)
			{
				if (px > 1)
					px = 1;
			}
		}
		if (DOWN)
		{
			DOWN = 0;
			py++;
			if (py >= logic_count)
			{
				py = 0;
				menu_y = 0;
			}
			else if (py >= (menu_y + logic_y_count))
				menu_y++;
			if (mixer_type[py] == 0)
			{
				if (px > 1)
					px = 1;
			}
		}
		if (ESC == 1)
		{
			ESC = 0;
			ui_menu_flag = 0;
		}
		if (OK == 1)
		{
			OK = 0;
			param_edit = 1;
		}
	}
}
