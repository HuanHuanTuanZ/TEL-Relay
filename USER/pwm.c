
#include "timer.h"
#include "common.h"
extern u8 beep_flag;


int16 beep_queue_index[20][2];
u8    beep_queue_seq=0;

u8    beep_queue_flag=0;//���о�����־
	
//����������ע��
//����ע��
void beep_queue_regedit(u16 freq,u16 time)
{
	++beep_queue_seq;
	beep_queue_index[beep_queue_seq][0]=freq;
	beep_queue_index[beep_queue_seq][1]=time;
}

void beep_queue_operation()
{
	if(beep_flag)beep_flag--;
	else 
	{
		beep_queue_flag=0;	
		if(beep_queue_seq==0)beep(0,0);
		else
		{
			if(beep_queue_flag==0)
			{
				beep_queue_flag=1;
				beep(beep_queue_index[beep_queue_seq][0],beep_queue_index[beep_queue_seq][1]);
				beep_queue_seq--;		
			}
		}		
	}	
}

void beep_positive()
{
	beep_queue_regedit(1400,10);
	beep_queue_regedit(0,4);	
	beep_queue_regedit(1000,10);
}

void beep_negative()
{
	beep_queue_regedit(1000,10);
	beep_queue_regedit(0,4);	
	beep_queue_regedit(1400,10);
}



const u16 beep_freq_map[21]={262,294,330,349,392,440,494,523,587,659,698,784,880,988,1046,1175,1318,1397,1568,1760,1976};
u16 beep_freq_temp=0;
	
void beep(u16 freq,u16 time)
{
	if(beep_freq_temp!=freq)
	{
		beep_freq_temp=freq;	
		if(beep_enable)
		{					
			if(freq==0)
			{			
				//TIM_SetCompare3(TIM4,0);	
				TIM_Cmd(TIM4, DISABLE);  //ʹ��TIM4
			}
			else if(freq<21)
			{
				BEEP_Init(1000000/beep_freq_map[freq]-1,CPU_SPEED/2-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.
				TIM_SetCompare3(TIM4,500000/beep_freq_map[freq]);
			}
			else
			{
				BEEP_Init(1000000/freq-1,CPU_SPEED/2-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.
				TIM_SetCompare3(TIM4,5000*100/freq);
			}
		}			
		else TIM_SetCompare3(TIM4,0);			
	}
	beep_flag=time;
}



void BEEP_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //Timer3������ӳ��  TIM4_CH2->PB5    
 
   //���ø�����Ϊ�����������,���TIM4 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM4 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	
	
	TIM_SetCompare3(TIM4,0);	

}


void backlight(u8 light)
{
	TIM_SetCompare4(TIM4,light*10);	
}

void IPS_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //Timer3������ӳ��  TIM4_CH2->PB5    
 
   //���ø�����Ϊ�����������,���TIM4 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM4 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	TIM_SetCompare4(TIM4,0);	
}


