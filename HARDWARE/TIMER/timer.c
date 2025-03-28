#include "timer.h"

//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK精英STM32开发板
// 定时器 驱动代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2012/9/3
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// 通用定时器3中断初始化
// 这里时钟选择为APB1的2倍，而APB1为36M
// arr：自动重装值。
// psc：时钟预分频数
// 这里使用的是定时器3!
void TIM3_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		// 时钟使能
	TIM_TimeBaseStructure.TIM_Period = arr;						// 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);				// 根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);					// 使能指定的TIM3中断,允许更新中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				// TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	// 先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			// 从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);								// 初始化NVIC寄存器
}

// void TIM4_Int_Init(u16 ttt,u16 sss)//???4
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
//	TIM_TimeBaseStructure.TIM_Period=ttt;
//	TIM_TimeBaseStructure.TIM_Prescaler=sss; //(72 - 1)
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim																						/////////////////////////
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //????g?
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
// }

void TIM2_Int_Init(u16 ttt, u16 sss) //???2
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = ttt;
	TIM_TimeBaseStructure.TIM_Prescaler = sss;					//(72 - 1)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//??????:TDTS = Tck_tim																						/////////////////////////
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //????g?
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM1_Int_Init(u16 ttt, u16 sss) //???4
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = ttt;
	TIM_TimeBaseStructure.TIM_Prescaler = sss;					//(72 - 1)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;		//??????:TDTS = Tck_tim																						/////////////////////////
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //????g?
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// void TIM5_Int_Init(u16 ttt,u16 sss)//???4
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
//	TIM_TimeBaseStructure.TIM_Period=ttt;
//	TIM_TimeBaseStructure.TIM_Prescaler=sss; //(72 - 1)
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim																						/////////////////////////
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //????g?
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
// }
