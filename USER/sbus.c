#include "sbus.h"
// #include "delay.h"
#include "gpio.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "common.h"
// #include "dma.h"

#define DMA_USE 0
u8 packet[25];
extern u16 CH_out[16];

u8 sbus_sta = 0;
u16 sbus_value[16];
u8 sbus_buf[25];
u8 sbus_status = 0;

void sbus_unpackage() // SBUS解包函数
{
    u8 i;
    u16 value_temp[16];
    value_temp[0] = ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF);
    value_temp[1] = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF);
    value_temp[2] = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF);
    value_temp[3] = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF);
    value_temp[4] = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
    value_temp[5] = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);
    value_temp[6] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    value_temp[7] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);

    value_temp[8] = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);
    value_temp[9] = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    value_temp[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
    value_temp[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);
    value_temp[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);
    value_temp[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF);
    value_temp[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);
    value_temp[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);
    for (i = 0; i < 16; i++)
    {
        value_temp[i] = (value_temp[i] + 1408) / 1.6; // 通道数值缩放
        if (value_temp[i] >= 500 && value_temp[i] <= 2500)
        {
            sbus_value[i] = value_temp[i];
            sbus_status = 5;
        }
    }
}

void USART1_IRQHandler(void) // 串口1中断服务程序
{
    u8 res;
    if (input_type) //SBUS_IN
    {
        if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        {
            res = USART_ReceiveData(USART1); // 读取数据
            if (sbus_sta)
            {
                sbus_buf[sbus_sta] = res;
                if (sbus_sta < 24)
                    sbus_sta++;
                else
                {
                    sbus_sta = 0;
                    sbus_unpackage();
                }
            }
            if (sbus_sta == 0)
            {
                if (res == 0x0f)
                {
                    sbus_buf[sbus_sta] = res;
                    sbus_sta = 1;
                }
            }
        }
    }
    else    //CRSF_IN
    {

    }
}
