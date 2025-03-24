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
u16 crsf_value[16]; // CRSF通道数值
u8 crsf_rc_buf[30];
u8 sbus_buf[25];
u8 sbus_status = 0;
u16 USART1_RX_STA = 0;
u8 CRSF_RC_start = 0; // 接受CRSF RC信息开始标志
u8 crsf_rc_len = 0;

void sbus_unpackage() // SBUS解包函数
{
    sbus_value[0] = ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF);
    sbus_value[1] = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF);
    sbus_value[2] = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF);
    sbus_value[3] = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF);
    sbus_value[4] = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
    sbus_value[5] = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);
    sbus_value[6] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    sbus_value[7] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    sbus_value[8] = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);
    sbus_value[9] = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    sbus_value[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
    sbus_value[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);
    sbus_value[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);
    sbus_value[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF);
    sbus_value[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);
    sbus_value[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);
}

void crsf_rc_depackage()
{
    crsf_value[0] = ((uint16_t)crsf_rc_buf[3] >> 0 | ((uint16_t)crsf_rc_buf[4] << 8)) & 0x07FF;
    crsf_value[1] = ((uint16_t)crsf_rc_buf[4] >> 3 | ((uint16_t)crsf_rc_buf[5] << 5)) & 0x07FF;
    crsf_value[2] = ((uint16_t)crsf_rc_buf[5] >> 6 | ((uint16_t)crsf_rc_buf[6] << 2) | ((uint16_t)crsf_rc_buf[7] << 10)) & 0x07FF;
    crsf_value[3] = ((uint16_t)crsf_rc_buf[7] >> 1 | ((uint16_t)crsf_rc_buf[8] << 7)) & 0x07FF;
    crsf_value[4] = ((uint16_t)crsf_rc_buf[8] >> 4 | ((uint16_t)crsf_rc_buf[9] << 4)) & 0x07FF;
    crsf_value[5] = ((uint16_t)crsf_rc_buf[9] >> 7 | ((uint16_t)crsf_rc_buf[10] << 1) | ((uint16_t)crsf_rc_buf[11] << 9)) & 0x07FF;
    crsf_value[6] = ((uint16_t)crsf_rc_buf[11] >> 2 | ((uint16_t)crsf_rc_buf[12] << 6)) & 0x07FF;
    crsf_value[7] = ((uint16_t)crsf_rc_buf[12] >> 5 | ((uint16_t)crsf_rc_buf[13] << 3)) & 0x07FF;
    crsf_value[8] = ((uint16_t)crsf_rc_buf[14] >> 0 | ((uint16_t)crsf_rc_buf[15] << 8)) & 0x07FF;
    crsf_value[9] = ((uint16_t)crsf_rc_buf[15] >> 3 | ((uint16_t)crsf_rc_buf[16] << 5)) & 0x07FF;
    crsf_value[10] = ((uint16_t)crsf_rc_buf[16] >> 6 | ((uint16_t)crsf_rc_buf[17] << 2) | ((uint16_t)crsf_rc_buf[18] << 10)) & 0x07FF;
    crsf_value[11] = ((uint16_t)crsf_rc_buf[18] >> 1 | ((uint16_t)crsf_rc_buf[19] << 7)) & 0x07FF;
    crsf_value[12] = ((uint16_t)crsf_rc_buf[19] >> 4 | ((uint16_t)crsf_rc_buf[20] << 4)) & 0x07FF;
    crsf_value[13] = ((uint16_t)crsf_rc_buf[20] >> 7 | ((uint16_t)crsf_rc_buf[21] << 1) | ((uint16_t)crsf_rc_buf[22] << 9)) & 0x07FF;
    crsf_value[14] = ((uint16_t)crsf_rc_buf[22] >> 2 | ((uint16_t)crsf_rc_buf[23] << 6)) & 0x07FF;
    crsf_value[15] = ((uint16_t)crsf_rc_buf[23] >> 5 | ((uint16_t)crsf_rc_buf[24] << 3)) & 0x07FF;
}

void USART1_IRQHandler(void) // 串口1中断服务程序
{
    u8 res;
    if (input_type) // CRSF_IN
    {
        if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
        {
            res = USART_ReceiveData(USART1); //(USART1->DR);	//读取接收到的数据
            if (CRSF_RC_start == 3)          // 数据接收
            {
                crsf_rc_buf[USART1_RX_STA++] = res;
                if (USART1_RX_STA == (crsf_rc_len + 2)) // 接收完成
                {
                    CRSF_RC_start = 0;
                    CRSF_TLM_flag = 3;
                    crsf_rc_depackage();
                }
            }
            else if (CRSF_RC_start == 2) // 获取CRSF回传ID
            {
                crsf_rc_buf[USART1_RX_STA++] = res;
                if (res == 0x16) // 如果是RC
                    CRSF_RC_start = 3;
                else
                    CRSF_RC_start = 0; // 未找到ID
            }
            else if (CRSF_RC_start == 1) // 记录包长度
            {
                crsf_rc_buf[USART1_RX_STA++] = res;
                CRSF_RC_start = 2;
                crsf_rc_len = res;
            }
            if (res == 0xEA) // 开始接收
            {
                USART1_RX_STA = 0;
                crsf_rc_buf[USART1_RX_STA++] = res;
                CRSF_RC_start = 1;
            }
        }
    }
    else // SBUS_IN
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
}
