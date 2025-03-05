#include "NRF.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
#include "led.h"
extern u8 nrf_len;

const u8 TX_ADDRESS[TX_ADR_WIDTH] = {'L', 'O', 'V', 'E', '!'}; // ���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH] = {'L', 'O', 'V', 'E', '!'};
extern const char address_0[5];
extern u8 address[5];
extern u8 hopping[5];
// ��ʼ��24L01��IO��
void NRF24L01_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); // ʹ��PB,G�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2; // PB8  PB9 ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);				   // ��ʼ��ָ��IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // PB3 ����
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SPI2_Init();														// ��ʼ��SPI
	SPI_Cmd(SPI2, DISABLE);												// SPI���費ʹ��
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// SPI����
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					// ���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							// ʱ�����յ�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						// ���ݲ����ڵ�1��ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS�ź����������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// ���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);									// ����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	SPI_Cmd(SPI2, ENABLE);												// ʹ��SPI����
	NRF24L01_CE = 0;													// ʹ��24L01
	NRF24L01_CSN = 1;													// SPIƬѡȡ��
}

u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
	u8 i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);				 // spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); // д��5���ֽڵĵ�ַ.
	NRF24L01_Read_Buf(TX_ADDR, buf, 5);					 // ����д��ĵ�ַ
	for (i = 0; i < 5; i++)
		if (buf[i] != 0XA5)
			break;
	if (i != 5)
		return 1; // ���24L01����
	return 0;	  // ��⵽24L01
}

u8 NRF24L01_Write_Reg(u8 reg, u8 value)
{
	u8 status;
	NRF24L01_CSN = 0;				  // ʹ��SPI����
	status = SPI2_ReadWriteByte(reg); // ���ͼĴ�����
	SPI2_ReadWriteByte(value);		  // д��Ĵ�����ֵ
	NRF24L01_CSN = 1;				  // ��ֹSPI����
	return (status);				  // ����״ֵ̬
}

u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;
	NRF24L01_CSN = 0;					// ʹ��SPI����
	SPI2_ReadWriteByte(reg);			// ���ͼĴ�����
	reg_val = SPI2_ReadWriteByte(0XFF); // ��ȡ�Ĵ�������
	NRF24L01_CSN = 1;					// ��ֹSPI����
	return (reg_val);					// ����״ֵ̬
}

u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status, u8_ctr;
	NRF24L01_CSN = 0;				  // ʹ��SPI����
	status = SPI2_ReadWriteByte(reg); // ���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		pBuf[u8_ctr] = SPI2_ReadWriteByte(0XFF); // ��������
	NRF24L01_CSN = 1;							 // �ر�SPI����
	return status;								 // ���ض�����״ֵ̬
}

u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status, u8_ctr;
	NRF24L01_CSN = 0;				  // ʹ��SPI����
	status = SPI2_ReadWriteByte(reg); // ���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		SPI2_ReadWriteByte(*pBuf++); // д������
	NRF24L01_CSN = 1;				 // �ر�SPI����
	return status;					 // ���ض�����״ֵ̬
}

u8 TX(u8 *txbuf)
{
	u8 i = 0;
	u8 sta;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); // spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, nrf_len); // д���ݵ�TX BUF  32���ֽ�
	NRF24L01_CE = 1;								 // ��������	 +
	while (NRF24L01_IRQ != 0)
	{
		i++;
		if (i == 255)
			break;
	}
	sta = NRF24L01_Read_Reg(STATUS);				 // ��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // ���TX_DS��MAX_RT�жϱ�־
	if (sta & MAX_TX)								 // �ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); // ���TX FIFO�Ĵ���
		return MAX_TX;
	}
	if (sta & TX_OK) // �������
	{
		return TX_OK;
	}
	return 0xff; // ����ԭ����ʧ��
}

u8 RX(u8 *rxbuf)
{
	u8 sta;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);			 // spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
	sta = NRF24L01_Read_Reg(STATUS);				 // ��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // ���TX_DS��MAX_RT�жϱ�־
	if (sta & RX_OK)								 // ���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, nrf_len); // ��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);				// ���RX FIFO�Ĵ���
		return 1;
	}
	return 0; // û�յ��κ�����
}

void TX_Address(u8 *p)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, p, TX_ADR_WIDTH); // дTX�ڵ��ַ
	NRF24L01_CE = 1;
}
void RX_Address(u8 *p)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, p, RX_ADR_WIDTH); // ����RX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_CE = 1;
}

void NRF_power(u8 p, u8 s) // ���书������
{
	u8 temp = 0x01;
	if (p == 3)
		temp |= 0x06;
	else if (p == 2)
		temp |= 0x04;
	else if (p == 1)
		temp |= 0x02;
	else if (p == 0)
		temp |= 0x00;

	if (s == 2)
		temp |= 0x08; // 2Mbps
	else if (s == 1)
		temp |= 0x00; // 1Mbps
	else if (s == 0)
		temp |= 0x28; // 250Kbps

	NRF24L01_CE = 0;
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, temp); // 0db ����֮ǰע�ʹ���
	NRF24L01_CE = 1;
}

void NRF_channel(u8 c)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, c); // ����RFͨ��Ϊ40
	NRF24L01_CE = 1;
}

void TX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e); // ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF24L01_CE = 1;								  // CEΪ��,10us����������
}

void RX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f); // ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
	NRF24L01_CE = 1;								  // CEΪ��,�������ģʽ
}

void NRF_setup()
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (u8 *)address, TX_ADR_WIDTH);	 // дTX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (u8 *)address, RX_ADR_WIDTH); // ����RX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0);								 // ʹ��ͨ��0���Զ�Ӧ��
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 1);							 // ʹ��ͨ��0�Ľ��յ�ַ
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0);							 // �����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, nrf_len);						 // ѡ��ͨ��0����Ч���ݿ��
	// NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);//����TX�������,0db����,2Mbps,���������濪��
	NRF_power(3, 0); // ���书������
	RX_Mode();
	NRF_channel(hopping[0]);
	NRF24L01_CE = 1; // CEΪ��,�������ģʽ
}
