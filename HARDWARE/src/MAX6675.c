/*********************************************************************************
 * �ļ���  Read_Temperature.c
 * ����    ��ͨ��stm32��spi1��ȡmax6675���¶�ֵ
 *          
 * Ӳ�����ӣ� ------------------------------------
 *           |PA6-SPI1-MISO��MAX6675-SO          |
 *           |PA7-SPI1-MOSI��MAX6675-SI          |
 *           |PA5-SPI1-SCK ��MAX6675-SCK         |
 *           |PA4-SPI1-NSS ��MAX6675-CS          |
 *            ------------------------------------
**********************************************************************************/
#include "stm32f4xx.h"
#include "MAX6675.h"
#include "stdio.h"
#include "delay.h"
/*
 * ��������SPI1_Init
 * ����  �MMAX6675 �ӿڳ�ʼ��
 * ����  ���� 
 * ���  ����
 * ����  ����
 */																						  
void SPI_MAX6675_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;	
	
	/* ʹ�� SPI1 ʱ�� */                         
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* ---------ͨ��I/O��ʼ��----------------
	 * PA5-SPI1-SCK :MAX6675_SCKʱ��
	 * PA6-SPI1-MISO:MAX6675_SO  //����
	 * PA7-SPI1-MOSI:MAX6675_SI	 //���
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// �������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	/* ---------����I/O��ʼ��----------------*/
	/* PA4-SPI1-NSS:MAX6675_CSƬѡ */			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// �������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						  
	GPIO_SetBits(GPIOA, GPIO_Pin_4);						// �Ȱ�Ƭѡ���ߣ������õ�ʱ��������

	/* SPI1 ���� */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE); 
}

/*
 *
 *
 *
 */
unsigned char MAX6675_ReadByte(void)
{
	
	/* Loop while DR register in not emplty */
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send byte through the SPI1 peripheral */
	SPI_SendData(SPI1, 0xff);
	
	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_ReceiveData(SPI1);
}
/*
 * ��������Read_Temperature
 * ����  �����ȵ�ż������ת�������϶�
 * ����  ����
 * ���  ���¶�ֵ	
 */
float Read_MAX6675_Temperature (void)
{
	//max6675��ת��ʱ����0.2�����ң���������ת�������Ҫ̫��
	unsigned int i;
	unsigned char c;
	unsigned char flag;
	float temprature;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//pin4��0
	i = MAX6675_ReadByte();
	i<<=8;
	i |= MAX6675_ReadByte();
	GPIO_SetBits(GPIOA, GPIO_Pin_4);//pin4��1
	
	// i = i|((unsigned int)c);			//i�Ƕ�������ԭʼ����
	flag = i&0x04;						//flag�������ȵ�ż������״̬
	i=i<<1;
	i=i>>4;
	temprature = i*1023.75/4095;
	if(i!=0)							//max6675�����ݷ���
	{
		if(flag==0)						//�ȵ�ż������
		{
			printf("ԭʼ�����ǣ�%04X,  ��ǰ�¶��ǣ�%4.2f\r\n",i,temprature);
			return temprature;
		}	
		else							//�ȵ�ż����
		{
			printf("δ��⵽�ȵ�ż������\r\n");
		}
	
	}
	else								//max6675û�����ݷ���
	{
		printf("max6675û�����ݷ��أ�����max6675����\r\n");
	}		
}



