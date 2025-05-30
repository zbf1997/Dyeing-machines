/*********************************************************************************
 * 文件名  Read_Temperature.c
 * 描述    ：通过stm32的spi1读取max6675的温度值
 *          
 * 硬件连接： ------------------------------------
 *           |PA6-SPI1-MISO：MAX6675-SO          |
 *           |PA7-SPI1-MOSI：MAX6675-SI          |
 *           |PA5-SPI1-SCK ：MAX6675-SCK         |
 *           |PA4-SPI1-NSS ：MAX6675-CS          |
 *            ------------------------------------
**********************************************************************************/
#include "stm32f4xx.h"
#include "MAX6675.h"
#include "stdio.h"
#include "delay.h"
/*
 * 函数名：SPI1_Init
 * 描述  MAX6675 接口初始化
 * 输入  ：无 
 * 输出  ：无
 * 返回  ：无
 */																						  
void SPI_MAX6675_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;	
	
	/* 使能 SPI1 时钟 */                         
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* ---------通信I/O初始化----------------
	 * PA5-SPI1-SCK :MAX6675_SCK时钟
	 * PA6-SPI1-MISO:MAX6675_SO  //输入
	 * PA7-SPI1-MOSI:MAX6675_SI	 //输出
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// 推免输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	/* ---------控制I/O初始化----------------*/
	/* PA4-SPI1-NSS:MAX6675_CS片选 */			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// 推免输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						  
	GPIO_SetBits(GPIOA, GPIO_Pin_4);						// 先把片选拉高，真正用的时候再拉低

	/* SPI1 配置 */ 
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
 * 函数名：Read_Temperature
 * 描述  ：将热电偶传感器转换成摄氏度
 * 输入  ：无
 * 输出  ：温度值	
 */
float Read_MAX6675_Temperature (void)
{
	//max6675的转换时间是0.2秒左右，所以两次转换间隔不要太近
	unsigned int i;
	unsigned char c;
	unsigned char flag;
	float temprature;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//pin4置0
	i = MAX6675_ReadByte();
	i<<=8;
	i |= MAX6675_ReadByte();
	GPIO_SetBits(GPIOA, GPIO_Pin_4);//pin4置1
	
	// i = i|((unsigned int)c);			//i是读出来的原始数据
	flag = i&0x04;						//flag保存了热电偶的连接状态
	i=i<<1;
	i=i>>4;
	temprature = i*1023.75/4095;
	if(i!=0)							//max6675有数据返回
	{
		if(flag==0)						//热电偶已连接
		{
			printf("原始数据是：%04X,  当前温度是：%4.2f\r\n",i,temprature);
			return temprature;
		}	
		else							//热电偶掉线
		{
			printf("未检测到热电偶，请检查\r\n");
		}
	
	}
	else								//max6675没有数据返回
	{
		printf("max6675没有数据返回，请检查max6675连接\r\n");
	}		
}



