// /*********************************************************************************
//  * 文件名  Read_MAX31865_Temperature.c
//  * 描述    ：通过stm32的spi1读取max6675的温度值
//  *          
//  * 硬件连接： ------------------------------------
//  *           |PB14-SPI1-MISO：MAX31865-SO          |
//  *           |PB15-SPI1-MOSI：MAX31865-SI          |
//  *           |PB13-SPI1-SCK ：MAX31865-SCK         |
//  *           |PB12-SPI1-NSS ：MAX31865-CS          |
//  *            ------------------------------------
// **********************************************************************************/
// #include "stm32f4xx.h"
// #include "MAX31865.h"
// #include "stdio.h"
// #include "delay.h"
// #include "math.h"
// /*
//  * 函数名：SPI1_Init
//  * 描述  MAX31865 接口初始化
//  * 输入  ：无 
//  * 输出  ：无
//  * 返回  ：无
//  */																						  
// void SPI_MAX31865_Init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	SPI_InitTypeDef  SPI_InitStructure;	
	
// 	/* 使能 SPI2时钟 */                         
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
// 	/* ---------通信I/O初始化----------------
// 	 * PB13-SPI2-SCK :MAX31865_SCK   时钟
// 	 * PB14-SPI2-MISO:MAX31865_SO    输入
// 	 * PB15-SPI2-MOSI:MAX31865_SI	 输出
// 	 */
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// 推免输出
// 	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);
// 	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
// 	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
// 	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

// 	/* ---------控制I/O初始化----------------*/
// 	/* PB12-SPI2-NSS:MAX31865_CS片选 */			
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// 推免输出
// 	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);						  
// 	GPIO_SetBits(GPIOB, GPIO_Pin_12);						// 先把片选拉高，真正用的时候再拉低

// 	/* SPI2 配置 */ 
// 	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
// 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
// 	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
// 	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
// 	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
// 	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
// 	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
// 	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
// 	SPI_InitStructure.SPI_CRCPolynomial = 7;
// 	SPI_Init(SPI2, &SPI_InitStructure);
// 	SPI_Cmd(SPI2, ENABLE); 
// }

 
// uint8_t MAX31865_SPI_Read(uint8_t addr)//SPI Single-Byte Read
 
// {
//     uint8_t read;
//     MAX31865_CS_LOW;
// 	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
// 	SPI_SendData(SPI2, addr);
// 	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
// 	read=SPI_ReceiveData(SPI2);
//     MAX31865_CS_HIGH;
//     return read;
// }

// void MAX31865_SPI_Write(uint8_t addr,uint8_t wdata)//SPI Single-Byte Write
// {
//     MAX31865_CS_LOW;
//  	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
// 	SPI_SendData(SPI2, addr);
// 	//while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
// 	SPI_SendData(SPI2, wdata);
//     MAX31865_CS_HIGH;
// }

// float Read_MAX31865_Temperature(void)//PT100
// {
// 	unsigned int data;
// 	float Rt;
// 	float Rt0 = 100; // PT100 0度对应的阻值 0-850时c=0;
// 	float Z1, Z2, Z3, Z4, temp;
// 	float a = 3.9083e-3;
// 	float b = -5.775e-7;
// 	float rpoly; //
// 	float temps;
// 	int temvalue = 0, RTDs = 0, i = 0;
// 	uint16_t data_r;
// 	data_r = MAX31865_SPI_Read(0x01) << 8;
// 	delay_ms(60);
// 	data_r |= MAX31865_SPI_Read(0x02);
// 	data_r >>= 1;
// 	printf("数字量为%d\r\n",data_r);
// 	temps = data_r;
// 	Rt = (float)temps / 32768.0 * REF_RES; // 阻值转换
// 	printf("阻值为%f\r\n",Rt);
// 	/*解一元二次方程*/
// 	Z1 = -a;
// 	Z2 = a * a - 4 * b;
// 	Z3 = 4 * b / Rt0;
// 	Z4 = 2 * b;
// 	temp = Z2 + Z3 * Rt;
// 	temp = (sqrt(temp) + Z1) / Z4;
// 	if (temp >= 0)
// 		return temp;
// 	rpoly = Rt;
// 	temp = -242.02;
// 	temp += 2.2228 * rpoly;
// 	rpoly *= Rt; // square
// 	temp += 2.5859e-3 * rpoly;
// 	rpoly *= Rt; // ^3
// 	temp -= 4.8260e-6 * rpoly;
// 	rpoly *= Rt; // ^4
// 	temp -= 2.8183e-8 * rpoly;
// 	rpoly *= Rt; // ^5
// 	temp += 1.5243e-10 * rpoly;
// 	return temp;
// }
 


#include "stm32f4xx.h"
#include "MAX31865.h"
#include "stdio.h"
#include "delay.h"
#include "math.h"
#include "max31865.h"
#include "timer.h"


void SPI_MAX31865_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;	
	
	/* 使能 SPI2时钟 */                         
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* ---------通信I/O初始化----------------
	 * PB13-SPI2-SCK :MAX31865_SCK   时钟
	 * PB14-SPI2-MISO:MAX31865_SO    输入
	 * PB15-SPI2-MOSI:MAX31865_SI	 输出
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// 推免输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	/* ---------控制I/O初始化----------------*/
	/* PB12-SPI2-NSS:MAX31865_CS片选 */			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// 推免输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						  
	GPIO_SetBits(GPIOB, GPIO_Pin_12);						// 先把片选拉高，真正用的时候再拉低

    /* -------MAX31865 REDY引脚初始化--------*/
	/* PB4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);



	/* SPI2 配置 */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE); 
}

void MAX31865Confg()
{
	//MAX31865_SB_Write(0x80,0xC2);//二线、四线配置
  	MAX31865_SB_Write(0x80,0xD2);//三线配置

  	delay_ms(10);
}


// void MAX31865_FaultDetect()
// {
// 	u8 fault_value;
// 	if(MAX31865_fault_state==1)
// 	{
// 		MAX31685_CS_LOW();
// 		fault_value = MAX31865_SB_Read(0x07);//读取故障状态寄存器(07h)
// 		MAX31685_CS_HIGH();
// 		MAX31865_fault_state=0;
// 		printf("MAX31865故障代码为：%d\r\n",fault_value);
// 	}
// }

float Read_MAX31865_Temperature(void)//PT100
{
	u8 fault_value;
	u8 MAX31865_fault_state=0;
	unsigned int data;
	float Rt;
	float Rt0 = 100; // PT100 0度对应的阻值 0-850时c=0;
	float Z1, Z2, Z3, Z4, temp;
	float a = 3.9083e-3;
	float b = -5.775e-7;
	float rpoly; //
	float temps;
	int temvalue = 0, RTDs = 0, i = 0;
	uint64_t tickCount=0;
	uint16_t data_r;
	tickCount=millis();
	while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==1)
	{
		if(millis()-tickCount==1000)//1秒超时
		{
			printf("等待MAX31865转换超时\r\n");
			break;
		}
	}
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)//DRDY引脚变低，说明MAX31865寄存器中有新的转换结果
	{
		MAX31685_CS_LOW();
		data_r = MAX31865_SB_Read(0x01) << 8;
		delay_ms(60);
		data_r |= MAX31865_SB_Read(0x02);
		MAX31685_CS_HIGH();
		MAX31865_fault_state=data_r & 0x01;
		data_r >>= 1;

		if(MAX31865_fault_state==0)
		{
			MAX31685_CS_LOW();
			fault_value = MAX31865_SB_Read(0x07);//读取故障状态寄存器(07h)
			MAX31865_SB_Write(0x80,0xD2);//三线故障位清零
			//MAX31865_SB_Write(0x80,0xC2);//二线、四线配置故障位清零
			MAX31685_CS_HIGH();
			printf("MAX31865故障代码为：%d\r\n",fault_value);
			return 10000.0;
		}

		else
		{
			printf("数字量为%d\r\n",data_r);
			temps = data_r;
			Rt = (float)temps / 32768.0 * REF_RES; // 阻值转换
			printf("阻值为%f\r\n",Rt);
			/*解一元二次方程*/
			Z1 = -a;
			Z2 = a * a - 4 * b;
			Z3 = 4 * b / Rt0;
			Z4 = 2 * b;
			temp = Z2 + Z3 * Rt;
			temp = (sqrt(temp) + Z1) / Z4;
			if (temp >= 0)
				return temp;
			rpoly = Rt;
			temp = -242.02;
			temp += 2.2228 * rpoly;
			rpoly *= Rt; // square
			temp += 2.5859e-3 * rpoly;
			rpoly *= Rt; // ^3
			temp -= 4.8260e-6 * rpoly;
			rpoly *= Rt; // ^4
			temp -= 2.8183e-8 * rpoly;
			rpoly *= Rt; // ^5
			temp += 1.5243e-10 * rpoly;
			return temp;
		}
	}
}

uint8_t MAX31865_SB_Read(uint8_t addr)//SPI Single-Byte Read
{
  	uint8_t read;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData(SPI2,addr);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	read=SPI_ReceiveData(SPI2);
	delay_ms(80);
  	return read;
}

void MAX31865_SB_Write(uint8_t addr,uint8_t wdata)//SPI Single-Byte Write
{
	uint8_t dat[2];
	uint16_t DATA;
	dat[0] = addr;
	dat[1] = wdata;
	MAX31685_CS_LOW();
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData(SPI2,dat[0]);
	//delay_ms(80);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData(SPI2,dat[1]);
	delay_ms(80);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
	delay_ms(80);
	MAX31685_CS_HIGH();
}
