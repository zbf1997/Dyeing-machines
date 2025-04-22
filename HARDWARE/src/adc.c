#include "adc.h"
#include "FreeRTOS.h"
#include "task.h"
volatile unsigned short m_ADCValue[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM] = {0};
//初始化ADC															   
void  ADC_DMA_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//使能DMA2时钟

	//先初始化ADC1通道0、1
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

	DMA_Cmd(DMA2_Stream0, DISABLE);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
 	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)(&(ADC1->DR));//外设地址（ADC的数据寄存器地址）
 	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)m_ADCValue;//存储器地址 (数组名就是变量)
 	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;//传输方式（外设ADC作为源）
 	DMA_InitStructure.DMA_BufferSize= ADC_SAMPLE_PNUM*ADC_SAMPLE_CNUM;//传输数目
 	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;//外设地址不递增
 	DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_HalfWord  ;//数据宽度(16位数据是两个字节)
 	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable ;//内存地址递增（由于是数组所以是递增）
 	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord ;
 	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular ;//循环模式
 	DMA_InitStructure.DMA_Priority=DMA_Priority_High ;//共有四种优先级
 	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO阈值，如果要传输到的存储器的宽度是一个字节，那么
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 	DMA_Init(DMA2_Stream0, &DMA_InitStructure);//ADC1对应DMA2的数据流0，通道0

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式，只使用一个ADC
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式,超过两个通道要使用扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换,将所有通道都转换完才停止
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_ExternalTrigConv =  0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_CNUM;//在规则序列中转换几个通道 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
	DMA_Cmd(DMA2_Stream0, ENABLE);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_480Cycles );
 	ADC_SoftwareStartConv(ADC1);

 }

float ADC_Proc_Value(unsigned char nIndex)//nIndex转换序列的次序,从1开始
{
	float temp;
	float qiyazhi;
	float ADC_Value[ADC_SAMPLE_CNUM];
	u32 nValue = 0;
	u32 nSum = 0;
	u8 i = 0, j = 0;
	u16 Temp = 0;	
	u16 tempArray[ADC_SAMPLE_PNUM] = {0};
	nIndex-=1;
	for	(i=0; i<ADC_SAMPLE_PNUM; i++)					//采集ADC_SAMPLE_PNUM组
	{
		tempArray[i] = m_ADCValue[i][nIndex];      
		//printf("\t\t\t******ADCValue:%X\r\n",tempArray[i]);
	}
	for(i=0; i<ADC_SAMPLE_PNUM-1; i++)						//去掉最大值
	{
		for(j = i+1; j< ADC_SAMPLE_PNUM ;j++){
			if(tempArray[j] < tempArray[i])
			{
				Temp = tempArray[i];
				tempArray[i] = tempArray[j];
				tempArray[j] = Temp;
			}
		}
	}	
	for(i=5; i<ADC_SAMPLE_PNUM-5; i++)						//去掉最小的数和最大的数再取平均
	{
		nSum += tempArray[i] ;
	}
	nValue = nSum / (ADC_SAMPLE_PNUM-10);//debug
	//printf("nValue=%d\r\n",nValue);
	//ADC输入电压值为0~3.3V转换为数字对应0-4096，压力变送器供电3.3V,输出电压值为0.33~2.97V对应0~-100KPa,在此范围内1v就代表37.88kPa
	temp=(float)nValue*(3.3f/4096);         	//获取计算后的带小数的实际电压值，比如0.3303
	//printf("temp=%f\r\n",temp);
	temp=(int)((temp*100)+0.5f)/100.0f;		 	//将小数点后第二位的数进行四舍五入
	if (temp<0.33f) temp=0.33f;         		//压力变送器输出的最小值为0.33v,但是单片机检测的时候可能会略小于0.33，比如说0.32
	qiyazhi=(temp-0.33f)*37.88f;				//因此需要对temp进行最小限定，防止temp-0.33变成负数
	return qiyazhi;								//输出值的单位为KPa
}






				  
// //获得ADC值
// //ch: @ref ADC_channels 
// //通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
// //返回值:转换结果
// u16 Get_Adc(u8 ch)   
// {
// 	  	//设置指定ADC的规则组通道，一个序列，采样时间
// 	//ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
// 	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
// 	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
// 	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
// }
// //获取通道ch的转换值，取times次,然后平均 
// //ch:通道编号
// //times:获取次数
// //返回值:通道ch的times次转换结果平均值
// u16 Get_Adc_Average(u8 ch,u8 times)
// {
// 	u32 temp_val=0;
// 	u8 t;
// 	for(t=0;t<times;t++)
// 	{
// 		temp_val+=Get_Adc(ch);
// 		vTaskDelay(5/portTICK_RATE_MS);
// 	}
// 	 return temp_val/times;
// } 


//ADC输入电压值为0~3.3V转换为数字对应0-4096，压力变送器供电3.3V,输出电压值为0.33~2.97V对应0~-100KPa,在此范围内1v就代表37.88kPa
// float Adc_Prosess(u8 nIndex)
// {
// 	u16 adcx=ADC_Proc_Value(nIndex);//获取通道5的转换值，10次取平均
// 	float temp;
// 	float qiyazhi;
// 	printf("adcx=%d\r\n",adcx);
// 	temp=(float)adcx*(3.3f/4096);          //获取计算后的带小数的实际电压值，比如0.3303
// 	printf("temp=%f\r\n",temp);
// 	adcx=temp;                            //赋值整数部分给adcx变量，因为adcx为u16整形
// 	temp=(int)((temp*100)+0.5f)/100.0f;		  //将小数点后第二位的数进行四舍五入
// 	if (temp<0.33f) temp=0.33f;//压力变送器输出的最小值为0.33v,但是单片机检测的时候可能会略小于0.33，比如说0.32
// 	qiyazhi=(temp-0.33f)*37.88f;//因此需要对temp进行最小限定，防止temp-0.33变成负数
// 	return qiyazhi;//输出值的单位为KPa
// }








