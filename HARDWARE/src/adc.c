#include "adc.h"
#include "FreeRTOS.h"
#include "task.h"
volatile unsigned short m_ADCValue[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM] = {0};
//��ʼ��ADC															   
void  ADC_DMA_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//ʹ��DMA2ʱ��

	//�ȳ�ʼ��ADC1ͨ��0��1
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 

	DMA_Cmd(DMA2_Stream0, DISABLE);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
 	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)(&(ADC1->DR));//�����ַ��ADC�����ݼĴ�����ַ��
 	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)m_ADCValue;//�洢����ַ (���������Ǳ���)
 	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;//���䷽ʽ������ADC��ΪԴ��
 	DMA_InitStructure.DMA_BufferSize= ADC_SAMPLE_PNUM*ADC_SAMPLE_CNUM;//������Ŀ
 	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;//�����ַ������
 	DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_HalfWord  ;//���ݿ��(16λ�����������ֽ�)
 	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable ;//�ڴ��ַ���������������������ǵ�����
 	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord ;
 	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular ;//ѭ��ģʽ
 	DMA_InitStructure.DMA_Priority=DMA_Priority_High ;//�����������ȼ�
 	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;//FIFO��ֵ�����Ҫ���䵽�Ĵ洢���Ŀ����һ���ֽڣ���ô
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 	DMA_Init(DMA2_Stream0, &DMA_InitStructure);//ADC1��ӦDMA2��������0��ͨ��0

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ��ֻʹ��һ��ADC
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ��ģʽ,��������ͨ��Ҫʹ��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��,������ͨ����ת�����ֹͣ
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_ExternalTrigConv =  0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_CNUM;//�ڹ���������ת������ͨ�� 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);//����ADת����
	DMA_Cmd(DMA2_Stream0, ENABLE);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_480Cycles );
 	ADC_SoftwareStartConv(ADC1);

 }

float ADC_Proc_Value(unsigned char nIndex)//nIndexת�����еĴ���,��1��ʼ
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
	for	(i=0; i<ADC_SAMPLE_PNUM; i++)					//�ɼ�ADC_SAMPLE_PNUM��
	{
		tempArray[i] = m_ADCValue[i][nIndex];      
		//printf("\t\t\t******ADCValue:%X\r\n",tempArray[i]);
	}
	for(i=0; i<ADC_SAMPLE_PNUM-1; i++)						//ȥ�����ֵ
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
	for(i=5; i<ADC_SAMPLE_PNUM-5; i++)						//ȥ����С��������������ȡƽ��
	{
		nSum += tempArray[i] ;
	}
	nValue = nSum / (ADC_SAMPLE_PNUM-10);//debug
	//printf("nValue=%d\r\n",nValue);
	//ADC�����ѹֵΪ0~3.3Vת��Ϊ���ֶ�Ӧ0-4096��ѹ������������3.3V,�����ѹֵΪ0.33~2.97V��Ӧ0~-100KPa,�ڴ˷�Χ��1v�ʹ���37.88kPa
	temp=(float)nValue*(3.3f/4096);         	//��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������0.3303
	//printf("temp=%f\r\n",temp);
	temp=(int)((temp*100)+0.5f)/100.0f;		 	//��С�����ڶ�λ����������������
	if (temp<0.33f) temp=0.33f;         		//ѹ���������������СֵΪ0.33v,���ǵ�Ƭ������ʱ����ܻ���С��0.33������˵0.32
	qiyazhi=(temp-0.33f)*37.88f;				//�����Ҫ��temp������С�޶�����ֹtemp-0.33��ɸ���
	return qiyazhi;								//���ֵ�ĵ�λΪKPa
}






				  
// //���ADCֵ
// //ch: @ref ADC_channels 
// //ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
// //����ֵ:ת�����
// u16 Get_Adc(u8 ch)   
// {
// 	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
// 	//ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
// 	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
// 	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
// 	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
// }
// //��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
// //ch:ͨ�����
// //times:��ȡ����
// //����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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


//ADC�����ѹֵΪ0~3.3Vת��Ϊ���ֶ�Ӧ0-4096��ѹ������������3.3V,�����ѹֵΪ0.33~2.97V��Ӧ0~-100KPa,�ڴ˷�Χ��1v�ʹ���37.88kPa
// float Adc_Prosess(u8 nIndex)
// {
// 	u16 adcx=ADC_Proc_Value(nIndex);//��ȡͨ��5��ת��ֵ��10��ȡƽ��
// 	float temp;
// 	float qiyazhi;
// 	printf("adcx=%d\r\n",adcx);
// 	temp=(float)adcx*(3.3f/4096);          //��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������0.3303
// 	printf("temp=%f\r\n",temp);
// 	adcx=temp;                            //��ֵ�������ָ�adcx��������ΪadcxΪu16����
// 	temp=(int)((temp*100)+0.5f)/100.0f;		  //��С�����ڶ�λ����������������
// 	if (temp<0.33f) temp=0.33f;//ѹ���������������СֵΪ0.33v,���ǵ�Ƭ������ʱ����ܻ���С��0.33������˵0.32
// 	qiyazhi=(temp-0.33f)*37.88f;//�����Ҫ��temp������С�޶�����ֹtemp-0.33��ɸ���
// 	return qiyazhi;//���ֵ�ĵ�λΪKPa
// }








