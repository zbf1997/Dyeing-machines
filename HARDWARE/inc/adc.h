#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"  

#define ADC_SAMPLE_PNUM             250//AD ����������  10
#define ADC_SAMPLE_CNUM             2//AD ����ͨ����							   
void  ADC_DMA_Init(void);
float ADC_Proc_Value(unsigned char nIndex);
#endif 















