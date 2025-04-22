#include "stm32f4xx.h"
#include "InputSlicDetect.h"
#include "Globalvariable.h"

void EXTI9_5_IRQHandler(void)

{
  // if (EXTI_GetITStatus(EXTI_Line5) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line5);
  //   SlicSensor1count++;
  //   printf("�����ⲿ�ж���5,SlicSensor1count=%d\r\n",SlicSensor1count);
  // }

  // if (EXTI_GetITStatus(EXTI_Line6) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line6);
  //   SlicSensor2count++;
  //   printf("�����ⲿ�ж���6,SlicSensor2count=%d\r\n",SlicSensor2count);
  // }

  if (EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line7);
    SlicSensor3count++;
    printf("�����ⲿ�ж���7,SlicSensor3count=%d\r\n",SlicSensor3count);
  }
}

//�ⲿ�жϳ�ʼ������
void InputSlicDetect_Init(void)
{
	// NVIC_InitTypeDef  NVIC_InitStructure;
	// EXTI_InitTypeDef  EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOA
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
 
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);//PA5 ���ӵ��ж���5
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//PA6 ���ӵ��ж���6
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);//PA7 ���ӵ��ж���7

	/*����GPIO��*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  // /* ����EXTI_Line*/
  // EXTI_InitStructure.EXTI_Line = EXTI_Line5|EXTI_Line6|EXTI_Line7;
  // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش��� 
  // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  // EXTI_Init(&EXTI_InitStructure);
	
	// NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x07;//��ռ���ȼ�7
  // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure);//����

  // /*�ر��жϣ����û��źŲſ����ⲿ�ж�*/
  // EXTI->IMR &= ~EXTI_Line5;   // �ر�EXTI5�ж�����
  // EXTI->IMR &= ~EXTI_Line6;   // �ر�EXTI6�ж�����
  // EXTI->IMR &= ~EXTI_Line7;   // �ر�EXTI7�ж�����
  // NVIC_DisableIRQ(EXTI9_5_IRQn); // �ر�NVIC�ж�ͨ�� 
}












