#include "stm32f4xx.h"
#include "InputSlicDetect.h"
#include "Globalvariable.h"

void EXTI9_5_IRQHandler(void)

{
  // if (EXTI_GetITStatus(EXTI_Line5) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line5);
  //   SlicSensor1count++;
  //   printf("进入外部中断线5,SlicSensor1count=%d\r\n",SlicSensor1count);
  // }

  // if (EXTI_GetITStatus(EXTI_Line6) != RESET)
  // {
  //   EXTI_ClearITPendingBit(EXTI_Line6);
  //   SlicSensor2count++;
  //   printf("进入外部中断线6,SlicSensor2count=%d\r\n",SlicSensor2count);
  // }

  if (EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line7);
    SlicSensor3count++;
    printf("进入外部中断线7,SlicSensor3count=%d\r\n",SlicSensor3count);
  }
}

//外部中断初始化程序
void InputSlicDetect_Init(void)
{
	// NVIC_InitTypeDef  NVIC_InitStructure;
	// EXTI_InitTypeDef  EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
 
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);//PA5 连接到中断线5
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//PA6 连接到中断线6
	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);//PA7 连接到中断线7

	/*配置GPIO口*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  // /* 配置EXTI_Line*/
  // EXTI_InitStructure.EXTI_Line = EXTI_Line5|EXTI_Line6|EXTI_Line7;
  // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发 
  // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  // EXTI_Init(&EXTI_InitStructure);
	
	// NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x07;//抢占优先级7
  // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure);//配置

  // /*关闭中断，等用户信号才开启外部中断*/
  // EXTI->IMR &= ~EXTI_Line5;   // 关闭EXTI5中断屏蔽
  // EXTI->IMR &= ~EXTI_Line6;   // 关闭EXTI6中断屏蔽
  // EXTI->IMR &= ~EXTI_Line7;   // 关闭EXTI7中断屏蔽
  // NVIC_DisableIRQ(EXTI9_5_IRQn); // 关闭NVIC中断通道 
}












