#ifndef __MAX31865_H
#define __MAX31865_H


#include "stm32f4xx.h"
#define REF_RES 430
#define MAX31685_RDY() GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define MAX31685_CS_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define MAX31685_CS_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_12)

void SPI_MAX31865_Init(void);
void MAX31865Confg(void);
uint8_t MAX31865_SB_Read(uint8_t addr);
void MAX31865_SB_Write(uint8_t addr,uint8_t wdata);
float Read_MAX31865_Temperature(void);

#endif
