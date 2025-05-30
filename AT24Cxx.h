#ifndef __AT24Cxx_H
#define __AT24Cxx_H

#include "stm32f4xx.h"    // STM32F4��׼��
#include "FreeRTOS.h"     // FreeRTOSͷ�ļ�
#include "semphr.h"       // �ź�������

/* �ͺ�ѡ��ȡ��ע������һ���� */
// #define AT24C01       // 128��Byte��16ҳ��ÿҳ8�ֽ�
// #define AT24C02       // 256��32ҳ��ÿҳ8�ֽ�
// #define AT24C04       // 512��32ҳ��ÿҳ16�ֽ�
// #define AT24C08       // 1024��64ҳ��ÿҳ16�ֽ�
// #define AT24C16       // 2048��128ҳ��ÿҳ16�ֽ�
// #define AT24C32       // 4096��128ҳ��ÿҳ32�ֽ�
// #define AT24C64       // 8192��256ҳ��ÿҳ32�ֽ�
// #define AT24C128       // 16384��256ҳ��ÿҳ64�ֽ�
 #define AT24C256       // 32768��512ҳ��ÿҳ64�ֽ�
// #define AT24C512       // 65536��512ҳ��ÿҳ128�ֽ�


/* �����ͺ��Զ����ò��� */
#ifdef AT24C01
#define AT24CXX_PAGE_SIZE     8     // ҳ��С���ֽڣ�
#define AT24CXX_PAGE_COUNT    16    // ��ҳ��
#define AT24CXX_ADDR_LEN      1     // ��ַ���ȣ�1�ֽڣ�
#elif defined(AT24C02)
#define AT24CXX_PAGE_SIZE     8
#define AT24CXX_PAGE_COUNT    32
#define AT24CXX_ADDR_LEN      1
#elif defined(AT24C04)
#define AT24CXX_PAGE_SIZE     16
#define AT24CXX_PAGE_COUNT    32
#define AT24CXX_ADDR_LEN      1
#elif defined(AT24C08)
#define AT24CXX_PAGE_SIZE     16
#define AT24CXX_PAGE_COUNT    64
#define AT24CXX_ADDR_LEN      1
#elif defined(AT24C16)
#define AT24CXX_PAGE_SIZE     16
#define AT24CXX_PAGE_COUNT    128
#define AT24CXX_ADDR_LEN      1
#elif defined(AT24C32)
#define AT24CXX_PAGE_SIZE     32
#define AT24CXX_PAGE_COUNT    128
#define AT24CXX_ADDR_LEN      2     
#elif defined(AT24C64)
#define AT24CXX_PAGE_SIZE     32
#define AT24CXX_PAGE_COUNT    256
#define AT24CXX_ADDR_LEN      2 
#elif defined(AT24C128)
#define AT24CXX_PAGE_SIZE     64
#define AT24CXX_PAGE_COUNT    256
#define AT24CXX_ADDR_LEN      2 
#elif defined(AT24C256)
#define AT24CXX_PAGE_SIZE     64
#define AT24CXX_PAGE_COUNT    512
#define AT24CXX_ADDR_LEN      2 
#elif defined(AT24C512)
#define AT24CXX_PAGE_SIZE     128
#define AT24CXX_PAGE_COUNT    512
#define AT24CXX_ADDR_LEN      2 
#endif

#define AT24CXX_TOTAL_SIZE    (AT24CXX_PAGE_SIZE * AT24CXX_PAGE_COUNT) // ������
#define AT24CXX_I2C_ADDR      0xA0  // �豸��ַ��A2/A1/A0�ӵأ�
#define I2C_TIMEOUT           1000  // ��ʱ��ֵ��ѭ��������

/* GPIO���Ŷ��� */
#define I2C_SCL_PORT          GPIOB 
#define I2C_SCL_PIN           GPIO_Pin_6  
#define I2C_SDA_PORT          GPIOB 
#define I2C_SDA_PIN           GPIO_Pin_7  

extern SemaphoreHandle_t xI2CMutex;  // FreeRTOS������

void AT24CXX_Init(void);
void EEPROM_Verify();
uint8_t AT24CXX_Write(uint16_t addr, uint8_t *data, uint16_t len);
uint8_t AT24CXX_Read(uint16_t addr, uint8_t *buffer, uint16_t len);

#endif
