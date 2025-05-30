#ifndef __AT24Cxx_H
#define __AT24Cxx_H

#include "stm32f4xx.h"    // STM32F4标准库
#include "FreeRTOS.h"     // FreeRTOS头文件
#include "semphr.h"       // 信号量操作

/* 型号选择（取消注释其中一个） */
// #define AT24C01       // 128个Byte，16页，每页8字节
// #define AT24C02       // 256，32页，每页8字节
// #define AT24C04       // 512，32页，每页16字节
// #define AT24C08       // 1024，64页，每页16字节
// #define AT24C16       // 2048，128页，每页16字节
// #define AT24C32       // 4096，128页，每页32字节
// #define AT24C64       // 8192，256页，每页32字节
// #define AT24C128       // 16384，256页，每页64字节
 #define AT24C256       // 32768，512页，每页64字节
// #define AT24C512       // 65536，512页，每页128字节


/* 根据型号自动配置参数 */
#ifdef AT24C01
#define AT24CXX_PAGE_SIZE     8     // 页大小（字节）
#define AT24CXX_PAGE_COUNT    16    // 总页数
#define AT24CXX_ADDR_LEN      1     // 地址长度（1字节）
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

#define AT24CXX_TOTAL_SIZE    (AT24CXX_PAGE_SIZE * AT24CXX_PAGE_COUNT) // 总容量
#define AT24CXX_I2C_ADDR      0xA0  // 设备地址（A2/A1/A0接地）
#define I2C_TIMEOUT           1000  // 超时阈值（循环次数）

/* GPIO引脚定义 */
#define I2C_SCL_PORT          GPIOB 
#define I2C_SCL_PIN           GPIO_Pin_6  
#define I2C_SDA_PORT          GPIOB 
#define I2C_SDA_PIN           GPIO_Pin_7  

extern SemaphoreHandle_t xI2CMutex;  // FreeRTOS互斥锁

void AT24CXX_Init(void);
void EEPROM_Verify();
uint8_t AT24CXX_Write(uint16_t addr, uint8_t *data, uint16_t len);
uint8_t AT24CXX_Read(uint16_t addr, uint8_t *buffer, uint16_t len);

#endif
