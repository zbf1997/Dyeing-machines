#ifndef __BSP_USARTX_FIFO_H__
#define __BSP_USARTX_FIFO_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include <stm32f4xx.h>
#include "stdio.h"
#include "string.h"
/* 类型定义 ------------------------------------------------------------------*/
/* 定义端口号 */
typedef enum
{
	COM1 = 0,	/* USART1 */
	COM2 = 1,	/* USART2 */
	COM3 = 2,	/* USART3 */
}COM_PORT_E;

/* 串口设备结构体 */
typedef struct
{
	USART_TypeDef *usart;              /* STM32内部串口设备指针 */
	uint8_t *pTxBuf;                   /* 发送缓冲区 */
	uint8_t *pRxBuf;                   /* 接收缓冲区 */
	uint16_t usTxBufSize;              /* 发送缓冲区大小 */
	uint16_t usRxBufSize;              /* 接收缓冲区大小 */
	__IO uint16_t usTxWrite;           /* 发送缓冲区写指针 */
	__IO uint16_t usTxRead;            /* 发送缓冲区读指针 */
	__IO uint16_t usTxCount;           /* 等待发送的数据个数 */

	__IO uint16_t usRxWrite;           /* 接收缓冲区写指针 */
	__IO uint16_t usRxRead;            /* 接收缓冲区读指针 */
	__IO uint16_t usRxCount;           /* 还未读取的新数据个数 */

	void (*SendBefor)(void);           /* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
	void (*SendOver)(void);            /* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
	void (*ReciveNew)(uint8_t _byte);  /* 串口收到数据的回调函数指针 */
}USART_FIFO_Typedef;


/* 宏定义 --------------------------------------------------------------------*/
#define	USART1_FIFO_EN                     	  1
#define	USART2_FIFO_EN                     	  1
#define	USART3_FIFO_EN                     	  1

#define DISABLE_INT()                         { __set_PRIMASK(1); }	/* 关中断 */
#define ENABLE_INT()                          { __set_PRIMASK(0); }	/* 开中断 */

#define AIBUS_CH1_ADDR         1
#define AIBUS_CH2_ADDR         2
#define AIBUS_FN_TEMP          0       //宇电温控器AIBUS协议的参数代号   温度值
#define AIBUS_COMD_WRITE       67  
#define AIBUS_COMD_READ        82


#define REG_READTEMP_CH1         349  
#define REG_READTEMP_CH2         350

#define REG_WRITETEMP_CH1        0  
#define REG_WRITETEMP_CH2        1


/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if USART1_FIFO_EN == 0
	#define USART1_BAUD			                    9600
	#define USART1_TX_BUF_SIZE	                1*1024
	#define USART1_RX_BUF_SIZE	                1*1024
#endif

#if USART2_FIFO_EN == 1
	#define USART2_BAUD			                    115200
	#define USART2_TX_BUF_SIZE	                1*1024
	#define USART2_RX_BUF_SIZE	                1*1024

	/* RS485芯片发送使能GPIO, PG8 */
	#define RS485_TXEN_RCC_CLOCKGPIO 	      RCC_AHB1Periph_GPIOG
	#define RS485_TXEN_GPIO                   GPIOG
	#define RS485_TXEN_GPIO_PIN	              GPIO_Pin_8

	#define RS485_RX_EN()	                      RS485_TXEN_GPIO->BSRRH= RS485_TXEN_GPIO_PIN
	#define RS485_TX_EN()	                      RS485_TXEN_GPIO->BSRRL = RS485_TXEN_GPIO_PIN
#endif

#if USART3_FIFO_EN == 0
	#define USART3_BAUD			                    9600
	#define USART3_TX_BUF_SIZE	                1*1024
	#define USART3_RX_BUF_SIZE	                1*1024
  
#endif

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void Usart_FIFO_Init(void);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);

void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);

void bsp_SetUsart1Baud(uint32_t _baud);
void bsp_SetUsart2Baud(uint32_t _baud);
void bsp_SetUsart3Baud(uint32_t _baud);

void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);


unsigned char Check_Recieve_Data( unsigned char *buf );






#endif 
