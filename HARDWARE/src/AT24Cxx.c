#include "AT24Cxx.h"
#include "task.h"       // FreeRTOS任务API
#include "delay.h"
#include <string.h>

/* GPIO电平操作宏 */
#define SCL_HIGH()  GPIO_SetBits(I2C_SCL_PORT, I2C_SCL_PIN)    // SCL置高
#define SCL_LOW()   GPIO_ResetBits(I2C_SCL_PORT, I2C_SCL_PIN)  // SCL置低
#define SDA_HIGH()  GPIO_SetBits(I2C_SDA_PORT, I2C_SDA_PIN)    // SDA置高
#define SDA_LOW()   GPIO_ResetBits(I2C_SDA_PORT, I2C_SDA_PIN)  // SDA置低
#define SDA_READ()  GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN) // 读SDA状态


// FreeRTOS互斥锁定义
SemaphoreHandle_t xI2CMutex;

/* 初始化函数（配置GPIO和互斥锁） */
void AT24CXX_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 初始化GPIO */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   // 使能GPIOB时钟
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;   
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;              // 输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;             // 开漏输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;               // 内部上拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

    /* 初始状态释放总线 */
    SCL_HIGH();
    SDA_HIGH();

    /* 创建互斥锁（保证I2C总线独占访问） */
    xI2CMutex = xSemaphoreCreateMutex();  
}

/* 生成I2C起始信号 */
static uint8_t I2C_Start(void) {
    uint32_t timeout = I2C_TIMEOUT;
    SDA_HIGH();
    SCL_HIGH();
    delay_us(8);
    while (!SDA_READ() && timeout--) {    // 检测总线是否被占用
        if (timeout == 0) return 1;      // 超时返回错误
    }
    SDA_LOW();                           // SDA下降沿
    delay_us(8);
    SCL_LOW();                           // SCL拉低，准备发送数据
    return 0;
}

/* 生成I2C停止信号 */
static void I2C_Stop(void) {
    SDA_LOW();                           // SDA拉低
    delay_us(8);
    SCL_HIGH();                          // SCL拉高
    delay_us(8);
    SDA_HIGH();                          // SDA上升沿，形成停止信号
    delay_us(8);
}

/* 发送一个字节并检测ACK */
static uint8_t I2C_SendByte(uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    for (uint8_t i = 0; i < 8; i++) {    // 发送8位数据
        SCL_LOW();                       // 确保SCL低电平期间改变SDA
        if (data & 0x80) SDA_HIGH();      // 发送最高位
        else SDA_LOW();
        data <<= 1;                      // 左移准备下一位
        delay_us(5);
        SCL_HIGH();                      // SCL上升沿，数据被采样
        delay_us(5);
        SCL_LOW();                       // 准备下一位数据
    }

    /* 检测ACK */
    SDA_HIGH();                          // 释放SDA线
    SCL_HIGH();                          // SCL上升沿
    delay_us(3);
    while (SDA_READ() && timeout--) {    // 等待从机拉低SDA（ACK）
        if (timeout == 0) return 1;      // 超时无应答
    }
    SCL_LOW();                           // SCL拉低，结束ACK检测
    return 0;
}

/* 读取一个字节并发送ACK/NACK */
static uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t data = 0;
    SDA_HIGH();                          // 释放SDA线，准备读取
    for (uint8_t i = 0; i < 8; i++) {    // 读取8位数据
        SCL_LOW();                       
        delay_us(5);
        SCL_HIGH();                      // SCL上升沿，数据稳定
        data <<= 1;                      // 左移接收下一位
        if (SDA_READ()) data |= 0x01;    // 读取当前位
        delay_us(5);
    }
    SCL_LOW();                           
    if (ack) SDA_LOW();                  // 发送ACK（拉低SDA）
    else SDA_HIGH();                     // 发送NACK（保持高电平）
    delay_us(8);
    SCL_HIGH();                          // SCL上升沿，ACK/NACK被采样
    delay_us(8);
    SCL_LOW();                           
    SDA_HIGH();                          // 释放SDA线
    return data;
}

/* 写入数据（自动分页处理） */
uint8_t AT24CXX_Write(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t status = 0;
    if (addr + len > AT24CXX_TOTAL_SIZE) return 4; // 地址越界

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 获取互斥锁
        uint16_t bytes_remaining = len;
        while (bytes_remaining > 0) {
            uint16_t page_offset = addr % AT24CXX_PAGE_SIZE;      // 计算当前页内偏移
            uint16_t write_size = AT24CXX_PAGE_SIZE - page_offset; // 当前页剩余空间
            if (write_size > bytes_remaining) write_size = bytes_remaining;

            // 发送起始信号、设备地址和内存地址
            if (I2C_Start() ||                                   // 起始信号
                I2C_SendByte(AT24CXX_I2C_ADDR) ||                // 发送设备地址（写模式）
                (AT24CXX_ADDR_LEN == 2 && I2C_SendByte(addr >> 8)) || // 高地址字节（仅2字节模式）
                I2C_SendByte(addr & 0xFF)) {                     // 低地址字节
                status = 1; // 总线错误
                break;
            }

            // 写入数据
            for (uint16_t i = 0; i < write_size; i++) {          
                if (I2C_SendByte(data[i])) {                     // 发送数据字节
                    status = 2; // 数据发送失败
                    break;
                }
            }
            I2C_Stop();                                          // 停止信号
            vTaskDelay(pdMS_TO_TICKS(5));                        // 等待EEPROM内部写入完成
            addr += write_size;                                  // 更新地址
            data += write_size;                                  // 更新数据指针
            bytes_remaining -= write_size;                       // 更新剩余字节
        }
        xSemaphoreGive(xI2CMutex);                               // 释放互斥锁
    } else {
        status = 3; // 获取互斥锁失败
    }
    return status;
}

/* 读取数据 */
uint8_t AT24CXX_Read(uint16_t addr, uint8_t *buffer, uint16_t len) {
    uint8_t status = 0;
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 获取互斥锁
        if (I2C_Start() ||                                        // 起始信号
            I2C_SendByte(AT24CXX_I2C_ADDR) ||                     // 设备地址（写模式）
            (AT24CXX_ADDR_LEN == 2 && I2C_SendByte(addr >> 8)) || // 高地址字节（仅2字节模式）
            I2C_SendByte(addr & 0xFF)) {                          // 低地址字节
            status = 1; // 地址发送失败
        } else {
            I2C_Start();                                          // 重复起始信号
            I2C_SendByte(AT24CXX_I2C_ADDR | 0x01);                // 设备地址（读模式）
            for (uint16_t i = 0; i < len; i++) {
                buffer[i] = I2C_ReadByte(i < (len - 1));          // 读取字节并发送ACK/NACK
            }
        }
        I2C_Stop();                                               // 停止信号
        xSemaphoreGive(xI2CMutex);                                // 释放互斥锁
    } else {
        status = 2; // 获取互斥锁失败
    }
    return status;
}

void EEPROM_Verify() 
{
    uint8_t data[32] = {0x01, 0x02, 0x03}; // 测试数据
    uint8_t buffer[32];
    uint16_t addr = 0;

    // 写入32字节（可能跨页）
    uint8_t write_status = AT24CXX_Write(addr, data, sizeof(data));
    if (write_status == 0) 
	{
     // 读取验证
        uint8_t read_status = AT24CXX_Read(addr, buffer, sizeof(buffer));
        if (read_status == 0 && memcmp(data, buffer, sizeof(data)) == 0) 
		{
			printf("EEPROM数据验证成功");  
        }
    }
}