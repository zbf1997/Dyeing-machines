#include "AT24Cxx.h"
#include "task.h"       // FreeRTOS����API
#include "delay.h"
#include <string.h>

/* GPIO��ƽ������ */
#define SCL_HIGH()  GPIO_SetBits(I2C_SCL_PORT, I2C_SCL_PIN)    // SCL�ø�
#define SCL_LOW()   GPIO_ResetBits(I2C_SCL_PORT, I2C_SCL_PIN)  // SCL�õ�
#define SDA_HIGH()  GPIO_SetBits(I2C_SDA_PORT, I2C_SDA_PIN)    // SDA�ø�
#define SDA_LOW()   GPIO_ResetBits(I2C_SDA_PORT, I2C_SDA_PIN)  // SDA�õ�
#define SDA_READ()  GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN) // ��SDA״̬


// FreeRTOS����������
SemaphoreHandle_t xI2CMutex;

/* ��ʼ������������GPIO�ͻ������� */
void AT24CXX_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ��ʼ��GPIO */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   // ʹ��GPIOBʱ��
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;   
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;              // ���ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;             // ��©���
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;               // �ڲ�����
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

    /* ��ʼ״̬�ͷ����� */
    SCL_HIGH();
    SDA_HIGH();

    /* ��������������֤I2C���߶�ռ���ʣ� */
    xI2CMutex = xSemaphoreCreateMutex();  
}

/* ����I2C��ʼ�ź� */
static uint8_t I2C_Start(void) {
    uint32_t timeout = I2C_TIMEOUT;
    SDA_HIGH();
    SCL_HIGH();
    delay_us(8);
    while (!SDA_READ() && timeout--) {    // ��������Ƿ�ռ��
        if (timeout == 0) return 1;      // ��ʱ���ش���
    }
    SDA_LOW();                           // SDA�½���
    delay_us(8);
    SCL_LOW();                           // SCL���ͣ�׼����������
    return 0;
}

/* ����I2Cֹͣ�ź� */
static void I2C_Stop(void) {
    SDA_LOW();                           // SDA����
    delay_us(8);
    SCL_HIGH();                          // SCL����
    delay_us(8);
    SDA_HIGH();                          // SDA�����أ��γ�ֹͣ�ź�
    delay_us(8);
}

/* ����һ���ֽڲ����ACK */
static uint8_t I2C_SendByte(uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;
    for (uint8_t i = 0; i < 8; i++) {    // ����8λ����
        SCL_LOW();                       // ȷ��SCL�͵�ƽ�ڼ�ı�SDA
        if (data & 0x80) SDA_HIGH();      // �������λ
        else SDA_LOW();
        data <<= 1;                      // ����׼����һλ
        delay_us(5);
        SCL_HIGH();                      // SCL�����أ����ݱ�����
        delay_us(5);
        SCL_LOW();                       // ׼����һλ����
    }

    /* ���ACK */
    SDA_HIGH();                          // �ͷ�SDA��
    SCL_HIGH();                          // SCL������
    delay_us(3);
    while (SDA_READ() && timeout--) {    // �ȴ��ӻ�����SDA��ACK��
        if (timeout == 0) return 1;      // ��ʱ��Ӧ��
    }
    SCL_LOW();                           // SCL���ͣ�����ACK���
    return 0;
}

/* ��ȡһ���ֽڲ�����ACK/NACK */
static uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t data = 0;
    SDA_HIGH();                          // �ͷ�SDA�ߣ�׼����ȡ
    for (uint8_t i = 0; i < 8; i++) {    // ��ȡ8λ����
        SCL_LOW();                       
        delay_us(5);
        SCL_HIGH();                      // SCL�����أ������ȶ�
        data <<= 1;                      // ���ƽ�����һλ
        if (SDA_READ()) data |= 0x01;    // ��ȡ��ǰλ
        delay_us(5);
    }
    SCL_LOW();                           
    if (ack) SDA_LOW();                  // ����ACK������SDA��
    else SDA_HIGH();                     // ����NACK�����ָߵ�ƽ��
    delay_us(8);
    SCL_HIGH();                          // SCL�����أ�ACK/NACK������
    delay_us(8);
    SCL_LOW();                           
    SDA_HIGH();                          // �ͷ�SDA��
    return data;
}

/* д�����ݣ��Զ���ҳ���� */
uint8_t AT24CXX_Write(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t status = 0;
    if (addr + len > AT24CXX_TOTAL_SIZE) return 4; // ��ַԽ��

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // ��ȡ������
        uint16_t bytes_remaining = len;
        while (bytes_remaining > 0) {
            uint16_t page_offset = addr % AT24CXX_PAGE_SIZE;      // ���㵱ǰҳ��ƫ��
            uint16_t write_size = AT24CXX_PAGE_SIZE - page_offset; // ��ǰҳʣ��ռ�
            if (write_size > bytes_remaining) write_size = bytes_remaining;

            // ������ʼ�źš��豸��ַ���ڴ��ַ
            if (I2C_Start() ||                                   // ��ʼ�ź�
                I2C_SendByte(AT24CXX_I2C_ADDR) ||                // �����豸��ַ��дģʽ��
                (AT24CXX_ADDR_LEN == 2 && I2C_SendByte(addr >> 8)) || // �ߵ�ַ�ֽڣ���2�ֽ�ģʽ��
                I2C_SendByte(addr & 0xFF)) {                     // �͵�ַ�ֽ�
                status = 1; // ���ߴ���
                break;
            }

            // д������
            for (uint16_t i = 0; i < write_size; i++) {          
                if (I2C_SendByte(data[i])) {                     // ���������ֽ�
                    status = 2; // ���ݷ���ʧ��
                    break;
                }
            }
            I2C_Stop();                                          // ֹͣ�ź�
            vTaskDelay(pdMS_TO_TICKS(5));                        // �ȴ�EEPROM�ڲ�д�����
            addr += write_size;                                  // ���µ�ַ
            data += write_size;                                  // ��������ָ��
            bytes_remaining -= write_size;                       // ����ʣ���ֽ�
        }
        xSemaphoreGive(xI2CMutex);                               // �ͷŻ�����
    } else {
        status = 3; // ��ȡ������ʧ��
    }
    return status;
}

/* ��ȡ���� */
uint8_t AT24CXX_Read(uint16_t addr, uint8_t *buffer, uint16_t len) {
    uint8_t status = 0;
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // ��ȡ������
        if (I2C_Start() ||                                        // ��ʼ�ź�
            I2C_SendByte(AT24CXX_I2C_ADDR) ||                     // �豸��ַ��дģʽ��
            (AT24CXX_ADDR_LEN == 2 && I2C_SendByte(addr >> 8)) || // �ߵ�ַ�ֽڣ���2�ֽ�ģʽ��
            I2C_SendByte(addr & 0xFF)) {                          // �͵�ַ�ֽ�
            status = 1; // ��ַ����ʧ��
        } else {
            I2C_Start();                                          // �ظ���ʼ�ź�
            I2C_SendByte(AT24CXX_I2C_ADDR | 0x01);                // �豸��ַ����ģʽ��
            for (uint16_t i = 0; i < len; i++) {
                buffer[i] = I2C_ReadByte(i < (len - 1));          // ��ȡ�ֽڲ�����ACK/NACK
            }
        }
        I2C_Stop();                                               // ֹͣ�ź�
        xSemaphoreGive(xI2CMutex);                                // �ͷŻ�����
    } else {
        status = 2; // ��ȡ������ʧ��
    }
    return status;
}

void EEPROM_Verify() 
{
    uint8_t data[32] = {0x01, 0x02, 0x03}; // ��������
    uint8_t buffer[32];
    uint16_t addr = 0;

    // д��32�ֽڣ����ܿ�ҳ��
    uint8_t write_status = AT24CXX_Write(addr, data, sizeof(data));
    if (write_status == 0) 
	{
     // ��ȡ��֤
        uint8_t read_status = AT24CXX_Read(addr, buffer, sizeof(buffer));
        if (read_status == 0 && memcmp(data, buffer, sizeof(data)) == 0) 
		{
			printf("EEPROM������֤�ɹ�");  
        }
    }
}