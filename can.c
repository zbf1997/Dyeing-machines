#include <string.h>
#include "can.h"
#include "stdbool.h"
CanRxMsg CanRxBuf;
CanTxMsg CanTxBuf;
uint16_t  CAN_ID;
boolean_t CAN_RxDone = FALSE;

uint8_t canCRC_ATM(uint8_t *buf,uint8_t len);
/**
	* @brief   初始化CAN 
	* @param   无
	* @retval  无
	*/
void CAN_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef	CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*----- 使能CAN 外设时钟-----*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/*----- 初始化CAN引脚-----*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1

	/*----- 初始化CAN -----*/
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;					// 关闭时间触发通讯模式
	CAN_InitStructure.CAN_ABOM = DISABLE;					// 开启自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;					// 关闭自动唤醒模式
	CAN_InitStructure.CAN_NART = DISABLE;					// 关闭非自动重传模式	DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM = DISABLE;					// 接收FIFO锁定模式		DISABLE-溢出时新报文会覆盖原有报文
	CAN_InitStructure.CAN_TXFP = DISABLE;					// 发送FIFO优先级			DISABLE-优先级取决于报文标识符
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	// 正常工作模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;			// 重新同步跳跃宽度为SJW + 1个时间单位
	CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 21;					// 波特率 = 42M / 21 / (1 + 13 + 2) = 0.125，即125K
	CAN_Init(CAN1, &CAN_InitStructure);

// 初始化CAN过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = 1;											// 过滤器1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 掩码模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 32位过滤器位宽
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x00;					// 过滤器标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x00;					// 过滤器标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_STD_ID_H_MASK_DONT_CARE;						// 过滤器屏蔽标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_STD_ID_L_MASK_DONT_CARE;						// 过滤器屏蔽标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;			// 指向过滤器的FIFO为0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// 使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	
	// 初始化CAN中断
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);													// 使能RX0一包数据接收中断
}

/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN1_RX0_IRQHandler(void)
{
	// 接收一帧数据
	CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&CanRxBuf));
	// 置位接收标志
	CAN_RxDone = TRUE;
}

//CAN发出标准帧
void CanTransfer(uint8_t *buf,uint8_t len)
{
    /* Init Transmit frame*/
    CanTxBuf.StdId   = CAN_ID; 					
    CanTxBuf.IDE     = CAN_ID_STD;  
    CanTxBuf.RTR     = CAN_RTR_DATA;    
    CanTxBuf.DLC     = len;  
		
	memcpy(CanTxBuf.Data,buf,len);//将buf所指向的内存地址的第一个数据到第len个数据复制到CanTxBuf.Data，CanTxBuf.Data的原型为CanTxMsg.Data[8]	
	CanTxBuf.Data[len-1] = canCRC_ATM(buf,len-1);
	
	CAN_Transmit(CAN1, &CanTxBuf);

}


//计算校验和
uint8_t canCRC_ATM(uint8_t *buf,uint8_t len) //CRC_SUM8
{
	uint32_t i;
	uint8_t check_sum;
	uint32_t sum = 0;
	
	for(i=0;i<len;i++)
	{
		sum += buf[i];
	}
	sum += CAN_ID;
	check_sum = sum & 0xFF;
	return check_sum;
}




/*******************CANopen驱动函数******************************/
//canopen发出标准帧 
void CANOpen_Transfer_Write(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value)
{
    /* Init Transmit frame*/
	uint8_t buf[8] = {0x00};
	uint8_t TransmitMailbox;
	bool statusOk;
    CanTxBuf.StdId   = 0x600+NodeID; 					
    CanTxBuf.IDE     = CAN_ID_STD;  
    CanTxBuf.RTR     = CAN_RTR_DATA;    
    CanTxBuf.DLC     = 8;
	buf[0] = CanOpearType;
	buf[1] = Address & 0xFF;
	buf[2] = Address >> 8;
	buf[3] = SubAddress;
	buf[4] = value & 0xff;
	buf[5] = ( value >> 8) & 0xff;
	buf[6] = ( value >> 16) & 0xff;
	buf[7] = value >> 24;
	memcpy(CanTxBuf.Data,buf,8);//将buf所指向的内存地址的第一个数据到第8个数据复制到CanTxBuf.Data，CanTxBuf.Data的原型为CanTxMsg.Data[8]
	statusOk = FALSE;
	CAN_Transmit(CAN1, &CanTxBuf);
	while(!statusOk)
	{
		if((CAN_TransmitStatus(CAN1, 0x00)==CAN_TxStatus_Ok) |(CAN_TransmitStatus(CAN1, 0x01)==CAN_TxStatus_Ok)|(CAN_TransmitStatus(CAN1, 0x02)==CAN_TxStatus_Ok))//can的三个发送邮箱只要有一个是空
		statusOk = TRUE;	
	}
}

void CANOpen_Transfer_Read(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value)
{
    /* Init Transmit frame*/
	uint8_t buf[8] = {0x00};
    CanTxBuf.StdId   = 0x580+NodeID; 					
    CanTxBuf.IDE     = CAN_ID_STD;  
    CanTxBuf.RTR     = CAN_RTR_DATA;    
    CanTxBuf.DLC     = 8;
	
	buf[0] = CanOpearType;
	buf[1] = Address & 0xFF;
	buf[2] = Address >> 8;
	buf[3] = SubAddress;
	buf[4] = value & 0xff;
	buf[5] = ( value >> 8) & 0xff;
	buf[6] = ( value >> 16) & 0xff;
	buf[7] = value >> 24;
	memcpy(CanTxBuf.Data,buf,8);//将buf所指向的内存地址的第一个数据到第8个数据复制到CanTxBuf.Data，CanTxBuf.Data的原型为CanTxMsg.Data[8]	
	CAN_Transmit(CAN1, &CanTxBuf);
}