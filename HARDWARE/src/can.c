#include <string.h>
#include "can.h"
#include "stdbool.h"
CanRxMsg CanRxBuf;
CanTxMsg CanTxBuf;
uint16_t  CAN_ID;
boolean_t CAN_RxDone = FALSE;

uint8_t canCRC_ATM(uint8_t *buf,uint8_t len);
/**
	* @brief   ��ʼ��CAN 
	* @param   ��
	* @retval  ��
	*/
void CAN_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef	CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*----- ʹ��CAN ����ʱ��-----*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/*----- ��ʼ��CAN����-----*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1

	/*----- ��ʼ��CAN -----*/
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;					// �ر�ʱ�䴥��ͨѶģʽ
	CAN_InitStructure.CAN_ABOM = DISABLE;					// �����Զ����߹���
	CAN_InitStructure.CAN_AWUM = DISABLE;					// �ر��Զ�����ģʽ
	CAN_InitStructure.CAN_NART = DISABLE;					// �رշ��Զ��ش�ģʽ	DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM = DISABLE;					// ����FIFO����ģʽ		DISABLE-���ʱ�±��ĻḲ��ԭ�б���
	CAN_InitStructure.CAN_TXFP = DISABLE;					// ����FIFO���ȼ�			DISABLE-���ȼ�ȡ���ڱ��ı�ʶ��
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	// ��������ģʽ
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;			// ����ͬ����Ծ���ΪSJW + 1��ʱ�䵥λ
	CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 21;					// ������ = 42M / 21 / (1 + 13 + 2) = 0.125����125K
	CAN_Init(CAN1, &CAN_InitStructure);

// ��ʼ��CAN������
	CAN_FilterInitStructure.CAN_FilterNumber = 1;											// ������1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// ����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 32λ������λ��
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x00;					// ��������ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x00;					// ��������ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_STD_ID_H_MASK_DONT_CARE;						// ���������α�ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_STD_ID_L_MASK_DONT_CARE;						// ���������α�ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;			// ָ���������FIFOΪ0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	
	// ��ʼ��CAN�ж�
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);													// ʹ��RX0һ�����ݽ����ж�
}

/**
	* @brief   CAN1_RX0�����ж�
	* @param   ��
	* @retval  ��
	*/
void CAN1_RX0_IRQHandler(void)
{
	// ����һ֡����
	CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&CanRxBuf));
	// ��λ���ձ�־
	CAN_RxDone = TRUE;
}

//CAN������׼֡
void CanTransfer(uint8_t *buf,uint8_t len)
{
    /* Init Transmit frame*/
    CanTxBuf.StdId   = CAN_ID; 					
    CanTxBuf.IDE     = CAN_ID_STD;  
    CanTxBuf.RTR     = CAN_RTR_DATA;    
    CanTxBuf.DLC     = len;  
		
	memcpy(CanTxBuf.Data,buf,len);//��buf��ָ����ڴ��ַ�ĵ�һ�����ݵ���len�����ݸ��Ƶ�CanTxBuf.Data��CanTxBuf.Data��ԭ��ΪCanTxMsg.Data[8]	
	CanTxBuf.Data[len-1] = canCRC_ATM(buf,len-1);
	
	CAN_Transmit(CAN1, &CanTxBuf);

}


//����У���
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




/*******************CANopen��������******************************/
//canopen������׼֡ 
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
	memcpy(CanTxBuf.Data,buf,8);//��buf��ָ����ڴ��ַ�ĵ�һ�����ݵ���8�����ݸ��Ƶ�CanTxBuf.Data��CanTxBuf.Data��ԭ��ΪCanTxMsg.Data[8]
	statusOk = FALSE;
	CAN_Transmit(CAN1, &CanTxBuf);
	while(!statusOk)
	{
		if((CAN_TransmitStatus(CAN1, 0x00)==CAN_TxStatus_Ok) |(CAN_TransmitStatus(CAN1, 0x01)==CAN_TxStatus_Ok)|(CAN_TransmitStatus(CAN1, 0x02)==CAN_TxStatus_Ok))//can��������������ֻҪ��һ���ǿ�
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
	memcpy(CanTxBuf.Data,buf,8);//��buf��ָ����ڴ��ַ�ĵ�һ�����ݵ���8�����ݸ��Ƶ�CanTxBuf.Data��CanTxBuf.Data��ԭ��ΪCanTxMsg.Data[8]	
	CAN_Transmit(CAN1, &CanTxBuf);
}