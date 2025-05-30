#include "stm32f4xx.h" 
#include <string.h>
#include "timer.h" 
#include "can.h" 
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "MotoDriver.h"

uint8_t txBuffer[8];      //��������������
CAN_Queue controllerStatusQueue;

/*
���ܣ����û�����ز���,������ɺ���Ҫִ��һ�λ��㶯��
���룺slaveAddr   �ӻ���ַ
	  homeTrig	  ��λ���رպ�ʱ����Ч��ƽ,0�͵�ƽ 1�ߵ�ƽ
	  homeDir	  ��λ���㷽�� 0˳ʱ�� 1��ʱ��
	  homeSpeed   ��λ�����ٶ�(0-3000RPM)
	  EndLimit    ��λʹ�� 
�������
 */
void Set_GoHome(u8 slaveAddr,u8 homeTrig,u8 homeDir,u16 homeSpeed,u8 EndLimit)
{
	CAN_ID = slaveAddr;				
	txBuffer[0] = 0x90;      
	txBuffer[1] = homeTrig;			
	txBuffer[2] = homeDir;					
	txBuffer[3] = homeSpeed;			
	txBuffer[4] = EndLimit;
	CanTransfer(txBuffer,6);
}

/*
���ܣ�ִ�л��㶯��
 */
void Go_Home(u8 slaveAddr)
{
	CAN_ID = slaveAddr;				
	txBuffer[0] = 0x91;      
	CanTransfer(txBuffer,2);
}



/*
���ܣ����ù�������
���룺slaveAddr �ӻ���ַ
			Ma ��������
�������
 */
void setWorkMode(uint8_t slaveAddr,uint16_t Ma)
{
	CAN_ID = slaveAddr;				//ID
	txBuffer[0] = 0x83;       //������
	txBuffer[1] = Ma>>8;			//������8λ
	txBuffer[2] = Ma;					//������8λ
	CanTransfer(txBuffer,4);
}


/*
���ܣ�can�����ٶ�ģʽ����ָ��
���룺slaveAddr �ӻ���ַ
      dir       ���з���
      speed     �����ٶ�
      acc       ���ٶ�
*/
void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF6;       //������
  txBuffer[1] = (dir<<7) | ((speed>>8)&0x0F); //������ٶȸ�4λ
  txBuffer[2] = speed&0x00FF;   //�ٶȵ�8λ
  txBuffer[3] = acc;            //���ٶ�
	CanTransfer(txBuffer,5);
}

/*
���ܣ�can�����ٶ�ģʽָֹͣ��
���룺slaveAddr �ӻ���ַ
      acc       ���ٶ�
*/
void speedModeStop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF6;       //������
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //���ٶ�200
  CanTransfer(txBuffer,5);
}

/*
���ܣ����ڷ���λ��ģʽ1����ָ��
���룺slaveAddr �ӻ���ַ
      dir       ���з���
      speed     �����ٶ�
      acc       ���ٶ�
      pulses    ������
*/
void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xFD;       //������
  txBuffer[1] = (dir<<7) | ((speed>>8)&0x0F); //������ٶȸ�4λ
  txBuffer[2] = speed&0x00FF;   //�ٶȵ�8λ
  txBuffer[3] = acc;            //���ٶ�
  txBuffer[4] = (pulses >> 16)&0xFF;  //������ bit23 - bit16   
  txBuffer[5] = (pulses >> 8)&0xFF;   //������ bit15 - bit8
  txBuffer[6] = (pulses >> 0)&0xFF;   //������ bit7 - bit0
	
	CanTransfer(txBuffer,8);
}         


/*
���ܣ�can����λ��ģʽ1ָֹͣ��
���룺slaveAddr �ӻ���ַ
      acc       ���ٶ�
*/
void positionMode1Stop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xFD;       //������
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //���ٶ�20
  txBuffer[4] = 0x00;       //������
  txBuffer[5] = 0x00; 
  txBuffer[6] = 0x00;   
  CanTransfer(txBuffer,8);
}


/*
���ܣ����ڷ���λ��ģʽ3����ָ��
���룺slaveAddr �ӻ���ַ
      speed     �����ٶ�
      acc       ���ٶ�
      absAxis   ��������
*/
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF5;       //������
  txBuffer[1] = (speed>>8)&0x00FF; //�ٶȸ�8λ
  txBuffer[2] = speed&0x00FF;     //�ٶȵ�8λ
  txBuffer[3] = acc;            //���ٶ�
  txBuffer[4] = (absAxis >> 16)&0xFF;  //�������� bit23 - bit16
  txBuffer[5] = (absAxis >> 8)&0xFF;   //�������� bit15 - bit8
  txBuffer[6] = (absAxis >> 0)&0xFF;   //�������� bit7 - bit0
	
	CanTransfer(txBuffer,8);
}

/*
���ܣ�can����λ��ģʽ3ָֹͣ��
���룺slaveAddr �ӻ���ַ
      acc       ���ٶ�
*/
void positionMode3Stop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF4;       //������
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //���ٶ�20
  txBuffer[4] = 0x00;       //������
  txBuffer[5] = 0x00; 
  txBuffer[6] = 0x00;   
  CanTransfer(txBuffer,8);
}


/*
���ܣ��ȴ��ӻ�Ӧ�����ó�ʱʱ��Ϊ5000ms
���룺 ��
�����
  ���гɹ�    TRUE
  ����ʧ��    FALSE
  ��ʱ��Ӧ��  FALSE
*/
boolean_t waitingForACK(void)
{
  boolean_t retVal = FALSE; //����ֵ
  unsigned long sTime;  		//��ʱ��ʼʱ��
  unsigned long time;  			//��ǰʱ��
  uint8_t rxByte;      
	
  sTime = millis();    //��ȡ��ǰʱ��
  while(1)
  {
	if(CAN_RxDone == TRUE)  //CAN���յ�����
	{
		CAN_RxDone = FALSE;
		rxByte = CanRxBuf.DLC;
		CAN_ID = CanRxBuf.StdId;
		if(CanRxBuf.Data[rxByte-1] == canCRC_ATM(CanRxBuf.Data,rxByte-1))
		{
			retVal = TRUE;   //У����ȷ
			break;
		}				
			
	}

    time = millis();
    if((time - sTime) > 5000)   //�ж��Ƿ�ʱ
    {
      retVal = FALSE;
      break;                    //��ʱ���˳�while(1)
    }
  }
  return(retVal);
}




void P1Moto_Init()
{
	 Set_GoHome(1,0,0,100,0);
	 Go_Home(1);
}



/*******************PMC007 CANopen��������******************************/
void CXCAN_SetMicrostepping(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, MicroStepping, 0x00, value_32);
}
void CXCAN_SetMaxPhaseCurrent(uint8_t NodeID, int value) 
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, MaxPhaseCurrent, 0x00, value_32);
}

void CXCAN_SetOperationMode(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE1, OperationMode, 0x00, value_32);
}

void CXCAN_SetRotationDirection(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE1, RotationDirection, 0x00, value_32);
}

void CXCAN_Set_PPRunningSpeed(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE4, PP_PVModeParameter2, 0x03, value_32);
}

void CXCAN_Set_PPTargetLocation(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE4, PP_PVModeParameter2, 0x04, value_32);
}

void CXCAN_Set_PPControlWord(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, PP_PVModeParameter2, 0x01, value_32);
}

void CXCAN_Set_PP_PVAcceleration(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, PP_PVModeParameter1, 0x01, value_32);
}

void CXCAN_Set_PP_PVDeceleration(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, PP_PVModeParameter1, 0x02, value_32);
}

void CXCAN_Set_Position_Acceleration(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE1, AccelerationCoefficient, 0x00, value_32);
}
void CXCAN_Set_Position_Deceleration(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE1, DecelerationCoefficient, 0x00, value_32);
}

void CXCAN_Set_Position_MaxSpeed(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE4, MaxSpeed, 0x00, value_32);//MaxSpeedȡֵ��(S32)-200000~200000
}

void CXCAN_Set_AbsolutePosition(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE4, StepAbsolutePosition, 0x00, value_32);
}
void CXCAN_Set_StartSpeed(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, StartSpeed, 0x00, value_32);
}
void CXCAN_Set_EndSpeed(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE2, EndSpeed, 0x00, value_32);
}
void CXCAN_Set_RelativePosition(uint8_t NodeID, int value)
{
	uint32_t value_32 = (uint32_t)value;
	CANOpen_Transfer_Write(NodeID, WRITE4, StepRelativePosition, 0x00, value_32);
}



void CANOpen_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
	CanTxMsg TxMessage;
	u8 statusOk = 0;
	uint8_t TransmitMailbox;
	uint8_t res;
	uint8_t i;
	TxMessage.StdId = ID;
	TxMessage.ExtId = ID;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = Length;
	for (i = 0; i < Length; i ++)
	{
		TxMessage.Data[i] = Data[i];
	}
	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	while(!statusOk)
	{
		res = CAN_TransmitStatus(CAN1, 0x00);
		statusOk = res == CAN_TxStatus_Ok;	
	}
}

void CXCAN_Transmit(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value)
{
	uint8_t Data[8] = {0x00};
	Data[0] = CanOpearType;
	Data[1] = Address & 0xFF;
	Data[2] = Address >> 8;
	Data[3] = SubAddress;
	Data[4] = value & 0xff;
	Data[5] = ( value >> 8) & 0xff;
	Data[6] = ( value >> 16) & 0xff;
	Data[7] = value >> 24;
	CANOpen_Transmit(0x600 + NodeID, 8, Data);
}

void CAN_Queue_Init(CAN_Queue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0;
}

uint8_t CAN_Queue_Enqueue(CAN_Queue *queue, CAN_Message *message) {
    if (queue->size == QUEUE_SIZE) {
        queue->head = (queue->head + 1) % QUEUE_SIZE;
				queue->size--;
    }

    queue->buffer[queue->tail].message = *message;

    queue->tail = (queue->tail + 1) % QUEUE_SIZE;

    queue->size++;

    return 1;
}

uint8_t CAN_Queue_Dequeue(CAN_Queue *queue, CAN_Message *message) {
    if (queue->size == 0) {
        return 0;
    }

    *message = queue->buffer[queue->head].message;

    queue->head = (queue->head + 1) % QUEUE_SIZE;

    queue->size--;

    return 1;
}


uint8_t IsControllerBusy(uint8_t NodeID) 
{
	uint32_t value_32 = 0x00;
	uint8_t controllerStatus;
	uint8_t bit;
	int i = 0;
	uint8_t result = 0x00;
	CAN_Message message;
	CXCAN_Transmit(NodeID, READ, ControllerStatus, 0x00, value_32);
	while(1)
	{
		if(CAN_Queue_Dequeue(&controllerStatusQueue, &message))
		{
			controllerStatus = message.data[4];
			bit = (controllerStatus & (1 << 3)) >> 3;
			result =  bit == 0x00;
			break;
		}
		if(i == 10) 
		{
			break;
		}
		delay_ms(2);
	}
	return result;
}







void MotoDriver1234_Init(int Acceleration,int Deceleration,int StartSpeed,int MaxSpeed)//��ʼ��NodeID�ֱ�Ϊ1234��������
{
	u8 NodeID=0;
	for(NodeID=1;NodeID<5;NodeID++)
	{
		CXCAN_SetMicrostepping(NodeID, 8);//ϸ����
		delay_ms(2);
		CXCAN_SetMaxPhaseCurrent(NodeID, 1600);//������
		delay_ms(2);
		CXCAN_SetRotationDirection(NodeID, 1); //ת������
		CXCAN_Set_Position_Acceleration(NodeID, Acceleration);//���ٶ�
		delay_ms(2);
		CXCAN_Set_Position_Deceleration(NodeID, Deceleration);//���ٶ�
		delay_ms(2);
		CXCAN_Set_StartSpeed(NodeID, StartSpeed);//�����ٶ�
		delay_ms(2);
	}
}

void AbsMove_1(int X,int Y,int Z)
{
	StartOncePPRun(1,4000,X);
	vTaskDelay(20/portTICK_RATE_MS);
	StartOncePPRun(2,38000,Y);
	vTaskDelay(20/portTICK_RATE_MS);
	StartOncePPRun(3,38000,Z);
}

void AbsMove_2(int X,int Y,int Z)
{
	StartOncePPRun(4,4000,X);
	vTaskDelay(20/portTICK_RATE_MS);
	StartOncePPRun(5,15000,Y);
	vTaskDelay(20/portTICK_RATE_MS);
	StartOncePPRun(6,7500,Z);
}

void StartOncePPRun(uint8_t NodeID,int RunningSpeed,int TargetLocation)
{
	CXCAN_Set_Position_MaxSpeed(NodeID, RunningSpeed);//��������ٶ�
	vTaskDelay(2/portTICK_RATE_MS);
	CXCAN_Set_AbsolutePosition(NodeID, TargetLocation);//��������λ�ò���
	vTaskDelay(2/portTICK_RATE_MS);
	//CXCAN_Set_PPControlWord(NodeID, 50);//�����֣�30�����ģʽ�������У�50�Ծ���ģʽ��������
}

void StartOnceSpeedRun(uint8_t NodeID,int RunningSpeed,int TargetLocation)//�ٶ�ģʽ
{
	CXCAN_Set_Position_MaxSpeed(NodeID, RunningSpeed);//��������ٶ�
	vTaskDelay(2/portTICK_RATE_MS);
	CXCAN_SetOperationMode(NodeID, 1);//�ٶ�ģʽ
	vTaskDelay(2/portTICK_RATE_MS);
}

void StopPositionSpeed(uint8_t NodeID)
{
	CANOpen_Transfer_Write(NodeID, WRITE1, AbortStep, 0x00, 0);
}



//���1������λ��ģʽϸ����16���Ӽ��ٶ�8�������ٶ�4000��λ���ܹ�5500
//���2������ϸ����16���Ӽ��ٶ�8�������ٶ�38000��λ���ܹ�60000
//���3������ϸ����16���Ӽ��ٶ�8�������ٶ�38000��λ���ܹ�45000��25000Ϊ������������İ�ȫ����
//���4������ϸ����16���Ӽ��ٶ�8�������ٶ�6000��λ���ܹ�15500������2000
//���5������ϸ����16���Ӽ��ٶ�8�������ٶ�15000��λ���ܹ�12000,����1000
//���6������ϸ����16���Ӽ��ٶ�8�������ٶ�7500��λ���ܹ�7000