#include "stm32f4xx.h" 
#include <string.h>
#include "timer.h" 
#include "can.h" 
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "MotoDriver.h"

uint8_t txBuffer[8];      //待发送数据数组
CAN_Queue controllerStatusQueue;

/*
功能：设置回零相关参数,设置完成后需要执行一次回零动作
输入：slaveAddr   从机地址
	  homeTrig	  限位开关闭合时的有效电平,0低电平 1高电平
	  homeDir	  限位归零方向 0顺时针 1逆时针
	  homeSpeed   限位归零速度(0-3000RPM)
	  EndLimit    限位使能 
输出：无
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
功能：执行回零动作
 */
void Go_Home(u8 slaveAddr)
{
	CAN_ID = slaveAddr;				
	txBuffer[0] = 0x91;      
	CanTransfer(txBuffer,2);
}



/*
功能：设置工作电流
输入：slaveAddr 从机地址
			Ma 工作电流
输出：无
 */
void setWorkMode(uint8_t slaveAddr,uint16_t Ma)
{
	CAN_ID = slaveAddr;				//ID
	txBuffer[0] = 0x83;       //功能码
	txBuffer[1] = Ma>>8;			//电流高8位
	txBuffer[2] = Ma;					//电流低8位
	CanTransfer(txBuffer,4);
}


/*
功能：can发送速度模式运行指令
输入：slaveAddr 从机地址
      dir       运行方向
      speed     运行速度
      acc       加速度
*/
void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF6;       //功能码
  txBuffer[1] = (dir<<7) | ((speed>>8)&0x0F); //方向和速度高4位
  txBuffer[2] = speed&0x00FF;   //速度低8位
  txBuffer[3] = acc;            //加速度
	CanTransfer(txBuffer,5);
}

/*
功能：can发送速度模式停止指令
输入：slaveAddr 从机地址
      acc       加速度
*/
void speedModeStop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF6;       //功能码
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //加速度200
  CanTransfer(txBuffer,5);
}

/*
功能：串口发送位置模式1运行指令
输入：slaveAddr 从机地址
      dir       运行方向
      speed     运行速度
      acc       加速度
      pulses    脉冲数
*/
void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xFD;       //功能码
  txBuffer[1] = (dir<<7) | ((speed>>8)&0x0F); //方向和速度高4位
  txBuffer[2] = speed&0x00FF;   //速度低8位
  txBuffer[3] = acc;            //加速度
  txBuffer[4] = (pulses >> 16)&0xFF;  //脉冲数 bit23 - bit16   
  txBuffer[5] = (pulses >> 8)&0xFF;   //脉冲数 bit15 - bit8
  txBuffer[6] = (pulses >> 0)&0xFF;   //脉冲数 bit7 - bit0
	
	CanTransfer(txBuffer,8);
}         


/*
功能：can发送位置模式1停止指令
输入：slaveAddr 从机地址
      acc       加速度
*/
void positionMode1Stop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xFD;       //功能码
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //加速度20
  txBuffer[4] = 0x00;       //功能码
  txBuffer[5] = 0x00; 
  txBuffer[6] = 0x00;   
  CanTransfer(txBuffer,8);
}


/*
功能：串口发送位置模式3运行指令
输入：slaveAddr 从机地址
      speed     运行速度
      acc       加速度
      absAxis   绝对坐标
*/
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis)
{
	CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF5;       //功能码
  txBuffer[1] = (speed>>8)&0x00FF; //速度高8位
  txBuffer[2] = speed&0x00FF;     //速度低8位
  txBuffer[3] = acc;            //加速度
  txBuffer[4] = (absAxis >> 16)&0xFF;  //绝对坐标 bit23 - bit16
  txBuffer[5] = (absAxis >> 8)&0xFF;   //绝对坐标 bit15 - bit8
  txBuffer[6] = (absAxis >> 0)&0xFF;   //绝对坐标 bit7 - bit0
	
	CanTransfer(txBuffer,8);
}

/*
功能：can发送位置模式3停止指令
输入：slaveAddr 从机地址
      acc       加速度
*/
void positionMode3Stop(uint8_t slaveAddr)
{
  CAN_ID = slaveAddr;				//ID
  txBuffer[0] = 0xF4;       //功能码
  txBuffer[1] = 0x00; 
  txBuffer[2] = 0x00;   
  txBuffer[3] = 0xc8;        //加速度20
  txBuffer[4] = 0x00;       //功能码
  txBuffer[5] = 0x00; 
  txBuffer[6] = 0x00;   
  CanTransfer(txBuffer,8);
}


/*
功能：等待从机应答，设置超时时间为5000ms
输入： 无
输出：
  运行成功    TRUE
  运行失败    FALSE
  超时无应答  FALSE
*/
boolean_t waitingForACK(void)
{
  boolean_t retVal = FALSE; //返回值
  unsigned long sTime;  		//计时起始时刻
  unsigned long time;  			//当前时刻
  uint8_t rxByte;      
	
  sTime = millis();    //获取当前时刻
  while(1)
  {
	if(CAN_RxDone == TRUE)  //CAN接收到数据
	{
		CAN_RxDone = FALSE;
		rxByte = CanRxBuf.DLC;
		CAN_ID = CanRxBuf.StdId;
		if(CanRxBuf.Data[rxByte-1] == canCRC_ATM(CanRxBuf.Data,rxByte-1))
		{
			retVal = TRUE;   //校验正确
			break;
		}				
			
	}

    time = millis();
    if((time - sTime) > 5000)   //判断是否超时
    {
      retVal = FALSE;
      break;                    //超时，退出while(1)
    }
  }
  return(retVal);
}




void P1Moto_Init()
{
	 Set_GoHome(1,0,0,100,0);
	 Go_Home(1);
}



/*******************PMC007 CANopen驱动函数******************************/
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
	CANOpen_Transfer_Write(NodeID, WRITE4, MaxSpeed, 0x00, value_32);//MaxSpeed取值：(S32)-200000~200000
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







void MotoDriver1234_Init(int Acceleration,int Deceleration,int StartSpeed,int MaxSpeed)//初始化NodeID分别为1234的驱动器
{
	u8 NodeID=0;
	for(NodeID=1;NodeID<5;NodeID++)
	{
		CXCAN_SetMicrostepping(NodeID, 8);//细分数
		delay_ms(2);
		CXCAN_SetMaxPhaseCurrent(NodeID, 1600);//最大电流
		delay_ms(2);
		CXCAN_SetRotationDirection(NodeID, 1); //转动方向
		CXCAN_Set_Position_Acceleration(NodeID, Acceleration);//加速度
		delay_ms(2);
		CXCAN_Set_Position_Deceleration(NodeID, Deceleration);//减速度
		delay_ms(2);
		CXCAN_Set_StartSpeed(NodeID, StartSpeed);//启动速度
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
	CXCAN_Set_Position_MaxSpeed(NodeID, RunningSpeed);//最大运行速度
	vTaskDelay(2/portTICK_RATE_MS);
	CXCAN_Set_AbsolutePosition(NodeID, TargetLocation);//启动绝对位置步进
	vTaskDelay(2/portTICK_RATE_MS);
	//CXCAN_Set_PPControlWord(NodeID, 50);//控制字，30以相对模式立即运行，50以绝对模式立即运行
}

void StartOnceSpeedRun(uint8_t NodeID,int RunningSpeed,int TargetLocation)//速度模式
{
	CXCAN_Set_Position_MaxSpeed(NodeID, RunningSpeed);//最大运行速度
	vTaskDelay(2/portTICK_RATE_MS);
	CXCAN_SetOperationMode(NodeID, 1);//速度模式
	vTaskDelay(2/portTICK_RATE_MS);
}

void StopPositionSpeed(uint8_t NodeID)
{
	CANOpen_Transfer_Write(NodeID, WRITE1, AbortStep, 0x00, 0);
}



//电机1参数：位置模式细分数16，加减速度8，运行速度4000，位置总共5500
//电机2参数：细分数16，加减速度8，运行速度38000，位置总共60000
//电机3参数：细分数16，加减速度8，运行速度38000，位置总共45000，25000为不触碰到缸体的安全距离
//电机4参数：细分数16，加减速度8，运行速度6000，位置总共15500，步进2000
//电机5参数：细分数16，加减速度8，运行速度15000，位置总共12000,步进1000
//电机6参数：细分数16，加减速度8，运行速度7500，位置总共7000