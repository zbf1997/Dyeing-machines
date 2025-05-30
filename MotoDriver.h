#ifndef __MOTODRIVER_H
#define __MOTODRIVER_H
#include "can.h" 
void positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absoluteAxis);
void setWorkMode(uint8_t slaveAddr,uint16_t Ma);
void speedModeRun(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc);
boolean_t waitingForACK(void);
void speedModeStop(uint8_t slaveAddr);
void positionMode1Stop(uint8_t slaveAddr);
void positionMode3Stop(uint8_t slaveAddr);
void P1Moto_Init();
void Set_GoHome(u8 slaveAddr,u8 homeTrig,u8 homeDir,u16 homeSpeed,u8 EndLimit);
void Go_Home(u8 slaveAddr);



/**********PMC007 canopen**********/
#define QUEUE_SIZE 10
typedef enum{
	DeviceType = 0x1000,
	ErrorRegister = 0x1001,
	ManufacturerStatusRegister = 0x1002,
	PreDefinedErrorField = 0x1003,
	COBIDSYNCMessage = 0x1005,
	CommunicationCyclePeriod = 0x1006,
	SynchronousWindowLength = 0x1007,
	ManufacturerDeviceName = 0x1008,
	ManufacturerHardwareVersion = 0x1009,
	ManufacturerSoftwareVersion = 0x100A,
	COBIDEmergencyMessage = 0x1014,
	InhibitTimeEMCY = 0x1015,
	ConsumerHeartbeatTime = 0x1016,
	ProducerHeartbeatTime = 0x1017,
	IdentityObject = 0x1018,
	ServerSDOParameter = 0x1200,
	ClientSDOParameter = 0x1280,
	ReceivePDO0CommunicationParameter = 0x1400,
	ReceivePDO1CommunicationParameter = 0x1401,
	ReceivePDO2CommunicationParameter = 0x1402,
	ReceivePDO3CommunicationParameter = 0x1403,
	ReceivePDO0mappingParameter = 0x1600,
	ReceivePDO1mappingParameter = 0x1601,
	ReceivePDO2mappingParameter = 0x1602,
	ReceivePDO3mappingParameter = 0x1603,
	TransmitPDO0CommunicationParameters = 0x1800,
	TransmitPDO1CommunicationParameters = 0x1801,
	TransmitPDO2CommunicationParameters = 0x1802,
	TransmitPDO3CommunicationParameters = 0x1803,
	TransmitPDO0MappingParameter = 0x1A00,
	TransmitPDO1MappingParameter = 0x1A01,
	TransmitPDO2MappingParameter = 0x1A02,
	TransmitPDO3MappingParameter = 0x1A03,
	CANNodeID = 0x2002,
	BaudRate = 0x2003,
	GroupID = 0x2006,
	SystemControl = 0x2007,
	MotorStatus = 0x6000,
	ControllerStatus = 0x6001,
	RotationDirection = 0x6002,
	MaxSpeed = 0x6003,
	StepRelativePosition = 0x6004,
	OperationMode = 0x6005,
	StartSpeed = 0x6006,
	EndSpeed = 0x6007,
	AccelerationCoefficient = 0x6008,
	DecelerationCoefficient = 0x6009,
	MicroStepping   = 0x600A,
	MaxPhaseCurrent = 0x600B,
	MotorPosition = 0x600C,
	CurrentDecay = 0x600D,
	MotorEnable = 0x600E,
	ExternalEmergencyStop = 0x600F,
	PVTStep = 0x6010,
	GPIOParameter = 0x6011,
	GPIOValue = 0x6012,
	OCPParameter = 0x6013,
	BrakeControl = 0x6016,
	StallParameter = 0x6017,
	OfflineParameter1 = 0x6018,
	OfflineParameter2 = 0x6019,
	JitterDelayOfExternalEmergencyStop = 0x601A,
	LockedRotorConfiguration = 0x601B,
	StepAbsolutePosition = 0x601C,
	PVStep = 0x601D,
	AbortStep = 0x6020,
	EncoderCPR = 0x6021,
	PowerDownStorageLocation = 0x6022,
	Kp = 0x6023,
	Ki = 0x6024,
	Kd = 0x6025,
	PreFilterParameters = 0x6026,
	PostFilterParameters = 0x6027,
	LockedRotorLength = 0x6028,
	TorqueLoopEnable = 0x6029,
	SaveAfterPowerFailure = 0x602A,
	AnalogueInput = 0x602B,
	StepPositionNotification = 0x602C,
	PP_PVModeParameter1 = 0x602D,
	PP_PVModeParameter2 = 0x602E,
	AnalogPositioningControlParameters = 0x602F,
	RealTimeSpeed = 0x6030,
	PowerDownBehaviorControl = 0x6031,
	CalixationZeroPosition = 0x6034,
	EncoderPositon = 0x6035
}CXCAN_ADDRESS;

typedef struct {
    uint32_t id;       
    uint8_t data[8];   
} CAN_Message;


typedef struct {
    CAN_Message message;  
} QueueNode;


typedef struct {
    QueueNode buffer[QUEUE_SIZE];  
    int head;                     
    int tail;                     
    int size;                     
} CAN_Queue;


void CAN_Queue_Init(CAN_Queue *queue);
uint8_t CAN_Queue_Enqueue(CAN_Queue *queue, CAN_Message *message);
uint8_t CAN_Queue_Dequeue(CAN_Queue *queue, CAN_Message *message);







void AbsMove_1(int X,int Y,int Z);
void AbsMove_2(int X,int Y,int Z);
void MotoDriver1234_Init(int Acceleration,int Deceleration,int StartSpeed,int EndSpeed);
void CXCAN_SetMicrostepping(uint8_t NodeID, int value);
void CXCAN_SetMaxPhaseCurrent(uint8_t NodeID, int value);
void CXCAN_SetOperationMode(uint8_t NodeID, int value);
void CXCAN_SetRotationDirection(uint8_t NodeID, int value);
void CXCAN_Set_PPRunningSpeed(uint8_t NodeID, int value);
void CXCAN_Set_PPTargetLocation(uint8_t NodeID, int value);
void CXCAN_Set_PPControlWord(uint8_t NodeID, int value);
void StartOncePPRun(uint8_t NodeID,int RunningSpeed,int TargetLocation);
void CXCAN_Set_Position_Deceleration(uint8_t NodeID, int value);
void CXCAN_Set_Position_Acceleration(uint8_t NodeID, int value);
void CXCAN_Set_AbsolutePosition(uint8_t NodeID, int value);
void CXCAN_Set_Position_MaxSpeed(uint8_t NodeID, int value);
void CXCAN_Set_StartSpeed(uint8_t NodeID, int value);
void CXCAN_Set_EndSpeed(uint8_t NodeID, int value);
void CANOpen_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
void CXCAN_Transmit(uint8_t NodeID, uint8_t CanOpearType, uint16_t Address, uint8_t SubAddress, uint32_t value);
void StartOnceSpeedRun(uint8_t NodeID,int RunningSpeed,int TargetLocation);
void StopPositionSpeed(uint8_t NodeID);
u8 IsControllerBusy(uint8_t NodeID);

#endif
