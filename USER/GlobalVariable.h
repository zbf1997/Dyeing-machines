#ifndef __GlobalVariable_H
#define __GlobalVariable_H
#include "FreeRTOS.h"
#include "task.h"
#include "hmi_driver.h"
#include "semphr.h"
#define GET_BYTE0(x) (((x) >>  0) & 0xff) /* 获取第0个字节 */
#define GET_BYTE1(x) (((x) >>  8) & 0xff) /* 获取第1个字节 */
#define GET_BYTE2(x) (((x) >> 16) & 0xff) /* 获取第2个字节 */
#define GET_BYTE3(x) (((x) >> 24) & 0xff) /* 获取第3个字节 */
#define GET_2BYTE_L(x) (((x) >> 0) & 0xffff) /* 获取低16位 */
#define GET_2BYTE_H(x) (((x) >> 16) & 0xffff) /* 获取高16位 */
#define Moto_OverTime 20000 //电机等待超时时间ms
#define MotoX1 0
#define MotoY1 1
#define MotoZ1 2
#define MotoA1 3
#define MotoX2 4
#define MotoY2 5
#define MotoZ2 6
#define MotoA2 7
#define MotoX3 8
#define MotoY3 9
#define MotoZ3 10
#define MotoA3 11

typedef enum {
      RelMode   = 1,//相对模式
      AbsMode   = 2,//绝对模式
      ResetMode = 3,//复位模式
      StopMode  = 4,//停止模式
      HYMode    = 7,//永动模式
      AgingMode = 8 //老化模式
  } MotorMode;

// 电机参数结构定义
typedef struct 
{
      int32_t MaxStep;        //最大步数
      int32_t SingleStep;     //单次运行步数
      uint16_t Acc;           //加速度
      uint16_t Dec;           //减速度
      uint16_t Speed;         //运行速度
      uint16_t Current;       //电机电流
      MotorMode RunMode;      //运行模式
      uint8_t Status;         //电机状态
      uint16_t ResetOffest;   //电机复位偏移量
      u8 MotoModBuf[30];      //存储modbus传输数据,字节0-3存储最大行程，字节4-7运行步数，字节8-9加速度，字节10-11减速度，字节12-13运行速度，字节14-15电机电流，字节16运行模式,字节18-19复位偏移值
} MotorParams;


//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		800  
//任务句柄
extern TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define Process_message_PRIO		5
//任务堆栈大小	
#define Process_message_SIZE 		200  
//任务句柄
extern TaskHandle_t Process_message_Handler;
//任务函数
void Process_message(void *pvParameters);

//任务优先级
#define moto_TASK_PRIO		4
//任务堆栈大小	
#define moto_STK_SIZE 		500  
//任务句柄
extern TaskHandle_t moto_Task_Handler;
//任务函数
void moto_task(void *pvParameters);


//任务优先级
#define LED0_TASK_PRIO		2
//任务堆栈大小	
#define LED0_STK_SIZE 		50  
//任务句柄
extern TaskHandle_t LED0Task_Handler;
//任务函数
void led0_task(void *pvParameters);

//任务优先级
#define MotoStausCheck_TASK_PRIO		2
//任务堆栈大小	
#define MotoStausCheck_STK_SIZE 		800  
//任务句柄
extern TaskHandle_t MotoStausCheckTask_Handler;
//任务函数
void MotoStausCheck(void *pvParameters);

//任务优先级
#define InputOutSlic_TASK_PRIO		2
//任务堆栈大小	
#define InputOutSlic_STK_SIZE 		200  
//任务句柄
extern TaskHandle_t InputOutSlic_Handler;
//任务函数
void InputOutSlic(void *pvParameters);

//任务优先级
#define MonitorTasks_TASK_PRIO		3
//任务堆栈大小	
#define MonitorTasks_STK_SIZE 		200  
//任务句柄
extern TaskHandle_t MonitorTasks_Handler;
//任务函数
void MonitorTasks(void *pvParameters);

//任务优先级
#define shiyanlicuheng1_TASK_PRIO		4
//任务堆栈大小	
#define shiyanlicuheng1_STK_SIZE 		500  
//任务句柄
extern TaskHandle_t shiyanlicuheng1_Handler;
//任务函数
void shiyanlicuheng1(void *pvParameters);

//任务优先级
#define TempControl_TASK_PRIO		2
//任务堆栈大小	
#define TempControl_STK_SIZE 		200
//任务句柄
extern TaskHandle_t TempControl_Handler;
//任务函数
void TempControl(void *pvParameters);

//任务优先级
#define ValvePump_TASK_PRIO		2
//任务堆栈大小	
#define ValvePump_STK_SIZE 		200
//任务句柄
extern TaskHandle_t ValvePump_Handler;
//任务函数
void ValvePump(void *pvParameters);

extern volatile uint16 ucg_Temp_ch1,MotoStatus[10];
extern uint8  cmd_buffer[CMD_MAX_SIZE]; 
extern volatile int32 MotoLocation[10];
extern MotorParams AxisMotors[12];

extern u8 ucg_ValveModBuf[45][10];
extern u8 ucg_X1MotoRun1Btn,ucg_Y1MotoRun1Btn,ucg_Z1MotoRun1Btn,ucg_A1MotoRunBtn,ucg_X1Y1Z1RunBtn,ucg_X1Y1Z1RstBtn,ucg_X1Y1Z1StopBtn,
   ucg_X2MotoRun1Btn,ucg_Y2MotoRun1Btn,ucg_Z2MotoRun1Btn,ucg_A2MotoRunBtn,ucg_X2Y2Z2RunBtn,ucg_X2Y2Z2RstBtn,ucg_X2Y2Z2StopBtn,
   ucg_X3MotoRunBtn,ucg_Y3MotoRunBtn,ucg_Z3MotoRunBtn,ucg_A3MotoRunBtn,ucg_X3Y3Z3A3RunBtn,ucg_X3Y3Z3A3RstBtn,ucg_X3Y3Z3A3StopBtn,
   ucg_X1MotoRun2Btn,ucg_Y1MotoRun2Btn,ucg_Z1MotoRun2Btn,ucg_X2MotoRun2Btn,ucg_Y2MotoRun2Btn,ucg_Z2MotoRun2Btn,
   ucg_X1RstBtn,ucg_Y1RstBtn,ucg_Z1RstBtn,ucg_X2RstBtn,ucg_Y2RstBtn,ucg_Z2RstBtn,
   ucg_X1StopBtn,ucg_Y1StopBtn,ucg_Z1StopBtn,ucg_X2StopBtn,ucg_Y2StopBtn,ucg_Z2StopBtn,
   ucg_X1SaveBtn,ucg_Y1SaveBtn,ucg_Z1SaveBtn,ucg_X2SaveBtn,ucg_Y2SaveBtn,ucg_Z2SaveBtn,
   ucg_BasketCapRunBtn;

extern uint8_t ucg_ValveSwitch[45],ucg_ValveDir[45],ucg_ValveRunBtn,ucg_ValveStopBtn,ucg_ValveReverseBtn;

extern u32 uwg_NewUsart2Baund,uwg_OldUsart2Baund;
extern u8 ucg_NewUsart2SlaveAddress,ucg_OldUsart2SlaveAddress,ucg_BaundSlaveAddressSetBtn;

extern u16   uhwg_X1ResetOffest,uhwg_Y1ResetOffest,uhwg_Z1ResetOffest,uhwg_A1ResetOffest,
      uhwg_X2ResetOffest,uhwg_Y2ResetOffest,uhwg_Z2ResetOffest,uhwg_A2ResetOffest,
      uhwg_X3ResetOffest,uhwg_Y3ResetOffest,uhwg_Z3ResetOffest,uhwg_A3ResetOffest;
extern u8 ucg_MotionPosition_flag[10];
extern u8 ucg_UncapPosition_flag;
extern u16 ZstepShakeinterval,Z1ShakeSpeed,Z1ShakeAcc,Z1ShakeDec,Z1Shaketime;

extern int16 uhwg_MotionPosition_Offest[33][3];
extern u32 uhwg_MotionPosition_Initial[33][3];
extern u32 uhwg_MotionPosition_Compose[33][3];
extern int16 uhwg_UncapPosition_Offest[33][3];
extern u32 uhwg_UncapPosition_Initial[33][3];
extern u32 uhwg_UncapPosition_Compose[33][3];
extern u16 uhw_liucheng1ReactTime[30];
extern int16 uhwg_SetTemp[4];
extern u8 ucg_SetTempBuf[8];
extern int16 uhwg_RealTemp[4];
extern u8 ucg_SetTempFlag[4];
extern u16 uhwg_SetTempPID[16];
extern u8 ucg_SetTempPIDBuf[32];
extern u8 ucg_SendPidFlag;

extern SemaphoreHandle_t MODH_CmdMutex;
extern SemaphoreHandle_t UiCmdBianry;
extern SemaphoreHandle_t MotoMonitBianry;
extern SemaphoreHandle_t InputOutSlicBianry,TempControlBianry,shiyanlicuheng1Bianry,ValvePumpBianry;
extern TaskHandle_t MODH_CmdMutexOwnership;
extern u8 StainingPodStatus[50];
extern u8 GetTakeSampleDir;
extern u8 StainingNumTest;
extern u8 SlaveaddressSwitchTest;
extern volatile u8 MotoTaskFlag;//任务挂起标志位,0表示任务继续，1表示任务挂起，2表示任务结束
extern u8 SlicSensor1count,SlicSensor2count,SlicSensor3count,SlicSensorCount,InputOutSlicFlag,TempControlFlag,shiyanlicuheng1Flag;

extern u16 shiyan1Param[40][2];
extern u8 kaopian[10];//kaopian[0]表示是否需要烤片，kaopian[1]表示烤片时间
extern u8 ActionTime1;
extern u8 ActionTime2;
extern u8 ActionTime3;

#endif