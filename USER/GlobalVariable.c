#include "GlobalVariable.h"
#include "semphr.h"
#include "FreeRTOS.h"

//任务句柄
TaskHandle_t StartTask_Handler,Process_message_Handler,moto_Task_Handler;
TaskHandle_t LED0Task_Handler,MotoStausCheckTask_Handler,MonitorTasks_Handler;
TaskHandle_t InputOutSlic_Handler, shiyanlicuheng1_Handler,TempControl_Handler;
TaskHandle_t ValvePump_Handler;

// 电机参数数组
MotorParams AxisMotors[12] = {
//MaxStep|SingleStep|Acc|Dec |Speed|Current|Mode|Status|Offset
   {110000,  5000, 3000, 3000,  800,   35,  2,  0,  250},  // X1，雷赛驱动器电流单位为0.1A
   { 35000,  5000, 1000, 1000,  300,   35,  2,  0,  220},  // Y1
   {  3500,  5000, 1000, 1000,  120,   16,  2,  0,   20},  // Z1
   {330000,  2000, 1000, 1000, 1000,    5,  2,  0,  500},  // A1
   { 95000,  5000, 2000, 2000,  800,   35,  2,  0,  400},  // X2
   {  8500,  1000, 1000, 1000,  100,    5,  2,  0,  250},  // Y2
   {125000,  1000,  500,  500, 6000,    5,  2,  0,   50},  // Z2
   {  2000,  2000, 2000, 2000,   60,    8,  2,  0,    0},  // A2
   { 32000, 28800,  200,  200,   80, 1500,  7,  0,    0},  // X3，奥创控制板电流单位为mA
   {200000, 20000, 1000, 1000,   95,  600,  7,  0,    0},  // Y3
   {200000, 20000, 1000, 1000,   95,  600,  7,  0,    0},  // Z3
   {200000, 20000, 1000, 1000,   95,  600,  7,  0,    0}   // A3
};

volatile uint16 ucg_Temp_ch1,MotoStatus[10];
uint8  cmd_buffer[CMD_MAX_SIZE];
volatile int32 MotoLocation[10];

u8 ucg_ValveModBuf[45][10];
u8 ucg_X1MotoRun1Btn,ucg_Y1MotoRun1Btn,ucg_Z1MotoRun1Btn,ucg_A1MotoRunBtn,ucg_X1Y1Z1RunBtn,ucg_X1Y1Z1RstBtn,ucg_X1Y1Z1StopBtn,
   ucg_X2MotoRun1Btn,ucg_Y2MotoRun1Btn,ucg_Z2MotoRun1Btn,ucg_A2MotoRunBtn,ucg_X2Y2Z2RunBtn,ucg_X2Y2Z2RstBtn,ucg_X2Y2Z2StopBtn,
   ucg_X3MotoRunBtn,ucg_Y3MotoRunBtn,ucg_Z3MotoRunBtn,ucg_A3MotoRunBtn,ucg_X3Y3Z3A3RunBtn,ucg_X3Y3Z3A3RstBtn,ucg_X3Y3Z3A3StopBtn,
   ucg_X1MotoRun2Btn,ucg_Y1MotoRun2Btn,ucg_Z1MotoRun2Btn,ucg_X2MotoRun2Btn,ucg_Y2MotoRun2Btn,ucg_Z2MotoRun2Btn,
   ucg_X1RstBtn,ucg_Y1RstBtn,ucg_Z1RstBtn,ucg_X2RstBtn,ucg_Y2RstBtn,ucg_Z2RstBtn,
   ucg_X1StopBtn,ucg_Y1StopBtn,ucg_Z1StopBtn,ucg_X2StopBtn,ucg_Y2StopBtn,ucg_Z2StopBtn,
   ucg_X1SaveBtn,ucg_Y1SaveBtn,ucg_Z1SaveBtn,ucg_X2SaveBtn,ucg_Y2SaveBtn,ucg_Z2SaveBtn,
   ucg_BasketCapRunBtn;

uint8_t ucg_ValveSwitch[45],ucg_ValveDir[45],ucg_ValveRunBtn,ucg_ValveStopBtn,ucg_ValveReverseBtn;

u32 uwg_NewUsart2Baund,uwg_OldUsart2Baund;
u8 ucg_NewUsart2SlaveAddress,ucg_OldUsart2SlaveAddress,ucg_BaundSlaveAddressSetBtn;

u8 ucg_MotionPosition_flag[10];//数组的值表示哪个仓体被选中,序号表示是哪一组，软件上暂时限定支持10组任务同时运行，具体几组看硬件限制
u8 ucg_UncapPosition_flag;

u16 ZstepShakeinterval=250;
u16 Z1ShakeSpeed=150;
u16 Z1ShakeAcc=80;
u16 Z1ShakeDec=250;
u16 Z1Shaketime=3000;//抖液时间，单位ms


int16 uhwg_MotionPosition_Offest[33][3]={
/*前排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*后排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*烤片仓*/{0,0,0},{0,0,0},{0,0,0},
/*输入仓*/{0,0,0},
/*输出仓*/{0,0,0},{0,0,0},{0,0,0}};


u32 uhwg_MotionPosition_Initial[33][3]={
/*前排染色缸*/{11697,33000,0},{17826,33000,0},{23959,33000,0},{30092,33000,0},{36225,33000,0},{42358,33000,0},{48491,33000,0},{54624,33000,0},{60757,33011,0},{66890,33024,0},{73023,33037,0},{79156,33050,0},{85289,33063,0},
/*后排染色缸*/{23842,310,0},{29969,310,0},{36097,310,0},{42225,310,0},{48353,310,0},{54481,310,0},{60609,310,0},{66737,310,0},{72865,310,0},{78993,310,0},{85121,310,0},{91249,310,0},{97377,310,0},
/*烤片仓*/{1700,250,0},{7833,250,0},{13967,250,0},
/*输入仓*/{1800,32615,0},
/*输出仓*/{93385,32870,0},{98557,32870,0},{103929,32870,0}};

u32 uhwg_MotionPosition_Compose[33][3]={
/*前排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*后排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*烤片仓*/{0,0,0},{0,0,0},{0,0,0},
/*输入仓*/{0,0,0},
/*输出仓*/{0,0,0},{0,0,0},{0,0,0}};

int16 uhwg_UncapPosition_Offest[33][3]={
/*前排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*后排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*烤片仓*/{0,0,0},{0,0,0},{0,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u32 uhwg_UncapPosition_Initial[33][3]={
/*前排染色缸*/{89790,0,0},{83658,0,0},{77530,0,0},{71402,0,0},{65274,0,0},{59146,0,0},{53018,0,0},{46890,0,0},{40762,0,0},{34634,0,0},{28506,0,0},{22378,0,0},{16250,0,0},
/*后排染色缸*/{74336,0,0},{68208,0,0},{62080,0,0},{55952,0,0},{49824,0,0},{43696,0,0},{37568,0,0},{31440,0,0},{25312,0,0},{19184,0,0},{13056,0,0},{6928,0,0},{800,0,0},
/*烤片仓*/{96351,0,0},{90368,0,0},{84385,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u32 uhwg_UncapPosition_Compose[33][3]={
/*前排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*后排染色缸*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*烤片仓*/{0,0,0},{0,0,0},{0,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u16 uhwg_liucheng1ReactTime[30];//流程1反应时间
u8 ucg_liucheng1shijiNum[30];//选中的流程1的试剂号
int16 uhwg_SetTemp[4];//温度设置值
u8 ucg_SetTempBuf[8];//温度设置值存储缓冲区
int16 uhwg_RealTemp[4];//温度实时值
u8 ucg_SetTempFlag[4];//温度设置标志位，1表示开启温控，0表示关闭温控
u16 uhwg_SetTempPID[16];//温度PID及温度校准参数0-3为1-4号温控P参数，4-7为1-4号温控I参数，8-11为1-4号温控D参数，12-15为1-4号温度校准参数
u8 ucg_SetTempPIDBuf[32];//温度PID及温度校准参数存储缓冲区
u8 ucg_SendPidFlag;//温度PID发送标志位，1表示发送，0表示不发送

SemaphoreHandle_t MODH_CmdMutex= NULL; //防止Modbus指令发送冲突的互斥锁
SemaphoreHandle_t UiCmdBianry=NULL;//用于通知串口屏命令接收处理程序
SemaphoreHandle_t MotoMonitBianry=NULL;//用于通知监控任务对电机任务进行挂起恢复等操作
SemaphoreHandle_t InputOutSlicBianry=NULL;//输入输出玻片信号
SemaphoreHandle_t TempControlBianry=NULL;//温度控制信号
SemaphoreHandle_t shiyanlicuheng1Bianry=NULL;//实验流程1的开启信号
SemaphoreHandle_t ValvePumpBianry=NULL;//泵阀启停信号
TaskHandle_t MODH_CmdMutexOwnership;//Modbus互斥量所有权记录

u8 StainingPodStatus[50];//记录对应染色仓的状态，1占用 0未占用
u8 GetTakeSampleDir;//机械臂取放的方式，1取，2放
u8 StainingNumTest;//测试用仓体序号
u8 SlaveaddressSwitchTest;//测试用转换从机地址
volatile u8 MotoTaskFlag;//任务挂起标志位,0表示任务继续，1表示任务挂起，2表示任务结束

u8 SlicSensor1count,SlicSensor2count,SlicSensor3count;//三个光电传感器的计数值
u8 SlicSensorCount;//光电传感器计数总和
u8 InputOutSlicFlag;//输入输出玻片信号,1表示输入仓运行到输入位，2表示输入仓回零，3表示输出仓运行到输出位，4表示输出仓归零
u8 TempControlFlag;//温度控制信号，1表示开启温控，2表示关闭温控
u8 shiyanlicuheng1Flag;//实验流程1的开启信号

u16 shiyan1Param[40][2];//实验流程1的参数
u8 kaopian[10];//kaopian[0]表示是否需要烤片，kaopian[1]表示烤片时间
u8 ActionTime1=5;//吊臂将样品放入试剂后Z1、Z2、Y2、复位动作的时间,这部份时间要算到样品的反应时间内
u8 ActionTime2=6;//样品与试剂反应的时间结束后，机械臂将样品从试剂取出的动作时间，这部份时间要算到样品的反应时间内
u8 ActionTime3=1;//下发指令后机械臂的响应时间
