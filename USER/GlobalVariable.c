#include "GlobalVariable.h"
#include "semphr.h"
#include "FreeRTOS.h"

//������
TaskHandle_t StartTask_Handler,Process_message_Handler,moto_Task_Handler;
TaskHandle_t LED0Task_Handler,MotoStausCheckTask_Handler,MonitorTasks_Handler;
TaskHandle_t InputOutSlic_Handler, shiyanlicuheng1_Handler,TempControl_Handler;
TaskHandle_t ValvePump_Handler;

// �����������
MotorParams AxisMotors[12] = {
//MaxStep|SingleStep|Acc|Dec |Speed|Current|Mode|Status|Offset
   {110000,  5000, 3000, 3000,  800,   35,  2,  0,  250},  // X1������������������λΪ0.1A
   { 35000,  5000, 1000, 1000,  300,   35,  2,  0,  220},  // Y1
   {  3500,  5000, 1000, 1000,  120,   16,  2,  0,   20},  // Z1
   {330000,  2000, 1000, 1000, 1000,    5,  2,  0,  500},  // A1
   { 95000,  5000, 2000, 2000,  800,   35,  2,  0,  400},  // X2
   {  8500,  1000, 1000, 1000,  100,    5,  2,  0,  250},  // Y2
   {125000,  1000,  500,  500, 6000,    5,  2,  0,   50},  // Z2
   {  2000,  2000, 2000, 2000,   60,    8,  2,  0,    0},  // A2
   { 32000, 28800,  200,  200,   80, 1500,  7,  0,    0},  // X3���´����ư������λΪmA
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

u8 ucg_MotionPosition_flag[10];//�����ֵ��ʾ�ĸ����屻ѡ��,��ű�ʾ����һ�飬�������ʱ�޶�֧��10������ͬʱ���У����弸�鿴Ӳ������
u8 ucg_UncapPosition_flag;

u16 ZstepShakeinterval=250;
u16 Z1ShakeSpeed=150;
u16 Z1ShakeAcc=80;
u16 Z1ShakeDec=250;
u16 Z1Shaketime=3000;//��Һʱ�䣬��λms


int16 uhwg_MotionPosition_Offest[33][3]={
/*ǰ��Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*����Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*��Ƭ��*/{0,0,0},{0,0,0},{0,0,0},
/*�����*/{0,0,0},
/*�����*/{0,0,0},{0,0,0},{0,0,0}};


u32 uhwg_MotionPosition_Initial[33][3]={
/*ǰ��Ⱦɫ��*/{11697,33000,0},{17826,33000,0},{23959,33000,0},{30092,33000,0},{36225,33000,0},{42358,33000,0},{48491,33000,0},{54624,33000,0},{60757,33011,0},{66890,33024,0},{73023,33037,0},{79156,33050,0},{85289,33063,0},
/*����Ⱦɫ��*/{23842,310,0},{29969,310,0},{36097,310,0},{42225,310,0},{48353,310,0},{54481,310,0},{60609,310,0},{66737,310,0},{72865,310,0},{78993,310,0},{85121,310,0},{91249,310,0},{97377,310,0},
/*��Ƭ��*/{1700,250,0},{7833,250,0},{13967,250,0},
/*�����*/{1800,32615,0},
/*�����*/{93385,32870,0},{98557,32870,0},{103929,32870,0}};

u32 uhwg_MotionPosition_Compose[33][3]={
/*ǰ��Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*����Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*��Ƭ��*/{0,0,0},{0,0,0},{0,0,0},
/*�����*/{0,0,0},
/*�����*/{0,0,0},{0,0,0},{0,0,0}};

int16 uhwg_UncapPosition_Offest[33][3]={
/*ǰ��Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*����Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*��Ƭ��*/{0,0,0},{0,0,0},{0,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u32 uhwg_UncapPosition_Initial[33][3]={
/*ǰ��Ⱦɫ��*/{89790,0,0},{83658,0,0},{77530,0,0},{71402,0,0},{65274,0,0},{59146,0,0},{53018,0,0},{46890,0,0},{40762,0,0},{34634,0,0},{28506,0,0},{22378,0,0},{16250,0,0},
/*����Ⱦɫ��*/{74336,0,0},{68208,0,0},{62080,0,0},{55952,0,0},{49824,0,0},{43696,0,0},{37568,0,0},{31440,0,0},{25312,0,0},{19184,0,0},{13056,0,0},{6928,0,0},{800,0,0},
/*��Ƭ��*/{96351,0,0},{90368,0,0},{84385,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u32 uhwg_UncapPosition_Compose[33][3]={
/*ǰ��Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*����Ⱦɫ��*/{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
/*��Ƭ��*/{0,0,0},{0,0,0},{0,0,0},
{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

u16 uhwg_liucheng1ReactTime[30];//����1��Ӧʱ��
u8 ucg_liucheng1shijiNum[30];//ѡ�е�����1���Լ���
int16 uhwg_SetTemp[4];//�¶�����ֵ
u8 ucg_SetTempBuf[8];//�¶�����ֵ�洢������
int16 uhwg_RealTemp[4];//�¶�ʵʱֵ
u8 ucg_SetTempFlag[4];//�¶����ñ�־λ��1��ʾ�����¿أ�0��ʾ�ر��¿�
u16 uhwg_SetTempPID[16];//�¶�PID���¶�У׼����0-3Ϊ1-4���¿�P������4-7Ϊ1-4���¿�I������8-11Ϊ1-4���¿�D������12-15Ϊ1-4���¶�У׼����
u8 ucg_SetTempPIDBuf[32];//�¶�PID���¶�У׼�����洢������
u8 ucg_SendPidFlag;//�¶�PID���ͱ�־λ��1��ʾ���ͣ�0��ʾ������

SemaphoreHandle_t MODH_CmdMutex= NULL; //��ֹModbusָ��ͳ�ͻ�Ļ�����
SemaphoreHandle_t UiCmdBianry=NULL;//����֪ͨ������������մ������
SemaphoreHandle_t MotoMonitBianry=NULL;//����֪ͨ�������Ե��������й���ָ��Ȳ���
SemaphoreHandle_t InputOutSlicBianry=NULL;//���������Ƭ�ź�
SemaphoreHandle_t TempControlBianry=NULL;//�¶ȿ����ź�
SemaphoreHandle_t shiyanlicuheng1Bianry=NULL;//ʵ������1�Ŀ����ź�
SemaphoreHandle_t ValvePumpBianry=NULL;//�÷���ͣ�ź�
TaskHandle_t MODH_CmdMutexOwnership;//Modbus����������Ȩ��¼

u8 StainingPodStatus[50];//��¼��ӦȾɫ�ֵ�״̬��1ռ�� 0δռ��
u8 GetTakeSampleDir;//��е��ȡ�ŵķ�ʽ��1ȡ��2��
u8 StainingNumTest;//�����ò������
u8 SlaveaddressSwitchTest;//������ת���ӻ���ַ
volatile u8 MotoTaskFlag;//��������־λ,0��ʾ���������1��ʾ�������2��ʾ�������

u8 SlicSensor1count,SlicSensor2count,SlicSensor3count;//������紫�����ļ���ֵ
u8 SlicSensorCount;//��紫���������ܺ�
u8 InputOutSlicFlag;//���������Ƭ�ź�,1��ʾ��������е�����λ��2��ʾ����ֻ��㣬3��ʾ��������е����λ��4��ʾ����ֹ���
u8 TempControlFlag;//�¶ȿ����źţ�1��ʾ�����¿أ�2��ʾ�ر��¿�
u8 shiyanlicuheng1Flag;//ʵ������1�Ŀ����ź�

u16 shiyan1Param[40][2];//ʵ������1�Ĳ���
u8 kaopian[10];//kaopian[0]��ʾ�Ƿ���Ҫ��Ƭ��kaopian[1]��ʾ��Ƭʱ��
u8 ActionTime1=5;//���۽���Ʒ�����Լ���Z1��Z2��Y2����λ������ʱ��,�ⲿ��ʱ��Ҫ�㵽��Ʒ�ķ�Ӧʱ����
u8 ActionTime2=6;//��Ʒ���Լ���Ӧ��ʱ������󣬻�е�۽���Ʒ���Լ�ȡ���Ķ���ʱ�䣬�ⲿ��ʱ��Ҫ�㵽��Ʒ�ķ�Ӧʱ����
u8 ActionTime3=1;//�·�ָ����е�۵���Ӧʱ��
