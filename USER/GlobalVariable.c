#include "GlobalVariable.h"
#include "semphr.h"
#include "FreeRTOS.h"

//������
TaskHandle_t StartTask_Handler;
TaskHandle_t Process_message_Handler;
TaskHandle_t moto_Task_Handler;
TaskHandle_t LED0Task_Handler;
TaskHandle_t MotoStausCheckTask_Handler;
TaskHandle_t MonitorTasks_Handler;
TaskHandle_t InputOutSlic_Handler;
TaskHandle_t shiyanlicuheng1_Handler;
TaskHandle_t TempControl_Handler;
TaskHandle_t ValvePump_Handler;


int32 X1MotoMaxStep=110000;int32 X1MotoSingleStep=5000;//����г̣��������в���
int32 Y1MotoMaxStep=35000;int32 Y1MotoSingleStep=5000;
int32 Z1MotoMaxStep=3500;int32 Z1MotoSingleStep=5000;
int32 A1MotoMaxStep=330000;int32 A1MotoSingleStep=2000;
int32 X2MotoMaxStep=95000;int32 X2MotoSingleStep=5000;
int32 Y2MotoMaxStep=8500;int32 Y2MotoSingleStep=1000;
int32 Z2MotoMaxStep=125000;int32 Z2MotoSingleStep=1000;
int32 A2MotoMaxStep=2000;int32 A2MotoSingleStep=2000;
int32 X3MotoMaxStep=2000;int32 X3MotoSingleStep=2000;
int32 Y3MotoMaxStep=2000;int32 Y3MotoSingleStep=2000;
int32 Z3MotoMaxStep=2000;int32 Z3MotoSingleStep=2000;
int32 A3MotoMaxStep=2000;int32 A3MotoSingleStep=2000;


uint16 X1MotoAcc=3000;uint16 X1MotoDec=3000;uint16 X1MotoCurrent=3500;//���ٶȣ����ٶȣ��������
uint16 Y1MotoAcc=1000;uint16 Y1MotoDec=1000;uint16 Y1MotoCurrent=3500;
uint16 Z1MotoAcc=1000;uint16 Z1MotoDec=1000;uint16 Z1MotoCurrent=1500;
uint16 A1MotoAcc=1000;uint16 A1MotoDec=1000;uint16 A1MotoCurrent=1000;
uint16 X2MotoAcc=2000;uint16 X2MotoDec=2000;uint16 X2MotoCurrent=3500;
uint16 Y2MotoAcc=1000;uint16 Y2MotoDec=1000;uint16 Y2MotoCurrent=600;
uint16 Z2MotoAcc=500;uint16 Z2MotoDec=500;uint16 Z2MotoCurrent=600;
uint16 A2MotoAcc=2000;uint16 A2MotoDec=2000;uint16 A2MotoCurrent=600;
uint16 X3MotoAcc=1000;uint16 X3MotoDec=1000;uint16 X3MotoCurrent=600;
uint16 Y3MotoAcc=1000;uint16 Y3MotoDec=1000;uint16 Y3MotoCurrent=600;
uint16 Z3MotoAcc=1000;uint16 Z3MotoDec=1000;uint16 Z3MotoCurrent=600;
uint16 A3MotoAcc=1000;uint16 A3MotoDec=1000;uint16 A3MotoCurrent=600;

uint8 X1MotoStatus=0;uint16 X1MotoSpeed=800;uint8 X1MotoRunMode=2;//���״̬�������ٶȣ�����ģʽ
uint8 Y1MotoStatus=0;uint16 Y1MotoSpeed=300;uint8 Y1MotoRunMode=2; 
uint8 Z1MotoStatus=0;uint16 Z1MotoSpeed=120;uint8 Z1MotoRunMode=2;
uint8 A1MotoStatus=0;uint16 A1MotoSpeed=1000;uint8 A1MotoRunMode=2;
uint8 X2MotoStatus=0;uint16 X2MotoSpeed=800;uint8 X2MotoRunMode=2;
uint8 Y2MotoStatus=0;uint16 Y2MotoSpeed=100;uint8 Y2MotoRunMode=2; 
uint8 Z2MotoStatus=0;uint16 Z2MotoSpeed=6000;uint8 Z2MotoRunMode=2;
uint8 A2MotoStatus=0;uint16 A2MotoSpeed=60;uint8 A2MotoRunMode=2;
uint8 X3MotoStatus=0;uint16 X3MotoSpeed=95;uint8 X3MotoRunMode=7;
uint8 Y3MotoStatus=0;uint16 Y3MotoSpeed=95;uint8 Y3MotoRunMode=7; 
uint8 Z3MotoStatus=0;uint16 Z3MotoSpeed=95;uint8 Z3MotoRunMode=7;
uint8 A3MotoStatus=0;uint16 A3MotoSpeed=95;uint8 A3MotoRunMode=7;

volatile uint16 ucg_Temp_ch1,MotoStatus[10];
uint8  cmd_buffer[CMD_MAX_SIZE];
volatile int32 MotoLocation[10];

u8 ucg_X1MotoModBuf[30],ucg_Y1MotoModBuf[30],ucg_Z1MotoModBuf[30],ucg_A1MotoModBuf[30],//�洢modbus��������,�ֽ�0-3�洢����г̣��ֽ�4-7���в������ֽ�8-9���ٶȣ��ֽ�10-11���ٶȣ��ֽ�12-13�����ٶȣ��ֽ�14-15����������ֽ�16-17����ģʽ
   ucg_X2MotoModBuf[30],ucg_Y2MotoModBuf[30],ucg_Z2MotoModBuf[30],ucg_A2MotoModBuf[30],
   ucg_X3MotoModBuf[30],ucg_Y3MotoModBuf[30],ucg_Z3MotoModBuf[30],ucg_A3MotoModBuf[30];

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

u16 uhwg_X1ResetOffest=250; u16 uhwg_Y1ResetOffest=220; u16 uhwg_Z1ResetOffest=20; u16 uhwg_A1ResetOffest=500;
u16 uhwg_X2ResetOffest=400; u16 uhwg_Y2ResetOffest=250; u16 uhwg_Z2ResetOffest=50; u16 uhwg_A2ResetOffest;
u16 uhwg_X3ResetOffest; u16 uhwg_Y3ResetOffest; u16 uhwg_Z3ResetOffest; u16 uhwg_A3ResetOffest;

u8 ucg_MotionPosition_flag[10];//�����ֵ��ʾ�ĸ����屻ѡ��,��ű�ʾ����һ�飬�������ʱ�޶�֧��10������ͬʱ���У����弸�鿴Ӳ������
u8 ucg_UncapPosition_flag;

u16 ZstepShakeinterval=100;
u16 Z1ShakeSpeed=250;
u16 Z1ShakeAcc=250;
u16 Z1ShakeDec=250;


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
int16 uhwg_RealTemp[4];//�¶�ʵʱֵ
u8 ucg_SetTempFlag[4];//�¶����ñ�־λ��1��ʾ�����¿أ�0��ʾ�ر��¿�
u16 uhwg_SetTempPID[16];//�¶�PID���¶�У׼����0-3Ϊ1-4���¿�P������4-7Ϊ1-4���¿�I������8-11Ϊ1-4���¿�D������12-15Ϊ1-4���¶�У׼����
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