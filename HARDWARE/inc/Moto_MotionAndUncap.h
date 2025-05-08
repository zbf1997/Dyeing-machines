#ifndef _Moto_MotionAndUncap_H
#define _Moto_MotionAndUncap_H
#include "GlobalVariable.h"
#define LSMotoStatus 0X1003 //����������������״̬
#define Moto1Status 0X0B//��ѯ���1������״̬�Ĵ��������أ�0:ֹͣ 1:���� 2:����
#define Moto2Status 0X17//��ѯ���2������״̬�Ĵ��������أ�0:ֹͣ 1:���� 2:����
#define Moto3Status 0X23//��ѯ���3������״̬�Ĵ��������أ�0:ֹͣ 1:���� 2:����
#define Moto4Status 0X2F//��ѯ���4������״̬�Ĵ��������أ�0:ֹͣ 1:���� 2:����

#define LSMotoLocation 0X602C//��ѯ��������ĵ�ǰλ��
#define Moto1Location 0X09//��ѯ���1�ĵ�ǰλ��
#define Moto2Location 0X15//��ѯ���2�ĵ�ǰλ��
#define Moto3Location 0X21//��ѯ���3�ĵ�ǰλ��
#define Moto4Location 0X2D//��ѯ���4�ĵ�ǰλ��
void WaitMotoStop(u8 SlaveAddress,u16 reg,u16 num);
void WaitMotoStop_WithoutRTOS(u8 SlaveAddress,u16 reg,u16 num);
void WaitMotoPosition(u8 SlaveAddress,u16 reg,u16 num,int32 Position,u8 Dir );
void UpDownBasket(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond);
void UpDownCap(u8 Dir,u8 StainingNumber);
void TakeGetSample(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond);
void TakeGetSampleNoCloseCap(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond);
void X1Y1MotoMove(u8 StainingNumber);
void ConvertStep(u8* p,int32 step);
void ConvertRunMode(u8* p,int32 step);
void MotoInit();
void PositionInit();
void X1Y1Z1GoHome();
void X2Y2Z2GoHome();
void AllAxisStop();
void MotoShakeWaterInit(u16 ZstepShakeinterval,u16 Z1ShakeSpeed,u16 Z1ShakeAcc,u16 Z1ShakeDec);
void ShakeWater(u32 ShakeTime);
void MotoBasketCapInit();
void MODH_WriteOrReadParam(uint8_t WriteOrRead, uint8_t SlaveAddr, uint16_t _reg, uint16_t _value,uint16_t _num,uint8_t *_buf,SemaphoreHandle_t SemaHandle);
void testliucheng();
u8 TankAndReagenMapping(u8 reagnum);
#endif