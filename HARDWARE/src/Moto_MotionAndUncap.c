#include "Moto_MotionAndUncap.h"
#include "modbus_host.h"
#include "timer.h"
#include "delay.h"

void  moto_task(void *pvParameters)
{
	u8 Key_Value,MODRev,i;
	int16 value1;
    u8 delaytime=50;
    MotoShakeWaterInit(ZstepShakeinterval,Z1ShakeSpeed,Z1ShakeAcc,Z1ShakeDec);
    MotoBasketCapInit();
    while(1)
    {
        if(ucg_X1MotoRun1Btn==1)//����X1���������а�ť
        {
            ucg_X1MotoRun1Btn=0;
            MODH_WriteOrReadParam(6,1,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            value1=GET_2BYTE_H(X1MotoSingleStep);
            MODH_WriteOrReadParam(6,1,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            value1=GET_2BYTE_L(X1MotoSingleStep); 
            MODH_WriteOrReadParam(6,1,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
        }
        if(ucg_Y1MotoRun1Btn==1)//����Y1���������а�ť
        {
            ucg_Y1MotoRun1Btn=0;

            MODH_WriteOrReadParam(6,2,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            value1=GET_2BYTE_H(Y1MotoSingleStep);
            MODH_WriteOrReadParam(6,2,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            value1=GET_2BYTE_L(Y1MotoSingleStep); 
            MODH_WriteOrReadParam(6,2,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
        }
        if(ucg_Z1MotoRun1Btn==1)//����Z1���������а�ť
        {
            ucg_Z1MotoRun1Btn=0;

            MODH_WriteOrReadParam(6,3,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            value1=GET_2BYTE_H(Z1MotoSingleStep);
            MODH_WriteOrReadParam(6,3,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            value1=GET_2BYTE_L(Z1MotoSingleStep); 
            MODH_WriteOrReadParam(6,3,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            MODH_WriteOrReadParam(6,3,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
        }
        if(ucg_X1MotoRun2Btn==1)//����X1����������а�ť
        {
            ucg_X1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,1,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(X1MotoSingleStep);
            MODH_WriteOrReadParam(6,1,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(X1MotoSingleStep); 
            MODH_WriteOrReadParam(6,1,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Y1MotoRun2Btn==1)//����Y1����������а�ť
        {
            ucg_Y1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,2,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(Y1MotoSingleStep);
            MODH_WriteOrReadParam(6,2,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Y1MotoSingleStep); 
            MODH_WriteOrReadParam(6,2,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Z1MotoRun2Btn==1)//����Z1����������а�ť
        {
            ucg_Z1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,3,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(Z1MotoSingleStep);
            MODH_WriteOrReadParam(6,3,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Z1MotoSingleStep); 
            MODH_WriteOrReadParam(6,3,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,3,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_X1RstBtn==1)
        {
            ucg_X1RstBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_Y1RstBtn==1)
        {
            ucg_Y1RstBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_Z1RstBtn==1)
        {
            ucg_Z1RstBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_X1StopBtn==1)
        {
            ucg_X1StopBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x6002,0x40,0,NULL,MODH_CmdMutex);//X1��ͣ
            
        }
        if(ucg_Y1StopBtn==1)
        {
            ucg_Y1StopBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y1��ͣ
            
        }
        if(ucg_Z1StopBtn==1)
        {
            ucg_Z1StopBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1��ͣ
            
        }
        if(ucg_X1SaveBtn==1)
        {
            ucg_X1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x1801,0x2211,0,NULL,MODH_CmdMutex);//X1�������
            
        }
        if(ucg_Y1SaveBtn==1)
        {
            ucg_Y1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Y1�������
            
        }
        if(ucg_Z1SaveBtn==1)
        {
            ucg_Z1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Z1�������
            
        }
        
        if(ucg_X2MotoRun1Btn==1)//����X2���������а�ť
        {
            ucg_X2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,4,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            
            value1=GET_2BYTE_H(X2MotoSingleStep);
            MODH_WriteOrReadParam(6,4,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(X2MotoSingleStep); 
            MODH_WriteOrReadParam(6,4,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,4,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Y2MotoRun1Btn==1)//����Y2���������а�ť
        {
            ucg_Y2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,5,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            
            value1=GET_2BYTE_H(Y2MotoSingleStep);
            MODH_WriteOrReadParam(6,5,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Y2MotoSingleStep); 
            MODH_WriteOrReadParam(6,5,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,5,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Z2MotoRun1Btn==1)//����Z2���������а�ť
        {
            ucg_Z2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,6,0x6200,0x41,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ���ģʽ
            
            value1=GET_2BYTE_H(Z2MotoSingleStep);
            MODH_WriteOrReadParam(6,6,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Z2MotoSingleStep); 
            MODH_WriteOrReadParam(6,6,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,6,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_X2MotoRun2Btn==1)//����X2����������а�ť
        {
            ucg_X2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,4,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(X2MotoSingleStep);
            MODH_WriteOrReadParam(6,4,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(X2MotoSingleStep); 
            MODH_WriteOrReadParam(6,4,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,4,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Y2MotoRun2Btn==1)//����Y2����������а�ť
        {
            ucg_Y2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,5,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(Y2MotoSingleStep);
            MODH_WriteOrReadParam(6,5,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Y2MotoSingleStep); 
            MODH_WriteOrReadParam(6,5,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,5,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_Z2MotoRun2Btn==1)//����Z2����������а�ť
        {
            ucg_Z2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,6,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
            
            value1=GET_2BYTE_H(Z2MotoSingleStep);
            MODH_WriteOrReadParam(6,6,0x6201,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            
            value1=GET_2BYTE_L(Z2MotoSingleStep); 
            MODH_WriteOrReadParam(6,6,0x6202,value1,0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            
            MODH_WriteOrReadParam(6,6,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
            
        }
        if(ucg_X2RstBtn==1)
        {
            ucg_X2RstBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_Y2RstBtn==1)
        {
            ucg_Y2RstBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_Z2RstBtn==1)
        {
            ucg_Z2RstBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//��������
            

        }
        if(ucg_X2StopBtn==1)
        {
            ucg_X2StopBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x6002,0x40,0,NULL,MODH_CmdMutex);//X2��ͣ
            
        }
        if(ucg_Y2StopBtn==1)
        {
            ucg_Y2StopBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y2��ͣ
            
        }
        if(ucg_Z2StopBtn==1)
        {
            ucg_Z2StopBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z2��ͣ
            
        }
        if(ucg_X2SaveBtn==1)
        {
            ucg_X2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x1801,0x2211,0,NULL,MODH_CmdMutex);//X2�������
            
        }
        if(ucg_Y2SaveBtn==1)
        {
            ucg_Y2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Y2�������
            
        }
        if(ucg_Z2SaveBtn==1)
        {
            ucg_Z2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Z2�������
            
        }
        if(ucg_X1Y1Z1RstBtn==1&&ucg_X2Y2Z2RstBtn==1)
        {
            ucg_X1Y1Z1RstBtn=0;
            ucg_X2Y2Z2RstBtn=0;
            
            X1Y1Z1GoHome();
            
            X2Y2Z2GoHome();
            
        }
        if(ucg_X1Y1Z1StopBtn==1&&ucg_X2Y2Z2StopBtn==1)
        {
            ucg_X1Y1Z1StopBtn=0;
            ucg_X2Y2Z2StopBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x6002,0x40,0,NULL,MODH_CmdMutex);//X1��ͣ
            
            MODH_WriteOrReadParam(6,2,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y1��ͣ
            
            MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1��ͣ
            
            MODH_WriteOrReadParam(6,4,0x6002,0x40,0,NULL,MODH_CmdMutex);//X2��ͣ
            
            MODH_WriteOrReadParam(6,5,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y2��ͣ
            
            MODH_WriteOrReadParam(6,6,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z2��ͣ
            
        }
        if(ucg_BasketCapRunBtn==1)
        {
            ucg_BasketCapRunBtn=0;
            //TakeGetSample(GetTakeSampleDir,StainingNumTest);
            testliucheng();
        }
        if (ucg_BaundSlaveAddressSetBtn==1)
        {
            // bsp_SetUsart2Baud(uwg_OldUsart2Baund);
            // MODH_WriteParam_06H(ucg_OldUsart2SlaveAddress,0x60,ucg_NewUsart2SlaveAddress);
            // MODH_WriteParam_06H(ucg_NewUsart2SlaveAddress,0x61,3);
            // bsp_SetUsart2Baud(uwg_NewUsart2Baund);
            ucg_BaundSlaveAddressSetBtn=0;
        }
        vTaskDelay(200/portTICK_RATE_MS);
    }
}

/********************************************************************************************************
*	�� �� ��: UpDownBasket
*	����˵��: ����Z1�ᡢY1���˶�ȡ�ŵ���
*	��    ��: Dir : 1ȡ������2�ŵ���
*             StainingNumber��ѡ�еĲ������
*			  ShakeWaterFlag���Ƿ�����ˮ
*			  millisecond����ˮʱ��
*	�� �� ֵ: ��

*	�� �� ֵ: ��
*********************************************************************************************************/
void UpDownBasket(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond)
{
    delay_ms(2);//ʹ��������ֹ���������ȼ�����
    if(Dir==1)//ȡ����
    {
        /*���µ���*/
        if(StainingNumber>=29)//���ڲ�����Ŵ��ڵ���29�Ĳ���û�и��ӣ�����Ҫ���ǣ�����Ҫ��X1Y1Z1MotoMove�˶����ٽ��е���ȡ��
        {
            WaitMotoStop(1,LSMotoStatus,2);//�ȴ�X1�����
            WaitMotoStop(2,LSMotoStatus,2);//�ȴ�Y1�����
            WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z1�����
        }
        MODH_WriteOrReadParam(6,3,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����
        MODH_WriteOrReadParam(6,3,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����

        /*��ס����*/
        MODH_WriteOrReadParam(6,2,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(2,LSMotoStatus,2);//�ȴ�Y�����

        if (ShakeWaterFlag==1) //�Ƿ�����ˮ
        {
            ShakeWater(millisecond);//������

        }

        /*�������������λ*/
        MODH_WriteOrReadParam(6,3,0x6002,0x15,0,NULL,MODH_CmdMutex);  //��������PR5
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����
        MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z1����
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����
        StainingPodStatus[StainingNumber]=0;//��״̬��Ϊ����
            
    }
    if(Dir==2)//�ŵ���
    {
        /*�ƶ��������۵����Ϸ�*/
        if(StainingNumber>=29)//���ڲ�����Ŵ��ڵ���29�Ĳ���û�и��ӣ�����Ҫ���ǣ�����Ҫ��X1Y1Z1MotoMove�˶����ٽ��е���ȡ��
        {
            WaitMotoStop(1,LSMotoStatus,2);//�ȴ�X1�����
            WaitMotoStop(2,LSMotoStatus,2);//�ȴ�Y1�����
            WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z1�����
        }
        MODH_WriteOrReadParam(6,2,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(2,LSMotoStatus,2);//�ȴ�Y�����
        
        /*���µ�������������*/
        MODH_WriteOrReadParam(6,3,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����
        MODH_WriteOrReadParam(6,3,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����

        /*�ѹ�*/
        MODH_WriteOrReadParam(6,2,0x6002,0x12,0,NULL,MODH_CmdMutex);  //��������PR2
        WaitMotoStop(2,LSMotoStatus,2);//�ȴ�Y�����

        /*�������*/
        MODH_WriteOrReadParam(6,3,0x6002,0x15,0,NULL,MODH_CmdMutex);  //��������PR5
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z1�����
        MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z1����
        WaitMotoStop(3,LSMotoStatus,2);//�ȴ�z1�����
        StainingPodStatus[StainingNumber]=1;
        
    }    
}

/********************************************************************************************************
*	�� �� ��: UpDownCap
*	����˵��: ����X2�ᡢY2�ᡢZ2���˶����ز���
*	��    ��: Dir : 1�����ţ�2�ز���
*			  StainingNumber : ѡ�еĲ������ 
*	�� �� ֵ: ��
*********************************************************************************************************/
void UpDownCap(u8 Dir,u8 StainingNumber)
{
    delay_ms(2);//ʹ��������ֹ���������ȼ�����
    if(Dir==1)//������
    {
        if(StainingNumber>=13&&StainingNumber<29)//ǰ����Ⱦɫ�ֿ��Ƿ���һ����X2��ǰ�ƶ���С�̾��뷽���෴
        {
        /*X2�ƶ������Ǵ�*/
        MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
        MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
        MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����

        /*Y2�ƶ������������·�*/
        MODH_WriteOrReadParam(6,5,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(5,LSMotoStatus,2);//�ȴ�X2�����

        /*Z2���𽫸��ӿ��ڽǶȶ�����ֱ*/
        MODH_WriteOrReadParam(6,6,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����

        /*X2�����ƶ�һС�Σ���ֹ����ʱ���ӽǶ�С��90��*/
        MODH_WriteOrReadParam(6,4,0x6002,0x12,0,NULL,MODH_CmdMutex);  //��������PR2
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����
        }
        if(StainingNumber<13)//ǰ����Ⱦɫ�ֿ��Ƿ���һ����X2��ǰ�ƶ���С�̾��뷽���෴
        {
        /*X2�ƶ������Ǵ�*/
        MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
        MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
        MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����

        /*Y2�ƶ������������·�*/
        MODH_WriteOrReadParam(6,5,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(5,LSMotoStatus,2);//�ȴ�X2�����

        /*Z2���𽫸��ӿ��ڽǶȶ�����ֱ*/
        MODH_WriteOrReadParam(6,6,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����

        /*X2��ǰ�ƶ�һС�Σ���ֹ����ʱ���ӽǶ�С��90��*/
        MODH_WriteOrReadParam(6,4,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����
        }
    }
    if(Dir==2)//�ز���
    {
        if(StainingNumber>=13&&StainingNumber<29)//ǰ����Ⱦɫ�ֿ��Ƿ���һ����X2��ǰ�ƶ���С�̾��뷽���෴
        {
        /*X2��ǰ�ƶ�һС�Σ���ֹZ2�½���ʱ����ӽǶȳ���90��*/
        MODH_WriteOrReadParam(6,4,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����
        
        /*Z2���㣬�½������*/
        MODH_WriteOrReadParam(6,6,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1���е�500��λ���ٻ���
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����
        MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z2����
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����

        /*Y2����,���ƶ���Y2���м�*/
        MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Y2����
        WaitMotoStop(5,LSMotoStatus,2);//�ȴ�Y2�����
        }  
        if(StainingNumber<13)//ǰ����Ⱦɫ�ֿ��Ƿ���һ����X2��ǰ�ƶ���С�̾��뷽���෴
        {
        /*X2�����ƶ�һС�Σ���ֹ����ʱ���ӽǶ�С��90��*/
        MODH_WriteOrReadParam(6,4,0x6002,0x12,0,NULL,MODH_CmdMutex);  //��������PR2
        WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����
        
        /*Z2���㣬�½������*/
        MODH_WriteOrReadParam(6,6,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������PR1���е�500��λ���ٻ���
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����
        MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z2����
        WaitMotoStop(6,LSMotoStatus,2);//�ȴ�Z2�����

        /*Y2�ƶ���Y2���м�*/
        MODH_WriteOrReadParam(6,5,0x6002,0x12,0,NULL,MODH_CmdMutex);  //��������PR2
        WaitMotoStop(5,LSMotoStatus,2);//�ȴ�X2�����
        }  
    }
}
/********************************************************************************************************
*	�� �� ��: TakeGetSample
*	����˵��: ����X1��Y1��Z1��X2��Y2��Z2�˶�����Ʒ��ѡ���Ĳ����ڷ������ȡ��
*	��    ��: Dir : 1ȡ��Ʒ��2����Ʒ
*			  StainingNumber : ѡ�еĲ������ 
*			  ShakeWaterFlag : �Ƿ���Ҫ��ˮ
*			  millisecond : ��ˮʱ��
*	�� �� ֵ: ��
*	�� �� ֵ: ��
*********************************************************************************************************/
void TakeGetSample(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond)
{
    
    X1Y1MotoMove(StainingNumber);
    if(Dir==1)
    {
        if(StainingPodStatus[StainingNumber]==1)//��������Ʒ������ȡ��
        {
            UpDownCap(1,StainingNumber);//����
            UpDownBasket(Dir,StainingNumber,ShakeWaterFlag,millisecond);//ȡ����
            UpDownCap(2,StainingNumber);//�ظ�
        }
    }
 
    if(Dir==2)
    {
        if(StainingPodStatus[StainingNumber]==0)//����û��Ʒ�����Է���
        {
            UpDownCap(1,StainingNumber);//����
            UpDownBasket(Dir,StainingNumber,ShakeWaterFlag,millisecond);  //�ŵ���
            UpDownCap(2,StainingNumber);//�ظ�
        }
    }
}

/**********************************************************************************************************
*	�� �� ��: WaitMotoStop
*	����˵��: ��ѯ�������״̬���������������һֱ�ȴ�
*	��    ��: SlaveAddress : ��վ��ַ
*			  reg : �Ĵ�����ַ
*			  num : �Ĵ�������
*	�� �� ֵ: ��
**********************************************************************************************************/
void WaitMotoStop(u8 SlaveAddress,u16 reg,u16 num)
{
    const TickType_t max_time_ticks = pdMS_TO_TICKS(Moto_OverTime); //��ʱʱ��ת��tick��
    TickType_t start_time = xTaskGetTickCount();
    uint64_t Last_Time=0;
    uint64_t Now_Time=0;
    Last_Time=millis();
    Now_Time=millis();
    uint16_t crc1;
    u8 i;
    u8 Monitorstatus=0x04;
        /*Monitorstatus
        Bit0=1 ����
        Bit1=1 ʹ��
        Bit2=1 ����
        Bit3=1 ��Ч
        Bit4=1 ָ�����
        Bit5=1 ·�����
        Bit6=1 �������*/
    MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);

    for (i=0;i<3;i++)//�����һ�ν��ճ��������¶�ȡ����    
    {
        if ((g_tModH.RxCount < 4)|(crc1 != 0))
        {
            MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
            crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
        }
        else
        break;
    }

    while (xTaskGetTickCount()-start_time<max_time_ticks)
    {

        if ((MotoStatus[SlaveAddress-1]&Monitorstatus)==Monitorstatus)//ָ�����״̬λΪ1
        {
            MODH_WriteOrReadParam(3,SlaveAddress,LSMotoStatus,0,2,NULL,MODH_CmdMutex);
            MODH_WriteOrReadParam(3,SlaveAddress,LSMotoLocation,0,2,NULL,MODH_CmdMutex);
            #if 0
            if (SlaveAddress==1)
            {
                printf("X1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==2)
            {
                printf("Y1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==3)
            {
                printf("Z1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==4)
            {
                printf("X2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==7)
            {
                printf("A1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==8)
            {
                printf("A2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            #endif 
        }
        else
        {
            #if 0
            if (SlaveAddress==1)
            {
                printf("X1���ֹͣ\r\n");
            }
            if (SlaveAddress==2)
            {
                printf("Y1���ֹͣ\r\n");
            }
            if (SlaveAddress==3)
            {
                printf("Z1���ֹͣ\r\n"); 
            }
            if (SlaveAddress==4)
            {
                printf("X2���ֹͣ\r\n"); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2���ֹͣ\r\n"); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2���ֹͣ\r\n");
            }
            if (SlaveAddress==7)
            {
                printf("A1���ֹͣ\r\n");
            }
            if (SlaveAddress==8)
            {
                printf("A2���ֹͣ\r\n");
            }
            #endif
            return;
        }
        //Now_Time=millis();
        if(xTaskGetTickCount()-start_time>=Moto_OverTime)
        {
            printf("�ȴ���ʱ\r\n");
        }
    }
}

void WaitMotoStop_WithoutRTOS(u8 SlaveAddress,u16 reg,u16 num)
{
    uint64_t Last_Time=0;
    uint64_t Now_Time=0;
    Last_Time=millis();
    Now_Time=millis();
    uint16_t crc1;
    u8 i;
    u8 delaytime=50;
    u8 Monitorstatus=0x04;
        /*Monitorstatus
        Bit0=1 ����
        Bit1=1 ʹ��
        Bit2=1 ����
        Bit3=1 ��Ч
        Bit4=1 ָ�����
        Bit5=1 ·�����
        Bit6=1 �������*/
    MODH_ReadParam_03H(SlaveAddress,reg,num);
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);

    for (i=0;i<3;i++)//�����һ�ν��ճ��������¶�ȡ����    
    {
        if ((g_tModH.RxCount < 4)|(crc1 != 0))
        {
            delay_ms(delaytime);
            MODH_ReadParam_03H(SlaveAddress,reg,num);
            delay_ms(delaytime);
            crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
        }
        else
        break;
    }

    while (Now_Time-Last_Time<Moto_OverTime)
    {

        if ((MotoStatus[SlaveAddress-1]&Monitorstatus)==Monitorstatus)//ָ�����״̬λΪ1
        {
            delay_ms(delaytime);
            MODH_ReadParam_03H(SlaveAddress,reg,num);
            delay_ms(delaytime);
            MODH_ReadParam_03H(SlaveAddress,LSMotoLocation,num);
            #if 1
            if (SlaveAddress==1)
            {
                printf("X1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==2)
            {
                printf("Y1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==3)
            {
                printf("Z1������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==4)
            {
                printf("X2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2������У���ǰλ��Ϊ��%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            #endif 
        }
        else
        {
            #if 1
            if (SlaveAddress==1)
            {
                printf("X1���ֹͣ\r\n");
            }
            if (SlaveAddress==2)
            {
                printf("Y1���ֹͣ\r\n");
            }
            if (SlaveAddress==3)
            {
                printf("Z1���ֹͣ\r\n"); 
            }
            if (SlaveAddress==4)
            {
                printf("X2���ֹͣ\r\n"); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2���ֹͣ\r\n"); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2���ֹͣ\r\n");
            }
            #endif
            return;
        }
        Now_Time=millis();
        if(Now_Time-Last_Time>=Moto_OverTime)
        {
            printf("�ȴ���ʱ\r\n");
        }
    }
}

/**********************************************************************************************************
*	�� �� ��: WaitMotoStop
*	����˵��: ��ѯ���λ�ã����û��ָ��λ����һֱ�ȴ�
*	��    ��: SlaveAddress : ��վ��ַ
*			  reg : ���λ�üĴ�����ַ
*			  num : �Ĵ�������
              Position: ָ��λ��
              Dir��2 ����ָ��λ�ý����ȴ���1 С��ָ��λ�ý����ȴ�
*	�� �� ֵ: ��
**********************************************************************************************************/
void WaitMotoPosition(u8 SlaveAddress,u16 reg,u16 num,int32 Position,u8 Dir)
{
    const TickType_t max_time_ticks = pdMS_TO_TICKS(Moto_OverTime); //��ʱʱ��ת��tick��
    TickType_t start_time = xTaskGetTickCount(); //��ȡ��ǰʱ��
    uint16_t crc1;
    u8 i;

    for (i=0;i<3;i++)//�����һ�ν��ճ��������¶�ȡ����    
    {
        if ((g_tModH.RxCount < 4)|(crc1 != 0))
        {
            MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
            crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
        }
        else
        break;
    }

    while (xTaskGetTickCount()-start_time<Moto_OverTime)//�ȴ�ʱ�����20s
    {
        MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
        if(Dir==2)
        {
            if(MotoLocation[SlaveAddress-1]>=Position)
            break;
        }
        if(Dir==1)
        {
            if(MotoLocation[SlaveAddress-1]<=Position)
            break;
        }
    }
    if(xTaskGetTickCount()-start_time>=Moto_OverTime) 
    {
        printf("�ȴ���ʱ\r\n");
    }
}

/********************************************************************************************************
*	�� �� ��: XYZMotoMove
*	����˵��: ����XYZ���˶���XY��ֹͣ�������Z�ᣬZ��ֹͣ��Ž����ú���
*	��    ��: StainingNumber : ѡ�еĲ������
*	�� �� ֵ: ��
*********************************************************************************************************/
void X1Y1MotoMove(u8 StainingNumber)
{
    MODH_WriteOrReadParam(6,1,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,1,0x6201,GET_2BYTE_H(uhwg_MotionPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
    MODH_WriteOrReadParam(6,1,0x6202,GET_2BYTE_L(uhwg_MotionPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
    MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������

    MODH_WriteOrReadParam(6,2,0x6200,0X01,0,NULL,MODH_CmdMutex);//�趨PR0ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,2,0x6201,GET_2BYTE_H(uhwg_MotionPosition_Compose[StainingNumber][1]),0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
    MODH_WriteOrReadParam(6,2,0x6202,GET_2BYTE_L(uhwg_MotionPosition_Compose[StainingNumber][1]),0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
    MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //��������
}
void MotoShakeWaterInit(u16 ZstepShakeinterval,u16 Z1ShakeSpeed,u16 Z1ShakeAcc,u16 Z1ShakeDec)//��ʼ����ˮ·��PR3\PR4
{   
    //PR3������ɺ��Զ���ת��PR4,PR4������ɺ��Զ���ת��PR3,ѭ���˶�ֱ����ͣ
    u16 Z1ShakeStop=10;//�����˶�֮���ͣ��ʱ��

    MODH_WriteOrReadParam(6,3,0x6218,0X4441,0,NULL,MODH_CmdMutex);//�趨PR3ģʽΪ���ģʽ��������ɺ��Զ���ת��PR4
    MODH_WriteOrReadParam(6,3,0x6219,GET_2BYTE_H(-ZstepShakeinterval),0,NULL,MODH_CmdMutex);//�趨PR3λ�ø�λ
    MODH_WriteOrReadParam(6,3,0x621A,GET_2BYTE_L(-ZstepShakeinterval),0,NULL,MODH_CmdMutex);//�趨PR3λ�õ�λ
    MODH_WriteOrReadParam(6,3,0x621B,Z1ShakeSpeed,0,NULL,MODH_CmdMutex);//�趨PR3�ٶ� rpm
    MODH_WriteOrReadParam(6,3,0x621C,Z1ShakeAcc,0,NULL,MODH_CmdMutex);//�趨PR3���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x621D,Z1ShakeDec,0,NULL,MODH_CmdMutex);//�趨PR3���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x621E,Z1ShakeStop,0,NULL,MODH_CmdMutex);//�趨PR3ͣ��ʱ��ms

    MODH_WriteOrReadParam(6,3,0x6220,0X4341,0,NULL,MODH_CmdMutex);//�趨PR4ģʽΪ���ģʽ,������ɺ��Զ���ת��PR3
    MODH_WriteOrReadParam(6,3,0x6221,GET_2BYTE_H(ZstepShakeinterval),0,NULL,MODH_CmdMutex);//�趨PR4λ�ø�λ
    MODH_WriteOrReadParam(6,3,0x6222,GET_2BYTE_L(ZstepShakeinterval),0,NULL,MODH_CmdMutex);//�趨PR4λ�õ�λ
    MODH_WriteOrReadParam(6,3,0x6223,Z1ShakeSpeed,0,NULL,MODH_CmdMutex);//�趨PR4�ٶ� rpm
    MODH_WriteOrReadParam(6,3,0x6224,Z1ShakeAcc,0,NULL,MODH_CmdMutex);//�趨PR4���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6225,Z1ShakeDec,0,NULL,MODH_CmdMutex);//�趨PR4���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6226,Z1ShakeStop,0,NULL,MODH_CmdMutex);//�趨PR4ͣ��ʱ��ms
}
void MotoBasketCapInit()//��ʼ��ȡ�ŵ����Ϳ��ظ�·��
{
    int32 Z1step1=3530;//��������ʱ�ľ���λ�ã�Z2������λ3500��Ϊ�˷�ֹ��;���������ô�һ��ʹ�������ȫ��������λλ��
    int32 Z1step2=100;//��������ʱֹͣ�ľ���λ��
    int32 Z1stepShake=1500;//ȡ��������Ȼ��ʼ������ˮ��λ��
    int32 Y1Interval=1100;//Y2��ȡ���ͷ���ʱ�ѹ��ľ���
    int32 Z2step1=90000;//����ʱZ2��һ�������ľ���
    int32 Y2step1=0;//���������>=13ʱ��Ҳ���Ǻ���Ⱦɫ�׿���ʱY2��һ���ľ���
    int32 Y2step2=7900;//���������<13ʱ��Ҳ����ǰ��Ⱦɫ�׿���ʱY2��һ���ľ���
    int32 Y2step3=3500;//���������<13ʱ��Ҳ����ǰ��Ⱦɫ�׿���ʱY2�ڶ����ľ���
    int32 X2Interval=2200;//X2���ظ�ʱ�ƶ���С�ξ��룬��ֹZ2�½�ʱ���ǽǶȴ���90��
    u8 X2Intervalspeed=50;//X2���ظ�ʱ�ƶ���С�ξ�����ٶ�

    #if 1//Y1·������
    MODH_WriteOrReadParam(6,2,0x6208,0X41,0,NULL,MODH_CmdMutex);//�趨PR1ģʽΪ���ģʽ
    MODH_WriteOrReadParam(6,2,0x6209,GET_2BYTE_H(Y1Interval),0,NULL,MODH_CmdMutex);//�趨PR1λ�ø�λ
    MODH_WriteOrReadParam(6,2,0x620A,GET_2BYTE_L(Y1Interval),0,NULL,MODH_CmdMutex);//�趨PR1λ�õ�λ
    MODH_WriteOrReadParam(6,2,0x620B,Y1MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR1�ٶ� rpm
    MODH_WriteOrReadParam(6,2,0x620C,Y1MotoAcc,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,2,0x620D,Y1MotoDec,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,2,0x620E,10,0,NULL,MODH_CmdMutex);//�趨PR1ͣ��ʱ��ms

    MODH_WriteOrReadParam(6,2,0x6210,0X41,0,NULL,MODH_CmdMutex);//�趨PR2ģʽΪ���ģʽ
    MODH_WriteOrReadParam(6,2,0x6211,GET_2BYTE_H(-Y1Interval),0,NULL,MODH_CmdMutex);//�趨PR2λ�ø�λ
    MODH_WriteOrReadParam(6,2,0x6212,GET_2BYTE_L(-Y1Interval),0,NULL,MODH_CmdMutex);//�趨PR2λ�õ�λ
    MODH_WriteOrReadParam(6,2,0x6213,Y1MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR2�ٶ� rpm
    MODH_WriteOrReadParam(6,2,0x6214,Y1MotoAcc,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,2,0x6215,Y1MotoDec,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,2,0x6216,10,0,NULL,MODH_CmdMutex);//�趨PR2ͣ��ʱ��ms

    #endif

    #if 1//Z1·������
    MODH_WriteOrReadParam(6,3,0x6208,0X01,0,NULL,MODH_CmdMutex);//�趨PR1ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,3,0x6209,GET_2BYTE_H(Z1step1),0,NULL,MODH_CmdMutex);//�趨PR1λ�ø�λ
    MODH_WriteOrReadParam(6,3,0x620A,GET_2BYTE_L(Z1step1),0,NULL,MODH_CmdMutex);//�趨PR1λ�õ�λ
    MODH_WriteOrReadParam(6,3,0x620B,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR1�ٶ� rpm
    MODH_WriteOrReadParam(6,3,0x620C,1000,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x620D,1000,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x620E,10,0,NULL,MODH_CmdMutex);//�趨PR1ͣ��ʱ��ms

    MODH_WriteOrReadParam(6,3,0x6210,0X01,0,NULL,MODH_CmdMutex);//�趨PR2ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,3,0x6211,GET_2BYTE_H(Z1stepShake),0,NULL,MODH_CmdMutex);//�趨PR2λ�ø�λ
    MODH_WriteOrReadParam(6,3,0x6212,GET_2BYTE_L(Z1stepShake),0,NULL,MODH_CmdMutex);//�趨PR2λ�õ�λ
    MODH_WriteOrReadParam(6,3,0x6213,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR2�ٶ� rpm
    MODH_WriteOrReadParam(6,3,0x6214,1000,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6215,1000,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6216,10,0,NULL,MODH_CmdMutex);//�趨PR2ͣ��ʱ��ms

    MODH_WriteOrReadParam(6,3,0x6228,0X01,0,NULL,MODH_CmdMutex);//�趨PR5ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,3,0x6229,GET_2BYTE_H(Z1step2),0,NULL,MODH_CmdMutex);//�趨PR5λ�ø�λ
    MODH_WriteOrReadParam(6,3,0x622A,GET_2BYTE_L(Z1step2),0,NULL,MODH_CmdMutex);//�趨PR5λ�õ�λ
    MODH_WriteOrReadParam(6,3,0x622B,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR5�ٶ� rpm
    MODH_WriteOrReadParam(6,3,0x622C,1000,0,NULL,MODH_CmdMutex);//�趨PR5���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x622D,1000,0,NULL,MODH_CmdMutex);//�趨PR5���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,3,0x622E,10,0,NULL,MODH_CmdMutex);//�趨PR5ͣ��ʱ��ms
    #endif

    #if 1//X2·������
    MODH_WriteOrReadParam(6,4,0x6208,0X41,0,NULL,MODH_CmdMutex);//�趨PR1ģʽΪ���ģʽ
    MODH_WriteOrReadParam(6,4,0x6209,GET_2BYTE_H(X2Interval),0,NULL,MODH_CmdMutex);//�趨PR1λ�ø�λ
    MODH_WriteOrReadParam(6,4,0x620A,GET_2BYTE_L(X2Interval),0,NULL,MODH_CmdMutex);//�趨PR1λ�õ�λ
    MODH_WriteOrReadParam(6,4,0x620B,X2Intervalspeed,0,NULL,MODH_CmdMutex);//�趨PR1�ٶ� rpm
    MODH_WriteOrReadParam(6,4,0x620C,1000,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,4,0x620D,1000,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,4,0x620E,10,0,NULL,MODH_CmdMutex);//�趨PR1ͣ��ʱ��ms


    MODH_WriteOrReadParam(6,4,0x6210,0X41,0,NULL,MODH_CmdMutex);//�趨PR2ģʽΪ���ģʽ
    MODH_WriteOrReadParam(6,4,0x6211,GET_2BYTE_H(-X2Interval),0,NULL,MODH_CmdMutex);//�趨PR2λ�ø�λ
    MODH_WriteOrReadParam(6,4,0x6212,GET_2BYTE_L(-X2Interval),0,NULL,MODH_CmdMutex);//�趨PR2λ�õ�λ
    MODH_WriteOrReadParam(6,4,0x6213,X2Intervalspeed,0,NULL,MODH_CmdMutex);//�趨PR2�ٶ� rpm
    MODH_WriteOrReadParam(6,4,0x6214,1000,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,4,0x6215,1000,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,4,0x6216,10,0,NULL,MODH_CmdMutex);//�趨PR2ͣ��ʱ��ms
    #endif

    #if 1//Y2·������
    MODH_WriteOrReadParam(6,5,0x6208,0X01,0,NULL,MODH_CmdMutex);//�趨PR1ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,5,0x6209,GET_2BYTE_H(Y2step2),0,NULL,MODH_CmdMutex);//�趨PR1λ�ø�λ
    MODH_WriteOrReadParam(6,5,0x620A,GET_2BYTE_L(Y2step2),0,NULL,MODH_CmdMutex);//�趨PR1λ�õ�λ
    MODH_WriteOrReadParam(6,5,0x620B,Y2MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR1�ٶ� rpm
    MODH_WriteOrReadParam(6,5,0x620C,Y2MotoAcc,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,5,0x620D,Y2MotoDec,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,5,0x620E,10,0,NULL,MODH_CmdMutex);//�趨PR1ͣ��ʱ��ms


    MODH_WriteOrReadParam(6,5,0x6210,0X01,0,NULL,MODH_CmdMutex);//�趨PR2ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,5,0x6211,GET_2BYTE_H(Y2step3),0,NULL,MODH_CmdMutex);//�趨PR2λ�ø�λ
    MODH_WriteOrReadParam(6,5,0x6212,GET_2BYTE_L(Y2step3),0,NULL,MODH_CmdMutex);//�趨PR2λ�õ�λ
    MODH_WriteOrReadParam(6,5,0x6213,Y2MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR2�ٶ� rpm
    MODH_WriteOrReadParam(6,5,0x6214,Y2MotoAcc,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,5,0x6215,Y2MotoDec,0,NULL,MODH_CmdMutex);//�趨PR2���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,5,0x6216,10,0,NULL,MODH_CmdMutex);//�趨PR2ͣ��ʱ��ms
    #endif

    #if 1//Z2·������
    MODH_WriteOrReadParam(6,6,0x6208,0X01,0,NULL,MODH_CmdMutex);//�趨PR1ģʽΪ����ģʽ
    MODH_WriteOrReadParam(6,6,0x6209,GET_2BYTE_H(500),0,NULL,MODH_CmdMutex);//�趨PR1λ�ø�λ,���λ����Ϊ0���ٶȹ���ᵼ�¹���ײ����λ�������趨500��֮���������ٻ���
    MODH_WriteOrReadParam(6,6,0x620A,GET_2BYTE_H(500),0,NULL,MODH_CmdMutex);//�趨PR1λ�õ�λ
    MODH_WriteOrReadParam(6,6,0x620B,Z2MotoSpeed,0,NULL,MODH_CmdMutex);//�趨PR1�ٶ� rpm
    MODH_WriteOrReadParam(6,6,0x620C,Z2MotoAcc,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,6,0x620D,Z2MotoDec,0,NULL,MODH_CmdMutex);//�趨PR1���ٶ� ms/Krpm
    MODH_WriteOrReadParam(6,6,0x620E,10,0,NULL,MODH_CmdMutex);//�趨PR1ͣ��ʱ��ms
    #endif
}
void ShakeWater(u32 ShakeTime)
{
    //ShakeTimeΪ����ʱ��
    u8 delaytime=25;
    //�����������ˮλ��
    MODH_WriteOrReadParam(6,3,0x6002,0x12,0,NULL,MODH_CmdMutex);  //��������PR2
    WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z�����

    //��ʼ����
    MODH_WriteOrReadParam(6,3,0x6002,0x13,0,NULL,MODH_CmdMutex);//��������PR3
    vTaskDelay(ShakeTime/portTICK_RATE_MS);
    MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1��ͣ
}
/********************************************************************************************************
*	�� �� ��: MODH_WriteOrReadSingleParam
*	����˵��: ���ͻ��߶�ȡModbusָ��
*	��    ��: WriteOrRead : 6дһ��������3������,10д�������
*	          SlaveAddr : �ӻ���ַ
*	          _reg : �Ĵ�����ַ
*	          _value : д���ֵ
*	          _num : ��ȡ�ĸ���
*             *_buf��д�������
*             SemaHandle�������ź������
*	�� �� ֵ: ��
*********************************************************************************************************/
void MODH_WriteOrReadParam(uint8_t WriteOrRead, uint8_t SlaveAddr, uint16_t _reg, uint16_t _value,uint16_t _num,uint8_t *_buf,SemaphoreHandle_t SemaHandle)
{
    u8 delaytime=50;
    BaseType_t xReturn = pdFALSE;
    
    xReturn = xSemaphoreTake(SemaHandle,portMAX_DELAY); /* ��ȡ�����ź��� */
    if (xReturn == pdTRUE)
    {
        
        taskENTER_CRITICAL();
        MODH_CmdMutexOwnership = xTaskGetCurrentTaskHandle();//��¼�����ź������ߣ��ڹ����ɾ������ʱ���ͷ��ź���
        taskEXIT_CRITICAL();
        vTaskDelay(delaytime);//��MODBUS��д�������ӳٶ������ں������ӳ٣���ֹʵ�������л�ʱ��������֡û�б�ʱ��ָ��������Aʹ����MODBUS�����ݺ�����������л�������B������B֮ǰ����ʱ�ȴ��պõ��ˣ�ʹ��MODBUS�����ݣ���������A������B�����һ��ʹ������֡û���ָ����MODBUS����֡����

        if(WriteOrRead==3)
        {
            MODH_ReadParam_03H(SlaveAddr,_reg,_num);
        }
        if(WriteOrRead==6)
        {
            MODH_WriteParam_06H(SlaveAddr,_reg,_value);
        }
        if(WriteOrRead==10)
        {
            MODH_WriteParam_10H(SlaveAddr,_reg,_num,_buf);
        }
        taskENTER_CRITICAL();
        MODH_CmdMutexOwnership = NULL;  // ��¼������
        taskEXIT_CRITICAL();
        xSemaphoreGive(SemaHandle); /* �ͷŻ����ź��� */
        if(ulTaskNotifyTake(pdTRUE, 0)== pdTRUE)//�յ�MonitorTasks֪ͨ����ظ�MonitorTasks
        {
            xTaskNotifyGive(MonitorTasks_Handler); // ����֪ͨ
        }
        
        vTaskDelay(10);
    }
    else
    {
        printf("MODH_WriteOrReadSingleParam:��ȡ�ź���ʧ��!\r\n");
    }
}
void testliucheng()
{
    for(u8 i=0;i<30;i++)
    {
        StainingPodStatus[29]=1;//�������������Ʒ
        TakeGetSample(1,29,0,0);
        TakeGetSample(2,28,1,2000);
        TakeGetSample(1,28,1,2000);
        TakeGetSample(2,0,1,2000);
        TakeGetSample(1,0,1,2000);
        TakeGetSample(2,27,1,2000);
        TakeGetSample(1,27,1,2000);
        TakeGetSample(2,1,1,2000);
        TakeGetSample(1,1,1,2000);
        TakeGetSample(2,26,1,2000);
        TakeGetSample(1,26,1,2000);
        TakeGetSample(2,2,1,2000);
        TakeGetSample(1,2,1,2000);
        TakeGetSample(2,25,1,2000);
        TakeGetSample(1,25,1,2000);
        TakeGetSample(2,3,1,2000);
        TakeGetSample(1,3,1,2000);
        TakeGetSample(2,24,1,2000);
        TakeGetSample(1,24,1,2000);
        TakeGetSample(2,4,1,2000);
        TakeGetSample(1,4,1,2000);
        TakeGetSample(2,23,1,2000);
        TakeGetSample(1,23,1,2000);
        TakeGetSample(2,5,1,2000);
        TakeGetSample(1,5,1,2000);
        TakeGetSample(2,22,1,2000);
        TakeGetSample(1,22,1,2000);
        TakeGetSample(2,6,1,2000);
        TakeGetSample(1,6,1,2000);
        TakeGetSample(2,21,1,2000);
        TakeGetSample(1,21,1,2000);
        TakeGetSample(2,7,1,2000);
        TakeGetSample(1,7,1,2000);
        TakeGetSample(2,20,1,2000);
        TakeGetSample(1,20,1,2000);
        TakeGetSample(2,8,1,2000);
        TakeGetSample(1,8,1,2000);
        TakeGetSample(2,19,1,2000);
        TakeGetSample(1,19,1,2000);
        TakeGetSample(2,9,1,2000);
        TakeGetSample(1,9,1,2000);
        TakeGetSample(2,18,1,2000);
        TakeGetSample(1,18,1,2000);
        TakeGetSample(2,10,1,2000);
        TakeGetSample(1,10,1,2000);
        TakeGetSample(2,17,1,2000);
        TakeGetSample(1,17,1,2000);
        TakeGetSample(2,11,1,2000);
        TakeGetSample(1,11,1,2000);
        TakeGetSample(2,16,1,2000);
        TakeGetSample(1,16,1,2000);
        TakeGetSample(2,12,1,2000);
        TakeGetSample(1,12,1,2000);
        TakeGetSample(2,15,1,2000);
        TakeGetSample(1,15,1,2000);
        TakeGetSample(2,13,1,2000);
        TakeGetSample(1,13,1,2000);
        TakeGetSample(2,14,1,2000);
        TakeGetSample(1,14,1,2000);
        TakeGetSample(2,32,1,2000);
        TakeGetSample(1,32,0,2000);
        TakeGetSample(2,29,1,2000);
        X1Y1Z1GoHome();
        X2Y2Z2GoHome();
        WaitMotoStop(1,LSMotoStatus,2);
        WaitMotoStop(2,LSMotoStatus,2);
        WaitMotoStop(3,LSMotoStatus,2);
        WaitMotoStop(4,LSMotoStatus,2);
        WaitMotoStop(5,LSMotoStatus,2);
        WaitMotoStop(6,LSMotoStatus,2);
    }  
}
void ConvertStep(u8* p,int32 step)//��32λ���ݰ��մ�˴洢��8λ������
{
    *(p+4)=GET_BYTE3(step);
    *(p+5)=GET_BYTE2(step);
    *(p+6)=GET_BYTE1(step);
    *(p+7)=GET_BYTE0(step);
}
void ConvertRunMode(u8* p,int32 step)
{
    *(p+16)=GET_BYTE1(step);
    *(p+17)=GET_BYTE0(step);
}
void PositionInit()
{
    u8 i,j;
    for(i=0;i<33;i++)
    {
        for(j=0;j<3;j++)
        {
            uhwg_UncapPosition_Compose[i][j]=uhwg_UncapPosition_Initial[i][j]+uhwg_UncapPosition_Offest[i][j];
            uhwg_MotionPosition_Compose[i][j]=uhwg_MotionPosition_Initial[i][j]+uhwg_MotionPosition_Offest[i][j];
        }
    }
    
}
void X1Y1Z1GoHome()
{

    MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z1����
    WaitMotoStop(3,LSMotoStatus,2);//�ȴ�Z1�Ḵλ���
    MODH_WriteOrReadParam(6,1,0x6002,0X20,0,NULL,MODH_CmdMutex);//����X1����
    MODH_WriteOrReadParam(6,2,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Y1����
}
void X2Y2Z2GoHome()
{

    MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Y2����
    WaitMotoStop(5,LSMotoStatus,2);//�ȴ�Y2�Ḵλ���
    MODH_WriteOrReadParam(6,4,0x6002,0X20,0,NULL,MODH_CmdMutex);//����X2����
    MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//����Z2����

}
void AllAxisStop()
{
    MODH_WriteOrReadParam(6,1,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,2,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,4,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,5,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,6,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,7,0x6002,0x40,0,NULL,MODH_CmdMutex);
    MODH_WriteOrReadParam(6,8,0x6002,0x40,0,NULL,MODH_CmdMutex);

}
void MotoInit()
{   
    u16 delaytime=10;
    /*0X6006��������λ��16λ��0X6007��������λ��16λ
      0x6008���ø���λ��16λ��0x6009���ø���λ��16λ
      0x600A�趨���㷽ʽ:���򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
      0x600B������λλ�ø�16λ��0x600C������λλ�õ�16λ
      0x600D���û���ֹͣλ�ø�16λ��0x600E���û���ֹͣλ�õ�16λ
      0x600F���û�����٣�0x6010���û������
      0x6011���û�����ٶȣ�0x6012���û�����ٶ�
    */
   
    delay_ms(3000);
    //X1�����ʼ������
    u8 buf1[26]={0x00,0x01,0xAD,0xB0,//����λ110000
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x03,0xE8,//����ֹͣλ��1000
               0x00,0x19,0x00,0x0A,//�������25���������15
               0x03,0xE8,0x03,0xE8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(1,0x6006,13,buf1);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0191,35);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0003,2);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6203,X1MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6204,X1MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6205,X1MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0173,5);//�趨��λʱλ��������������ʱms
    delay_ms(delaytime);

    //Y1�����ʼ������
    u8 buf2[26]={0x00,0x00,0x88,0xB8,//����λ35000
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x00,0xc8,//����ֹͣλ��200
               0x00,0x19,0x00,0x0f,//�������50���������25
               0x03,0xe8,0x03,0xe8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(2,0x6006,13,buf2);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0191,35);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0003,2);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6203,Y1MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6204,Y1MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6205,Y1MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0173,5);//�趨��λʱλ��������������ʱms
    delay_ms(delaytime);

    //Z1�����ʼ������
    u8 buf3[26]={0x00,0x00,0x1F,0x40,//�������λ8000��ʵ���г���3500��Ϊ�˷�ֹ����ֱ��һֱ����ֱ������λ��������λ��ͣ��
               0x00,0x00,0x00,0x00,//�������λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x00,0x00,//����ֹͣλ��0
               0x00,0x08,0x00,0x05,//�������8���������5
               0x0B,0xB8,0x0B,0xB8//������ٶ�3000��������ٶ�3000
              };
    MODH_WriteParam_10H(3,0x6006,13,buf3);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0149,0x25);//DI3����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0191,16);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0003,0);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6201,GET_2BYTE_H(200));//�趨PR0λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6202,GET_2BYTE_L(200));//�趨PR0λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6203,8);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6204,Z1MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6205,Z1MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);

    //X2�����ʼ������
    u8 buf4[26]={0x00,0x01,0x7E,0xD0,//����λ98000
               0xFF,0xFF,0xFB,0xB4,//����λ-1100
               0x00,0x06,//���㷽ʽ������ԭ���źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x03,0x20,//����ֹͣλ��800
               0x00,0x19,0x00,0x0A,//�������25���������10
               0x03,0xe8,0x03,0xe8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(4,0x6006,13,buf4);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0147,0x27);//DI2����Ϊԭ���ź�
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0191,35);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0003,2);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6203,X2MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6204,X2MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6205,X2MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0173,15);//�趨��λʱλ��������������ʱms
    delay_ms(delaytime);

    //Y2�����ʼ������
    u8 buf5[26]={0x00,0x00,0x21,0x34,//����λ8500
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x0D,0xAC,//����ֹͣλ��3500
               0x00,0x32,0x00,0x19,//�������50���������25
               0x03,0xe8,0x03,0xE8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(5,0x6006,13,buf5);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0191,5);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0003,0);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6201,0);//�趨PR0λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6202,0);//�趨PR0λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6203,Y2MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6204,Y2MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6205,Y2MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);

    //Z2�����ʼ������
    u8 buf6[26]={0x00,0x01,0xE8,0x48,//����λ125000
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x00,0x00,//����ֹͣλ��0
               0x00,0xc8,0x00,0x64,//�������200���������100
               0x03,0xe8,0x03,0xE8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(6,0x6006,13,buf6);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0191,5);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0003,0);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6201,GET_2BYTE_H(90000));//�趨PR0λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6202,GET_2BYTE_L(90000));//�趨PR0λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6203,Z2MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6204,Z2MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6205,Z2MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);

    //A1���ֵ����ʼ������
    u8 buf7[26]={0x00,0x0F,0x42,0x40,//����λ1000000
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x00,0x00,//����ֹͣλ��0
               0x00,0xC8,0x00,0x64,//�������200���������100
               0x03,0xe8,0x03,0xE8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(7,0x6006,13,buf7);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0191,5);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0003,0);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6201,GET_2BYTE_H(900000));//�趨PR0λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6202,GET_2BYTE_L(900000));//�趨PR0λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6203,A1MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6204,400);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6205,400);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6208,0X01);//�趨PR1ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6209,GET_2BYTE_H(341881));//�趨PR1λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620A,GET_2BYTE_L(341881));//�趨PR1λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620B,A1MotoSpeed);//�趨PR1�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620C,400);//�趨PR1���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620D,1500);//�趨PR1���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620E,600);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6210,0X41);//�趨PR2ģʽΪ���ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6211,GET_2BYTE_H(-17463));//�趨PR2λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6212,GET_2BYTE_L(-17463));//�趨PR2λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6213,A1MotoSpeed);//�趨PR2�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6214,1500);//�趨PR2���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6215,1500);//�趨PR2���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6216,600);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);

    //A2���ֵ����ʼ������
    u8 buf8[26]={0x00,0x0A,0x75,0x30,//����λ30000
               0x00,0x00,0x00,0x00,//����λ0
               0x00,0x02,//���㷽ʽ�����򣬸���λ�źŻ��㣬�����ƶ���ָ��λ��
               0x00,0x00,0x00,0x00,//��λλ��0
               0x00,0x00,0x00,0x00,//����ֹͣλ��0
               0x00,0x14,0x00,0x0A,//�������20���������10
               0x03,0xe8,0x03,0xE8//������ٶ�1000��������ٶ�1000
              };
    MODH_WriteParam_10H(8,0x6006,13,buf8);//����0x6006-0x6012 Pr������ÿ��Pr������ַ��Ӧһ��16λ�Ĵ���ֵ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0147,0x26);//DI2����Ϊ������λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x1801,0x1111);//��λ��ǰ����
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0191,8);//���õ����ֵ������0-70����λ0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0003,0);//���õ�����ջ�ģʽ��0������2�ջ�
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6200,0X01);//�趨PR0ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6201,GET_2BYTE_H(25000));//�趨PR0λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6202,GET_2BYTE_L(25000));//�趨PR0λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6203,A2MotoSpeed);//�趨PR0�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6204,A2MotoAcc);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6205,A2MotoDec);//�趨PR0���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6206,20);//�趨ͣ��ʱ��ms
    delay_ms(delaytime);

    MODH_WriteParam_06H(8,0x6208,0X01);//�趨PR1ģʽΪ����ģʽ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6209,GET_2BYTE_H(1000));//�趨PR1λ�ø�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620A,GET_2BYTE_L(1000));//�趨PR1λ�õ�λ
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620B,A1MotoSpeed);//�趨PR1�ٶ�rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620C,A1MotoAcc);//�趨PR1���ٶ� ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620D,A1MotoDec);//�趨PR1���ٶ� ms/Krpm
    delay_ms(delaytime);

    #if 0
    MODH_WriteParam_06H(1,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x1801,0x2211);//�������
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x1801,0x2211);//�������
    delay_ms(8000);
    #endif

    MODH_WriteParam_06H(7,0x6002,0X20);//����A1����
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6002,0X20);//����A2����
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6002,0X20);//����Y2����
    delay_ms(delaytime);
    WaitMotoStop_WithoutRTOS(5,LSMotoStatus,2);//�ȴ�Y2�Ḵλ���
    MODH_WriteParam_06H(4,0x6002,0X20);//����X2����
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6002,0X20);//����Z2����
    delay_ms(delaytime);

    MODH_WriteParam_06H(3,0x6002,0X20);//����Z1����
    delay_ms(delaytime);
    WaitMotoStop_WithoutRTOS(3,LSMotoStatus,2);//�ȴ�Z1�Ḵλ���
    MODH_WriteParam_06H(1,0x6002,0X20);//����X1����
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6002,0X20);//����Y1����
    WaitMotoStop_WithoutRTOS(1,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(2,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(3,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(4,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(5,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(6,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(7,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(8,LSMotoStatus,2);
}

