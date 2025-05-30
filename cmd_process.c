#include "cmd_process.h"
#include "math.h"
#include "cmd_queue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GlobalVariable.h"
#include "modbus_host.h"
#include "Moto_MotionAndUncap.h"
#include "AT24Cxx.h"
#include "shiyanliucheng.h"
static int32 progress_value = 0;                                                     //����������ֵ
static int32 test_value = 0;                                                         //����ֵ
static uint8 update_en = 0;                                                          //���±��
static int32 meter_flag = 0;                                                         //�Ǳ�ָ��������־λ
static int32 num = 0;                                                                //���߲��������
static int sec = 1;                                                                  //ʱ����
static int32 curves_type = 0;                                                        //���߱�־λ  0Ϊ���Ҳ���1Ϊ��ݲ�                  
static int32 second_flag=0;                                                          //ʱ���־λ
static int32 icon_flag = 0;                                                          //ͼ���־λ
static int32 Progress_Value = 0;                                                     //��������ֵ 


/*! 
*  \brief  ��Ϣ��������
*  \param msg ��������Ϣ
*  \param size ��Ϣ����
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
    uint8 cmd_type = msg->cmd_type;       //ָ������,cmd_buffer[0]Ϊ֡ͷ��cmd_buffer[1]Ϊmsg->cmd_type
    uint8 ctrl_msg = msg->ctrl_msg;        //��Ϣ������cmd_buffer[2]Ϊmsg->cmd_msg
    uint8 control_type = msg->control_type;  //�ؼ�����cmd_buffer[7]Ϊmsg->control_type
    uint16 screen_id = PTR2U16(&msg->screen_id); //����IDcmd_buffer[3] cmd_buffer[4]
    uint16 control_id = PTR2U16(&msg->control_id); //�ؼ�IDcmd_buffer[5] cmd_buffer[6]
    uint32 value = PTR2U32(msg->param);             //��ֵcmd_buffer[8] cmd_buffer[9] cmd_buffer[10] cmd_buffer[11]

    switch(cmd_type)
    {  
    case NOTIFY_TOUCH_PRESS:                                                        //����������
    case NOTIFY_TOUCH_RELEASE:                                                      //�������ɿ�
        NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
        break;                                                                    
    case NOTIFY_WRITE_FLASH_OK:                                                     //дFLASH�ɹ�
        NotifyWriteFlash(1);                                         
    case NOTIFY_WRITE_FLASH_FAILD:                                                  //дFLASHʧ��
        NotifyWriteFlash(0);                                                          
    case NOTIFY_READ_FLASH_OK:                                                      //��ȡFLASH�ɹ�
        NotifyReadFlash(1,cmd_buffer,30);                                                        
    case NOTIFY_READ_FLASH_FAILD:                                                   //��ȡFLASHʧ��
        NotifyReadFlash(0,cmd_buffer,30);                                                          
    case NOTIFY_CONTROL:
        {
            if(ctrl_msg==MSG_GET_CURRENT_SCREEN)                                    //����ID�仯֪ͨ
            {
                NotifyScreen(screen_id);                                            //�����л������ĺ���
            }
            else
            {
                switch(control_type)
                {
                case kCtrlButton:                                                   //��ť�ؼ�
                    NotifyButton(screen_id,control_id,msg->param[1]);
                    break;                                                             
                case kCtrlText:                                                     //�ı��ؼ�
                    NotifyText(screen_id,control_id,msg->param);                       
                    break;                                                             
                case kCtrlProgress:                                                 //�������ؼ�
                    NotifyProgress(screen_id,control_id,value);                        
                    break;                                                             
                case kCtrlSlider:                                                   //�������ؼ�
                    NotifySlider(screen_id,control_id,value);
                    break;                                                                                                                          
                case kCtrlRTC:                                                      //����ʱ�ؼ�
                    NotifyTimer(screen_id,control_id);
                    break;
                default:
                    break;
                }
            } 
            break;  
        } 

    default:
        break;
    }

}

/*
*  \brief  ���º�������4�ֽں�2�ֽڵ������д��u8�͵Ļ�������MotoModBuf��                                                     
*  \details                    
*  \param motor ��ͬ��ĵ��                                                     
*  \param offset �������������                                                     
*  \param value ��Ҫд�뻺�������ֵ
*  \param byteCount ��Ҫд�뻺��������ֽ���
*/
void UpdateMotorParam(MotorParams* motor, int offset, int value, int byteCount)
{ 
    if (byteCount ==4)
    {
        motor->MotoModBuf[offset] = GET_BYTE3(value);
        motor->MotoModBuf[offset+1] = GET_BYTE2(value);
        motor->MotoModBuf[offset+2] = GET_BYTE1(value);
        motor->MotoModBuf[offset+3] = GET_BYTE0(value);
    }
    if (byteCount == 2)
    {
        motor->MotoModBuf[offset] = GET_BYTE1(value);
        motor->MotoModBuf[offset+1] = GET_BYTE0(value);
    }
    else printf("�����ֽ�������\r\n");
}

/*
*  \brief  ���º����������ȫ������д��u8�͵Ļ�������MotoModBuf��                                                     
*  \details                    
*  \param motor ��ͬ��ĵ��                                                     
*/
void UpdateMotorParamAll(MotorParams* motor)
{
    motor->MotoModBuf[0]=GET_BYTE3(motor->MaxStep);
    motor->MotoModBuf[1]=GET_BYTE2(motor->MaxStep);
    motor->MotoModBuf[2]=GET_BYTE1(motor->MaxStep);
    motor->MotoModBuf[3]=GET_BYTE0(motor->MaxStep);
    motor->MotoModBuf[4]=GET_BYTE3(motor->SingleStep);
    motor->MotoModBuf[5]=GET_BYTE2(motor->SingleStep);
    motor->MotoModBuf[6]=GET_BYTE1(motor->SingleStep);
    motor->MotoModBuf[7]=GET_BYTE0(motor->SingleStep);
    motor->MotoModBuf[8]=GET_BYTE1(motor->Acc);
    motor->MotoModBuf[9]=GET_BYTE0(motor->Acc);
    motor->MotoModBuf[10]=GET_BYTE1(motor->Dec);
    motor->MotoModBuf[11]=GET_BYTE0(motor->Dec);
    motor->MotoModBuf[12]=GET_BYTE1(motor->Speed);
    motor->MotoModBuf[13]=GET_BYTE0(motor->Speed);
    motor->MotoModBuf[14]=GET_BYTE1(motor->Current);
    motor->MotoModBuf[15]=GET_BYTE0(motor->Current);
    motor->MotoModBuf[16]=GET_BYTE1(motor->RunMode);
    motor->MotoModBuf[17]=GET_BYTE0(motor->RunMode);
    motor->MotoModBuf[18]=GET_BYTE1(motor->ResetOffest);
    motor->MotoModBuf[19]=GET_BYTE0(motor->ResetOffest);
}

/*! 
*  \brief  �����л�֪ͨ
*  \details  ��ǰ����ı�ʱ(�����GetScreen)��ִ�д˺���
*  \param screen_id ��ǰ����ID
*/
void NotifyScreen(uint16 screen_id)
{
    u8 i;
    // if (screen_id==1)//�����е�������ã��ٶ�ȡһ�ε�ǰ����Ĳ���������ʹ����һ�ν���Ĳ���
    // {
    //     for(i=1;i<22;i++)
    //     {
    //         GetControlValue(screen_id,i);
    //     }
    // }
    // if (screen_id==0)//�����е�������ã��ٶ�ȡһ�ε�ǰ����Ĳ���������ʹ����һ����Ĳ���
    // {
    //     for(i=2;i<5;i++)
    //     {
    //         GetControlValue(screen_id,i);
    //     }
    // }

}

/*! 
*  \brief  ���������¼���Ӧ
*  \param press 1���´�������3�ɿ�������
*  \param x x����
*  \param y y����
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{ 
    //TODO: ����û�����
}


/*! 
*  \brief  ��ť�ؼ�֪ͨ
*  \details  ����ť״̬�ı�(�����GetControlValue)ʱ��ִ�д˺���
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param state ��ť״̬��0����1����
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{
    u8 i;
    int16 value1;
    if(screen_id==0)//�����˶�����                                                             
    {
        switch (control_id)
        {
            case 1:
                InputOutSlicFlag=1;
                xSemaphoreGive(InputOutSlicBianry);
                break;
            case 5:
                InputOutSlicFlag=2;
                xSemaphoreGive(InputOutSlicBianry);
                break;
            case 8:
                TempControlFlag=3;
                xSemaphoreGive(TempControlBianry);
                break;
            case 9:
                TempControlFlag=4;
                xSemaphoreGive(TempControlBianry);

                break;
            default:
                break;
        }
    }
    if(screen_id==1)//�÷�����                                                             
    {
        if (control_id >= 1 && control_id <= 90) 
        {
            i = (control_id - 1) / 2;
            if (i < 45) 
            {  // ��ֹiԽ��
                if (control_id % 2 == 1) 
                {  // ����ѡ�а�ť
                    ucg_ValveSwitch[i] = state;
                    ucg_ValveDir[i] = state;
                } 
                else
                {  // ����ť
                    ucg_ValveDir[i] = state + 1;  // 0ͣ����1����2����
                }
            }
        }
        if(control_id==93)//����ť���£�ȫ�����ŷ���
        {
            if(state==1)
            {
                for(i=0;i<45;i++)
                {
                    ucg_ValveDir[i]=2;
                    SetButtonValue(2,2*i+2,1);
                }
            }
            if(state==0)
            {
                for(i=0;i<45;i++)
                {
                    ucg_ValveDir[i]=1;
                    SetButtonValue(2,2*i+2,0);
                }
            }
        }

        if(control_id==94)//�÷��л�
        {
            if(state==1)
            {
                SlaveaddressSwitchTest=1;

            }
            if(state==0)
            {
                SlaveaddressSwitchTest=0;
            }
        }

        if(control_id==92)//ֹͣ��ť���£�ȫ�����Źر�
        {
            for(i=0;i<45;i++)
            {
                ucg_ValveDir[i]=0;
            } 
            ucg_ValveStopBtn=1;
            xSemaphoreGive(ValvePumpBianry);   
        }
        if(control_id==91)//���а�ť����
        {
            ucg_ValveRunBtn=1;
            xSemaphoreGive(ValvePumpBianry);     
        }    
    }
    if(screen_id==2)//������
    {
        if(control_id==4&&state==1)
        {
            ucg_BaundSlaveAddressSetBtn=1;
        }
    }  
    if(screen_id==3)//���������3                                                             
    {
        switch (control_id)
        {
            case 9://����X3������а�ť
                break;
            case 21://����Y3������а�ť

                break;
            case 33://����Z3������а�ť

                break;
            case 45://����A3������а�ť

                break;
            case 11://����X3������水ť
                SetTextValue(3,49,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[8]);
                AT24CXX_Write(0,AxisMotors[8].MotoModBuf,20);//��ucg_X3MotoModBufд��EEPROM�е�0-19��ַ
                SetTextValue(3,49,"����ɹ�!");
                delay_ms(100);
                SetTextValue(3,49,"");
                break;
            case 23://����Y3������水ť
                SetTextValue(3,49,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[9]);
                AT24CXX_Write(20,AxisMotors[9].MotoModBuf,20);//��ucg_X3MotoModBufд��EEPROM�е�20-39��ַ
                SetTextValue(3,49,"����ɹ�!");
                delay_ms(100);
                SetTextValue(3,49,"");
                break;
            case 35://����Z3������水ť
                SetTextValue(3,49,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[10]);
                AT24CXX_Write(40,AxisMotors[10].MotoModBuf,20);//��ucg_X3MotoModBufд��EEPROM�е�40-59��ַ
                SetTextValue(3,49,"����ɹ�!");
                delay_ms(100);
                SetTextValue(3,49,"");
                break;
            case 47://����A3������水ť
                SetTextValue(3,49,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[11]);
                AT24CXX_Write(60,AxisMotors[11].MotoModBuf,20);//��ucg_X3MotoModBufд��EEPROM�е�60-79��ַ
                SetTextValue(3,49,"����ɹ�!");
                delay_ms(100);
                SetTextValue(3,49,"");
                break;
            default:
                break;
        }
    }
    if(screen_id==4)//��е��λ������
    {
        if (control_id<=29)
        {
            ucg_MotionPosition_flag[0]=control_id;
        }

        if (control_id==117)
        {
            uhwg_MotionPosition_Compose[ucg_MotionPosition_flag[0]][0]=uhwg_MotionPosition_Initial[ucg_MotionPosition_flag[0]][0]+uhwg_MotionPosition_Offest[ucg_MotionPosition_flag[0]][0];
            uhwg_MotionPosition_Compose[ucg_MotionPosition_flag[0]][1]=uhwg_MotionPosition_Initial[ucg_MotionPosition_flag[0]][1]+uhwg_MotionPosition_Offest[ucg_MotionPosition_flag[0]][1];
            uhwg_MotionPosition_Compose[ucg_MotionPosition_flag[0]][2]=uhwg_MotionPosition_Initial[ucg_MotionPosition_flag[0]][2]+uhwg_MotionPosition_Offest[ucg_MotionPosition_flag[0]][2];
            ucg_X1Y1Z1RunBtn=1;
        }
        if (control_id==119)//������
        {
            MODH_ReadParam_03H(1,Moto1Location,2);
            printf("x��ǰλ��Ϊ��%d\r\n",MotoLocation[0]);
            DelayMS(600/portTICK_RATE_MS);
            MODH_ReadParam_03H(1,Moto1Status,1);
            printf("x��ǰ״̬Ϊ��%d\r\n",MotoStatus[0]);
        }
        if (control_id==122)//ͣ��
        {
            // X1MotoRunMode=4;//ͣ��ģʽ
            // ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
            // ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);
            // Y1MotoRunMode=4;//ͣ��ģʽ
            // ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
            // ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);
            // Z1MotoRunMode=4;//ͣ��ģʽ
            // ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
            // ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);
            // ucg_X1Y1Z1StopBtn=1;

        }
        if (control_id==124)//��λ
        {
            // X1MotoRunMode=3;//��λģʽ
            // ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
            // ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);
            // Y1MotoRunMode=3;//��λģʽ
            // ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
            // ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);
            // Z1MotoRunMode=3;//��λģʽ
            // Z1MotoSpeed=10;
            // ucg_Z1MotoModBuf[12]=GET_BYTE1(Z1MotoSpeed);
            // ucg_Z1MotoModBuf[13]=GET_BYTE0(Z1MotoSpeed);
            // ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
            // ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);
            // ucg_X1Y1Z1RstBtn=1;
        }
    }
    if(screen_id==5)//����λ������
    {
        if (control_id<=26)
        {
            ucg_UncapPosition_flag=control_id-1;
        }
        if (control_id==108)
        {
            uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][0]=uhwg_UncapPosition_Initial[ucg_UncapPosition_flag][0]+uhwg_UncapPosition_Offest[ucg_UncapPosition_flag][0];
            uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][1]=uhwg_UncapPosition_Initial[ucg_UncapPosition_flag][1]+uhwg_UncapPosition_Offest[ucg_UncapPosition_flag][1];
            uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][2]=uhwg_UncapPosition_Initial[ucg_UncapPosition_flag][2]+uhwg_UncapPosition_Offest[ucg_UncapPosition_flag][2];
            AxisMotors[4].SingleStep=uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][0];//X2������β���
            AxisMotors[5].SingleStep=uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][1];//Y2������β���

            ucg_X2Y2Z2RunBtn=1;
            
        }
    }
    if(screen_id==6)//��������������
    {
        switch (control_id)
        {
            case 8:
                ucg_X1MotoRun1Btn=1;//���ģʽ
                AxisMotors[0].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[0], 16, AxisMotors[0].RunMode, 2);
                break;
            case 9:
                ucg_X1MotoRun2Btn=1;//����ģʽ
                AxisMotors[0].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[0], 16, AxisMotors[0].RunMode, 2);
                break;
            case 10://����
                ucg_X1RstBtn=1;
                AxisMotors[0].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[0], 16, AxisMotors[0].RunMode, 2);
                break;
            case 18:
                ucg_Y1MotoRun1Btn=1;//���ģʽ
                AxisMotors[1].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[1], 16, AxisMotors[1].RunMode, 2);
                break;
            case 19:
                ucg_Y1MotoRun2Btn=1;//����ģʽ
                AxisMotors[1].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[1], 16, AxisMotors[1].RunMode, 2);
                break;

            case 20://����
                ucg_Y1RstBtn=1;
                AxisMotors[1].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[1], 16, AxisMotors[1].RunMode, 2);
                break;

            case 28:
                ucg_Z1MotoRun1Btn=1;//���ģʽ
                AxisMotors[2].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[2], 16, AxisMotors[2].RunMode, 2);
                break;
            case 29:
                ucg_Z1MotoRun2Btn=1;//����ģʽ
                AxisMotors[2].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[2], 16, AxisMotors[2].RunMode, 2);
                break;

            case 30://����
                ucg_Z1RstBtn=1;
                AxisMotors[2].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[2], 16, AxisMotors[2].RunMode, 2);
                break;

            case 47:
                ucg_X2MotoRun1Btn=1;
                AxisMotors[4].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[4], 16, AxisMotors[4].RunMode, 2);
                break;

            case 48:
                ucg_X2MotoRun2Btn=1;
                AxisMotors[4].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[4], 16, AxisMotors[4].RunMode, 2);
                break;

            case 49://����
                ucg_X2RstBtn=1;
                AxisMotors[4].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[4], 16, AxisMotors[4].RunMode, 2);
                break;

            case 57:
                ucg_Y2MotoRun1Btn=1;
                AxisMotors[5].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[5], 16, AxisMotors[5].RunMode, 2);
                break;

            case 58:
                ucg_Y2MotoRun2Btn=1;
                AxisMotors[5].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[5], 16, AxisMotors[5].RunMode, 2);
                break;

            case 59://����
                ucg_Y2RstBtn=1;
                AxisMotors[5].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[5], 16, AxisMotors[5].RunMode, 2);
                break;

            case 67:
                ucg_Z2MotoRun1Btn=1;
                AxisMotors[6].RunMode=0X41;//���ģʽ
                UpdateMotorParam(&AxisMotors[6], 16, AxisMotors[6].RunMode, 2);
                break;
            case 68:
                ucg_Z2MotoRun2Btn=1;
                AxisMotors[6].RunMode=0X01;//����ģʽ
                UpdateMotorParam(&AxisMotors[6], 16, AxisMotors[6].RunMode, 2);
                break;

            case 69://����
                ucg_Z2RstBtn=1;
                AxisMotors[6].RunMode=3;//��λģʽ
                UpdateMotorParam(&AxisMotors[6], 16, AxisMotors[6].RunMode, 2);
                break;

            case 31:
                ucg_X1StopBtn=1;
                break;
            case 32:
                ucg_Y1StopBtn=1;
                break;
            case 33:
                ucg_Z1StopBtn=1;
                break;
            case 34:
                ucg_X1SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[0]);
                AT24CXX_Write(80,AxisMotors[0].MotoModBuf,20);//��ucg_X1MotoModBufд��EEPROM�е�80-99
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
            case 35:
                ucg_Y1SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[1]);
                AT24CXX_Write(100,AxisMotors[1].MotoModBuf,20);//��ucg_Y1MotoModBufд��EEPROM�е�100-119
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
            case 36:
                ucg_Z1SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[2]);
                AT24CXX_Write(120,AxisMotors[2].MotoModBuf,20);//��ucg_Z1MotoModBufд��EEPROM�е�120-139 
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
            case 70:
                ucg_X2StopBtn=1;
                break;
            case 71:
                ucg_Y2StopBtn=1;
                break;
            case 72:
                ucg_Z2StopBtn=1;
                break;
            case 73:
                ucg_X2SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[4]);
                AT24CXX_Write(140,AxisMotors[4].MotoModBuf,20);//��ucg_X2MotoModBufд��EEPROM�е�140-159��ַ
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
            case 74:
                ucg_Y2SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[5]);
                AT24CXX_Write(160,AxisMotors[5].MotoModBuf,20);//��ucg_Y2MotoModBufд��EEPROM�е�160-179��ַ
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
            case 75:
                ucg_Z2SaveBtn=1;
                SetTextValue(6,81,"���ڱ�������...");
                UpdateMotorParamAll(&AxisMotors[6]);
                AT24CXX_Write(180,AxisMotors[6].MotoModBuf,20);//��ucg_Z2MotoModBufд��EEPROM�е�180-199��ַ
                SetTextValue(6,81,"����ɹ�!");
                delay_ms(100);
                SetTextValue(6,81,"");
                break;
        }
    }
    if(screen_id==7)//����ȡ�Ų���
    {
        switch (control_id)
        {
            case 3:
                ucg_X1Y1Z1RstBtn=1;
                ucg_X2Y2Z2RstBtn=1;
                break;
            case 4:
                if(state==0)
                {
                    MotoTaskFlag=1;//�������ָ���־
                    xSemaphoreGive(MotoMonitBianry);
                } 
                if(state==1)
                {
                    MotoTaskFlag=2;//��������־
                    xSemaphoreGive(MotoMonitBianry);
                }
                break;
            case 5:
                ucg_BasketCapRunBtn=1;//����ÿ���׵Ŀ��Ƕ����͵�������������
                break;
            case 12:
                if(state==0)
                {
                    MotoTaskFlag=3;//������ֹ��������־
                    xSemaphoreGive(MotoMonitBianry);
                } 
                if(state==1)
                {
                    MotoTaskFlag=4;//������ֹ��־
                    xSemaphoreGive(MotoMonitBianry);
                }
                break;
            case 13:
                ucg_X1Y1Z1StopBtn=1;
                break;
        }
    }    
    if(screen_id==8)//����ģ������
    {
        switch (control_id)
        {
            case 40:
                TempControlFlag=1;//�¶ȿ���������־
                xSemaphoreGive(TempControlBianry);
                break;
            case 41:
                TempControlFlag=2;//�¶ȿ���ֹͣ��־
                xSemaphoreGive(TempControlBianry);
                break;
            case 32:
                ucg_SetTempFlag[0]=state;
                break;
            case 31:
                ucg_SetTempFlag[1]=state;
                break;
            case 34:
                ucg_SetTempFlag[2]=state;
                break;
            case 72://����PID����
                ucg_SendPidFlag=1;
                xSemaphoreGive(TempControlBianry);
                break;
            case 94://����PID����
                for(i=0;i<16;i++)
                {
                    if(i<4)
                    {
                        ucg_SetTempBuf[2*i]=GET_BYTE1(uhwg_SetTemp[i]);
                        ucg_SetTempBuf[2*i+1]=GET_BYTE0(uhwg_SetTemp[i]);
                    }
                    ucg_SetTempPIDBuf[2*i]=GET_BYTE1(uhwg_SetTempPID[i]);
                    ucg_SetTempPIDBuf[2*i+1]=GET_BYTE0(uhwg_SetTempPID[i]);
                }
                SetTextValue(8,96,"���ڱ�������...");
                AT24CXX_Write(200,ucg_SetTempPIDBuf,32);//��ucg_SetTempPIDBufд��EEPROM�е�200-231��ַ
                delay_ms(10);
                AT24CXX_Write(232,ucg_SetTempBuf,8);//��ucg_SetTempBufд��EEPROM�е�232-239��ַ
                SetTextValue(8,96,"����ɹ�!");
                delay_ms(100);
                SetTextValue(8,96,"");
            
        }
    }
    if(screen_id==9)//ʵ��1��������
    {
        if(control_id==111)
        {
            SetTextValue(9,81,"���ڱ�������...");
            AT24CXX_Write(240,(u8*)shiyan1Param[0],160);//��shiyan1Paramд��EEPROM�е�240-399��ַ
            AT24CXX_Write(720,kaopian,2);
            SetTextValue(9,81,"����ɹ�!");
            delay_ms(100);
            SetTextValue(9,81,"");
        }
        if(control_id==83)
        {
            kaopian[0]=state;//0��ʾ����Ƭ��1��ʾ��Ƭ
        }
        if(control_id==87)
        {
            StartExperiment(1); // ����ʵ��1
            //xSemaphoreGive(shiyanlicuheng1Bianry);
        }
        if(control_id==90)
        {
            ucg_X3Y3Z3A3RunBtn=state;//���ȿ���
        }
    }
    if(screen_id==11)//����1��������
    {   
        if (control_id==2)//����ȡ����Ʒ��ťȷ��
        {
            SetScreen(2);
        }

    }
    if(screen_id==12)//ʵ��2��������
    {
        if(control_id==111)
        {
            SetTextValue(12,81,"���ڱ�������...");
            AT24CXX_Write(400,(u8*)shiyan2Param[0],160);//��shiyan1Paramд��EEPROM�е�400-559��ַ
            AT24CXX_Write(722,kaopian+2,2);
            SetTextValue(12,81,"����ɹ�!");
            delay_ms(100);
            SetTextValue(12,81,"");
        }
        if(control_id==83)
        {
            kaopian[2]=state;//0��ʾ����Ƭ��1��ʾ��Ƭ
        }
        if(control_id==87)
        {
            StartExperiment(2); // ����ʵ��2
        }
        if(control_id==90)
        {
            ucg_X3Y3Z3A3RunBtn=state;//���ȿ���
        }
    }
    if(screen_id==13)//ʵ��3��������
    {
        if(control_id==111)
        {
            SetTextValue(13,81,"���ڱ�������...");
            AT24CXX_Write(560,(u8*)shiyan3Param[0],160);//��shiyan1Paramд��EEPROM�е�560-719��ַ
            AT24CXX_Write(724,kaopian+4,2);
            SetTextValue(13,81,"����ɹ�!");
            delay_ms(100);
            SetTextValue(13,81,"");
        }
        if(control_id==83)
        {
            kaopian[4]=state;//0��ʾ����Ƭ��1��ʾ��Ƭ
        }
        if(control_id==87)
        {
            StartExperiment(3); // ����ʵ��3
        }
        if(control_id==90)
        {
            ucg_X3Y3Z3A3RunBtn=state;//���ȿ���
        }
    }
}

/*! 
*  \brief  �ı��ؼ�֪ͨ
*  \details  ���ı�ͨ�����̸���(�����GetControlValue)ʱ��ִ�д˺���
*  \details  �ı��ؼ����������ַ�����ʽ�·���MCU������ı��ؼ������Ǹ���ֵ��
*  \details  ����Ҫ�ڴ˺����н��·��ַ�������ת�ظ���ֵ��
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*  \param str �ı��ؼ�����
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
	int32 value1;
	int32 value2;
    float value3;

    u8 i,j;
    if(screen_id==0)//�����˶�����                                                             
    {
        switch (control_id)
        {
            case 2:
                sscanf(str,"%d",&value1);
                break;
            case 3:
                sscanf(str,"%d",&value1);
                break;
            case 4:
                sscanf(str,"%d",&value1);
		        // Z1MotoSingleStep=value1;
                // ucg_Z1MotoModBuf[4]=GET_BYTE3(Z1MotoSingleStep);
                // ucg_Z1MotoModBuf[5]=GET_BYTE2(Z1MotoSingleStep);
                // ucg_Z1MotoModBuf[6]=GET_BYTE1(Z1MotoSingleStep);
                // ucg_Z1MotoModBuf[7]=GET_BYTE0(Z1MotoSingleStep);
                break;
        }
    }
    if(screen_id==2)//������
    {
        if(control_id==9)
        {
            sscanf(str,"%d",&value1);
            uwg_OldUsart2Baund=value1;
        }
        if(control_id==10)
        {
            sscanf(str,"%d",&value1);
            ucg_OldUsart2SlaveAddress=value1;
        }
        if(control_id==11)
        {
            sscanf(str,"%d",&value1);
            uwg_NewUsart2Baund=value1;
        }
        if(control_id==12)
        {
            sscanf(str,"%d",&value1);
            ucg_NewUsart2SlaveAddress=value1;
        }
    }
    if(screen_id==3)//���������3                                                             
    {
        switch (control_id)
        {
            //X3�����������
            case 1:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[8], 0, value1, 4);
                break;
            case 2:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[8], 4, value1, 4);
                break;
            case 3:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].Acc=value1;
                //UpdateMotorParam(&AxisMotors[8], 8, value1, 2);
                break;
            case 4:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].Dec=value1;
                //UpdateMotorParam(&AxisMotors[8], 10, value1, 2);
                break;
            case 5:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].Speed=value1;
                //UpdateMotorParam(&AxisMotors[8], 12, value1, 2);
                break; 
            case 6:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].Current=value1;
                //UpdateMotorParam(&AxisMotors[8], 14, value1, 2);
                break;
            case 7:
                sscanf(str,"%d",&value1);
		        AxisMotors[8].RunMode=value1;
                //UpdateMotorParam(&AxisMotors[8], 16, value1, 2);
                break;
            case 8:
                sscanf(str,"%d",&value1);
                AxisMotors[8].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[8], 18, value1, 2);
            //Y3�����������
            case 13:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[9], 0, value1, 4);
                break;
            case 14:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[9], 4, value1, 4);
                break;
            case 15:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].Acc=value1;
                //UpdateMotorParam(&AxisMotors[9], 8, value1, 2);
                break;
            case 16:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].Dec=value1;
                //UpdateMotorParam(&AxisMotors[9], 10, value1, 2);
                break;
            case 17:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].Speed=value1;
                //UpdateMotorParam(&AxisMotors[9], 12, value1, 2);
                break; 
            case 18:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].Current=value1;
                //UpdateMotorParam(&AxisMotors[9], 14, value1, 2);
                break;
            case 19:
                sscanf(str,"%d",&value1);
		        AxisMotors[9].RunMode=value1;
                //UpdateMotorParam(&AxisMotors[9], 16, value1, 2);
                break;
            case 20:
                sscanf(str,"%d",&value1);
                AxisMotors[9].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[9], 18, value1, 2);  
            //Z3�����������
            case 25:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[10], 0, value1, 4);
                break;
            case 26:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[10], 4, value1, 4);
                break;
            case 27:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].Acc=value1;
                //UpdateMotorParam(&AxisMotors[10], 8, value1, 2);
                break;
            case 28:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].Dec=value1;
                //UpdateMotorParam(&AxisMotors[10], 10, value1, 2);
                break;
            case 29:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].Speed=value1;
                //UpdateMotorParam(&AxisMotors[10], 12, value1, 2);
                break; 
            case 30:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].Current=value1;
                //UpdateMotorParam(&AxisMotors[10], 14, value1, 2);
                break;
            case 31:
                sscanf(str,"%d",&value1);
		        AxisMotors[10].RunMode=value1;
                //UpdateMotorParam(&AxisMotors[10], 16, value1, 2);
                break;
            case 32:
                sscanf(str,"%d",&value1);
                AxisMotors[10].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[10], 18, value1, 2);
            //A3�����������
            case 37:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[11], 0, value1, 4);
                break;
            case 38:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[11], 4, value1, 4);
                break;
            case 39:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].Acc=value1;
                //UpdateMotorParam(&AxisMotors[11], 8, value1, 2);
                break;
            case 40:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].Dec=value1;
                //UpdateMotorParam(&AxisMotors[11], 10, value1, 2);
                break;
            case 41:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].Speed=value1;
                //UpdateMotorParam(&AxisMotors[11], 12, value1, 2);
                break; 
            case 42:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].Current=value1;
                //UpdateMotorParam(&AxisMotors[11], 14, value1, 2);
                break;
            case 43:
                sscanf(str,"%d",&value1);
		        AxisMotors[11].RunMode=value1;
                //UpdateMotorParam(&AxisMotors[11], 16, value1, 2);
                break;
            case 44:
                sscanf(str,"%d",&value1);
                AxisMotors[11].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[11], 18, value1, 2);
            default:
                break;
        }
    }
    if(screen_id==4)//��е������
    {
        sscanf(str,"%d",&value2);
        if(control_id>=30&&control_id<=116)
        {
            i = (control_id - 30) / 3;  // �Զ�ȡ��
            j = (control_id - 30) % 3;  // �Զ�ȡ��
            uhwg_MotionPosition_Offest[i][j] = value2;
        } 
    }
    if(screen_id==5)//��������
    {
        sscanf(str,"%d",&value2);
        if(control_id>=30&&control_id<=107)
        {
            sscanf(str,"%d",&value2);
            i = (control_id - 30) / 3;  // �Զ�ȡ��
            j = (control_id - 30) % 3;  // �Զ�ȡ��
            uhwg_MotionPosition_Offest[i][j] = value2;
        }
    }
    if(screen_id==6)//��������������
    {
        sscanf(str,"%d",&value1);
        switch (control_id)
        {
            //X1�����������
            case 1:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[0], 0, value1, 4);
                break;
            case 2:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[0], 4, value1, 4);
                break;
            case 3:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].Acc=value1;
                //UpdateMotorParam(&AxisMotors[0], 8, value1, 2);
                break;
            case 4:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].Dec=value1;
                //UpdateMotorParam(&AxisMotors[0], 10, value1, 2);
                break;
            case 5:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].Speed=value1;
                //UpdateMotorParam(&AxisMotors[0], 12, value1, 2);
                break; 
            case 6:
                sscanf(str,"%d",&value1);
		        AxisMotors[0].Current=value1;
                //UpdateMotorParam(&AxisMotors[0], 14, value1, 2);
                break;
            case 7:
                sscanf(str,"%d",&value1);
                AxisMotors[0].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[0], 18, value1, 2);
            //Y1�����������
            case 11:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[1], 0, value1, 4);
                break;
            case 12:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[1], 4, value1, 4);
                break;
            case 13:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].Acc=value1;
                //UpdateMotorParam(&AxisMotors[1], 8, value1, 2);
                break;
            case 14:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].Dec=value1;
                //UpdateMotorParam(&AxisMotors[1], 10, value1, 2);
                break;
            case 15:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].Speed=value1;
                //UpdateMotorParam(&AxisMotors[1], 12, value1, 2);
                break; 
            case 16:
                sscanf(str,"%d",&value1);
		        AxisMotors[1].Current=value1;
                //UpdateMotorParam(&AxisMotors[1], 14, value1, 2);
                break;
            case 17:
                sscanf(str,"%d",&value1);
                AxisMotors[1].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[1], 18, value1, 2);  
            //Z1�����������
            case 21:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[2], 0, value1, 4);
                break;
            case 22:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[2], 4, value1, 4);
                break;
            case 23:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].Acc=value1;
                //UpdateMotorParam(&AxisMotors[2], 8, value1, 2);
                break;
            case 24:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].Dec=value1;
                //UpdateMotorParam(&AxisMotors[2], 10, value1, 2);
                break;
            case 25:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].Speed=value1;
                //UpdateMotorParam(&AxisMotors[2], 12, value1, 2);
                break; 
            case 26:
                sscanf(str,"%d",&value1);
		        AxisMotors[2].Current=value1;
                //UpdateMotorParam(&AxisMotors[2], 14, value1, 2);
                break;
            case 27:
                sscanf(str,"%d",&value1);
                AxisMotors[2].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[2], 18, value1, 2);
            //X2�����������
            case 40:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[4], 0, value1, 4);
                break;
            case 41:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[4], 4, value1, 4);
                break;
            case 42:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].Acc=value1;
                //UpdateMotorParam(&AxisMotors[4], 8, value1, 2);
                break;
            case 43:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].Dec=value1;
                //UpdateMotorParam(&AxisMotors[4], 10, value1, 2);
                break;
            case 44:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].Speed=value1;
                //UpdateMotorParam(&AxisMotors[4], 12, value1, 2);
                break; 
            case 45:
                sscanf(str,"%d",&value1);
		        AxisMotors[4].Current=value1;
                //UpdateMotorParam(&AxisMotors[4], 14, value1, 2);
                break;
            case 46:
                sscanf(str,"%d",&value1);
                AxisMotors[4].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[4], 18, value1, 2);
            //Y2�����������
            case 50:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[5], 0, value1, 4);
                break;
            case 51:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[5], 4, value1, 4);
                break;
            case 52:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].Acc=value1;
                //UpdateMotorParam(&AxisMotors[5], 8, value1, 2);
                break;
            case 53:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].Dec=value1;
                //UpdateMotorParam(&AxisMotors[5], 10, value1, 2);
                break;
            case 54:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].Speed=value1;
                //UpdateMotorParam(&AxisMotors[5], 12, value1, 2);
                break; 
            case 55:
                sscanf(str,"%d",&value1);
		        AxisMotors[5].Current=value1;
                //UpdateMotorParam(&AxisMotors[5], 14, value1, 2);
                break;
            case 56:
                sscanf(str,"%d",&value1);
                AxisMotors[5].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[5], 18, value1, 2);  
            //Z2�����������
            case 60:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].MaxStep= value1;
                //UpdateMotorParam(&AxisMotors[6], 0, value1, 4);
                break;
            case 61:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].SingleStep= value1;
                //UpdateMotorParam(&AxisMotors[6], 4, value1, 4);
                break;
            case 62:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].Acc=value1;
                //UpdateMotorParam(&AxisMotors[6], 8, value1, 2);
                break;
            case 63:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].Dec=value1;
                //UpdateMotorParam(&AxisMotors[6], 10, value1, 2);
                break;
            case 64:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].Speed=value1;
                //UpdateMotorParam(&AxisMotors[6], 12, value1, 2);
                break; 
            case 65:
                sscanf(str,"%d",&value1);
		        AxisMotors[6].Current=value1;
                //UpdateMotorParam(&AxisMotors[6], 14, value1, 2);
                break;
            case 66:
                sscanf(str,"%d",&value1);
                AxisMotors[6].ResetOffest=value1;
                //UpdateMotorParam(&AxisMotors[6], 18, value1, 2);
            default:
                break;
        }
            
    }
    if(screen_id==7)//����ȡ������
    {
        sscanf(str,"%d",&value2);
        switch (control_id)
        {
            case 1:
                GetTakeSampleDir=value2;
                break;
            case 2:
                StainingNumTest=value2;
                break;
        }
    }
    if(screen_id==8)//����ģ������
    {
        switch (control_id)
        {
            case 8:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[0]=(u16)(((value3*10+0.5)/10.0)*10);//(value3*10+0.5)/10.0)�ȶ�value3����С�����һλ���������룬�¿ذ�Э��30.0����Ҫ����300�������*10,���ǿ��ת��Ϊ����
                break;
            case 11:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[1]=(u16)(((value3*10+0.5)/10.0)*10);
                break;
            case 23:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[2]=(u16)(((value3*10+0.5)/10.0)*10);
                break;
            case 76:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[3]=(u16)(((value3*10+0.5)/10.0)*10);
                break;
            case 46:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[0]=value2;
                break;
            case 57:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[1]=value2;
                break;
            case 65:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[2]=value2;
                break;
            case 87:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[3]=value2;
                break;    
            case 48:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[4]=value2;
                break;
            case 59:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[5]=value2;
                break;
            case 67:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[6]=value2;
                break;
            case 89:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[7]=value2;
                break;
            case 50:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[8]=value2;
                break;
            case 61:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[9]=value2;
                break;
            case 69:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[10]=value2;
                break;
            case 91:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[11]=value2;
                break;
            case 52:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[12]=value2;
                break;
            case 63:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[13]=value2;
                break;
            case 71:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[14]=value2;
                break;
            case 93:
                sscanf(str,"%d",&value2);
                uhwg_SetTempPID[15]=value2;
                break;
        }
    }
    if(screen_id==9)//ʵ��1��������
    {
        if(control_id<=80)
        {
            sscanf(str,"%d",&value2);
            for (i = 0; i < 40; i++) {
                // ÿ����ʼ control_id = 2i + 1 �� 2i + 2
                if (control_id == 2*i + 1 || control_id == 2*i + 2) 
                {
                    if (control_id % 2 == 1) 
                    { // ���ڵ�һ����������
                        shiyan1Param[i][0] = value2;
                    } 
                    else 
                    { // ���ڵڶ�����ż����
                        shiyan1Param[i][1] = value2;
                    }
                }
            }
            
        }
        
        if(control_id==85)
        {
            sscanf(str,"%d",&value2);
            kaopian[1]=value2;//��Ƭʱ��

        }
    }
    if(screen_id==12)//ʵ��2��������
    {
        if(control_id<=80)
        {
            sscanf(str,"%d",&value2);
            for (i = 0; i < 40; i++) {
                // ÿ����ʼ control_id = 2i + 1 �� 2i + 2
                if (control_id == 2*i + 1 || control_id == 2*i + 2) 
                {
                    if (control_id % 2 == 1) 
                    { // ���ڵ�һ����������
                        shiyan2Param[i][0] = value2;
                    } 
                    else 
                    { // ���ڵڶ�����ż����
                        shiyan2Param[i][1] = value2;
                    }
                }
            }
            
        }
        
        if(control_id==85)
        {
            sscanf(str,"%d",&value2);
            kaopian[3]=value2;//��Ƭʱ��

        }
    }
    if(screen_id==13)//ʵ��3��������
    {
        if(control_id<=80)
        {
            sscanf(str,"%d",&value2);
            for (i = 0; i < 40; i++) {
                // ÿ����ʼ control_id = 2i + 1 �� 2i + 2
                if (control_id == 2*i + 1 || control_id == 2*i + 2) 
                {
                    if (control_id % 2 == 1) 
                    { // ���ڵ�һ����������
                        shiyan3Param[i][0] = value2;
                    } 
                    else 
                    { // ���ڵڶ�����ż����
                        shiyan3Param[i][1] = value2;
                    }
                }
            }
            
        }
        
        if(control_id==85)
        {
            sscanf(str,"%d",&value2);
            kaopian[5]=value2;//��Ƭʱ��

        }
    }
}                                                                                
/**************************************************************** */

/*!                                                                              
*  \brief  �������ؼ�֪ͨ                                                       
*  \details  ����GetControlValueʱ��ִ�д˺���                                  
*  \param screen_id ����ID                                                      
*  \param control_id �ؼ�ID                                                     
*  \param value ֵ                                                              
*/                                                                              
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)           
{   
}                                                                                

/*!                                                                              
*  \brief  �������ؼ�֪ͨ                                                       
*  \details  ���������ı�(�����GetControlValue)ʱ��ִ�д˺���                  
*  \param screen_id ����ID                                                      
*  \param control_id �ؼ�ID                                                     
*  \param value ֵ                                                              
*/                                                                              
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)             
{	
}

/*! 
*  \brief  ��ʱ����ʱ֪ͨ����
*  \param screen_id ����ID
*  \param control_id �ؼ�ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{
}

/*! 
*  \brief  ��ȡ�û�FLASH״̬����
*  \param status 0ʧ�ܣ�1�ɹ�
*  \param _data ��������
*  \param length ���ݳ���
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)
{
    u8 i;
    if(status==1)
    {
        for (i=0;i<length;i++)
        {
            printf("%d ",_data+i);
        }
        printf("\r\n");

    }
    if(status==0)
    {
        printf("��flashʧ��");
    }
}

/*! 
*  \brief  д�û�FLASH״̬����
*  \param status 0ʧ�ܣ�1�ɹ�
*/
void NotifyWriteFlash(uint8 status)
{
    if(status==0)
    {
        printf("flashд��ʧ��\r\n");
    }
    if(status==1)
    {
        printf("flashд��ɹ�\r\n");
    }

}

void Him_Init()
{
    u8 i,j,index;
    u8 str[20];
    #if 1
    AT24CXX_Read(0,AxisMotors[8].MotoModBuf,20);//X3�������
    delay_ms(10);
    AT24CXX_Read(20,AxisMotors[9].MotoModBuf,20);//Y3�������
    delay_ms(10);
    AT24CXX_Read(40,AxisMotors[10].MotoModBuf,20);//Z3�������
    delay_ms(10);
    AT24CXX_Read(60,AxisMotors[11].MotoModBuf,20);//A3�������
    delay_ms(10);
    AT24CXX_Read(80,AxisMotors[0].MotoModBuf,20);//X1�������
    delay_ms(10);
    AT24CXX_Read(100,AxisMotors[1].MotoModBuf,20);//Y1�������
    delay_ms(10);
    AT24CXX_Read(120,AxisMotors[2].MotoModBuf,20);//Z1�������
    delay_ms(10);
    AT24CXX_Read(140,AxisMotors[4].MotoModBuf,20);//X2�������
    delay_ms(10);
    AT24CXX_Read(160,AxisMotors[5].MotoModBuf,20);//Y2�������
    delay_ms(10);
    AT24CXX_Read(180,AxisMotors[6].MotoModBuf,20);//Z2�������
    delay_ms(10);
    AT24CXX_Read(200,ucg_SetTempPIDBuf,32);//��·�¿�PID����
    delay_ms(10);
    AT24CXX_Read(232,ucg_SetTempBuf,8);//��·�¿��趨ֵ
    delay_ms(10);
    AT24CXX_Read(240,(u8*)shiyan1Param[0],160);//ʵ��1����
    delay_ms(10);
    AT24CXX_Read(400,(u8*)shiyan2Param[0],160);//ʵ��2����
    delay_ms(10);
    AT24CXX_Read(560,(u8*)shiyan3Param[0],160);//ʵ��3����
    delay_ms(10);
    AT24CXX_Read(720,kaopian,10);//��Ƭ����
    for(i=0;i<12;i++)
    {
        AxisMotors[i].MaxStep=PTR2U32(AxisMotors[i].MotoModBuf+0);
        AxisMotors[i].SingleStep=PTR2U32(AxisMotors[i].MotoModBuf+4);
        AxisMotors[i].Acc=PTR2U16(AxisMotors[i].MotoModBuf+8);
        AxisMotors[i].Dec=PTR2U16(AxisMotors[i].MotoModBuf+10);
        AxisMotors[i].Speed=PTR2U16(AxisMotors[i].MotoModBuf+12);
        AxisMotors[i].Current=PTR2U16(AxisMotors[i].MotoModBuf+14);
        AxisMotors[i].RunMode=PTR2U16(AxisMotors[i].MotoModBuf+16);
        AxisMotors[i].ResetOffest=PTR2U16(AxisMotors[i].MotoModBuf+18);
    }
    for(i=0;i<16;i++)
    {
        if(i<4)
        {
            uhwg_SetTemp[i]=ucg_SetTempBuf[2*i]<<8|ucg_SetTempBuf[2*i+1];
        }
        uhwg_SetTempPID[i]=ucg_SetTempPIDBuf[2*i]<<8|ucg_SetTempPIDBuf[2*i+1];
    }
    #endif

    for (j = 1; j <=37; j+=12)//��X3��Y3��Z3��A3�ĵ��������ʾ������
	{
        if(j==1)
        index=8;
        if(j==13)
        index=9;
        if(j==25)
        index=10;
        if(j==37)
        index=11;
        sprintf(str,"%d",AxisMotors[index].MaxStep);
        SetTextValue(3,j,str);
        sprintf(str,"%d",AxisMotors[index].SingleStep);
        SetTextValue(3,j+1,str);
        sprintf(str,"%d",AxisMotors[index].Acc);
        SetTextValue(3,j+2,str);
        sprintf(str,"%d",AxisMotors[index].Dec);
        SetTextValue(3,j+3,str);
        sprintf(str,"%d",AxisMotors[index].Speed);
        SetTextValue(3,j+4,str);
        sprintf(str,"%d",AxisMotors[index].Current);
        SetTextValue(3,j+5,str);
        sprintf(str,"%d",AxisMotors[index].RunMode);
        SetTextValue(3,j+6,str);
        sprintf(str,"%d",AxisMotors[index].ResetOffest);
        SetTextValue(3,j+7,str);

    }
    for(j=1;j<22;j+=10)//��X1��Y1��Z1�ĵ��������ʾ������
    {
        if(j==1)
        index=0;
        if(j==11)
        index=1;
        if(j==21)
        index=2;
        sprintf(str,"%d",AxisMotors[index].MaxStep);
        SetTextValue(6,j,str); 
        sprintf(str,"%d",AxisMotors[index].SingleStep);
        SetTextValue(6,j+1,str);
        sprintf(str,"%d",AxisMotors[index].Acc);
        SetTextValue(6,j+2,str);
        sprintf(str,"%d",AxisMotors[index].Dec);
        SetTextValue(6,j+3,str);
        sprintf(str,"%d",AxisMotors[index].Speed);
        SetTextValue(6,j+4,str);
        sprintf(str,"%d",AxisMotors[index].Current);
        SetTextValue(6,j+5,str);
        sprintf(str,"%d",AxisMotors[index].ResetOffest);
        SetTextValue(6,j+6,str);
    }
    for(j=40;j<61;j+=10)//��X2��Y2��Z2�ĵ��������ʾ������
    {
        if(j==40)
        index=4;
        if(j==50)
        index=5;
        if(j==60)
        index=6;
        sprintf(str,"%d",AxisMotors[index].MaxStep);
        SetTextValue(6,j,str); 
        sprintf(str,"%d",AxisMotors[index].SingleStep);
        SetTextValue(6,j+1,str);
        sprintf(str,"%d",AxisMotors[index].Acc);
        SetTextValue(6,j+2,str);
        sprintf(str,"%d",AxisMotors[index].Dec);
        SetTextValue(6,j+3,str);
        sprintf(str,"%d",AxisMotors[index].Speed);
        SetTextValue(6,j+4,str);
        sprintf(str,"%d",AxisMotors[index].Current);
        SetTextValue(6,j+5,str);
        sprintf(str,"%d",AxisMotors[index].ResetOffest);
        SetTextValue(6,j+6,str);
    }
    for(i=0;i<40;i++)//��ʵ��1��2��3���̲�����ʾ������
    {
        for(j=0;j<2;j++)
        {
            sprintf(str,"%d",shiyan1Param[i][j]);//ʵ��1����
            if(j==0)
            SetTextValue(9,2*i+1,str);// ���ڵ�һ�����������Լ���
            else 
            SetTextValue(9,2*i+2,str);// ���ڵڶ�����ż����ʱ��

            sprintf(str,"%d",shiyan2Param[i][j]);//ʵ��2����
            if(j==0)
            SetTextValue(12,2*i+1,str);// ���ڵ�һ�����������Լ���
            else 
            SetTextValue(12,2*i+2,str);// ���ڵڶ�����ż����ʱ��

            sprintf(str,"%d",shiyan3Param[i][j]);//ʵ��3����
            if(j==0)
            SetTextValue(13,2*i+1,str);// ���ڵ�һ�����������Լ���
            else 
            SetTextValue(13,2*i+2,str);// ���ڵڶ�����ż����ʱ��
        }
    }
    
    //����Ԥ���¶�
    SetTextFloat(8,8,uhwg_SetTemp[0]/10.0,1,0);
    SetTextFloat(8,11,uhwg_SetTemp[1]/10.0,1,0);
    SetTextFloat(8,23,uhwg_SetTemp[2]/10.0,1,0);
    SetTextFloat(8,76,uhwg_SetTemp[3]/10.0,1,0);
    //���ÿ�Ƭ����
    SetButtonValue(9,83,kaopian[0]);
    sprintf(str,"%d",kaopian[1]);
    SetTextValue(9,85,str);
    
    SetButtonValue(12,83,kaopian[2]);
    sprintf(str,"%d",kaopian[3]);
    SetTextValue(12,85,str);
    
    SetButtonValue(13,83,kaopian[4]);
    sprintf(str,"%d",kaopian[5]);
    SetTextValue(13,85,str);
    
    
}