#include "cmd_process.h"
#include "math.h"
#include "cmd_queue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GlobalVariable.h"
#include "modbus_host.h"
#include "Moto_MotionAndUncap.h"
static int32 progress_value = 0;                                                     //进度条测试值
static int32 test_value = 0;                                                         //测试值
static uint8 update_en = 0;                                                          //更新标记
static int32 meter_flag = 0;                                                         //仪表指针往返标志位
static int32 num = 0;                                                                //曲线采样点计数
static int sec = 1;                                                                  //时间秒
static int32 curves_type = 0;                                                        //曲线标志位  0为正弦波，1为锯齿波                  
static int32 second_flag=0;                                                          //时间标志位
static int32 icon_flag = 0;                                                          //图标标志位
static int32 Progress_Value = 0;                                                     //进度条的值 


/*! 
*  \brief  消息处理流程
*  \param msg 待处理消息
*  \param size 消息长度
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
    uint8 cmd_type = msg->cmd_type;       //指令类型,cmd_buffer[0]为帧头，cmd_buffer[1]为msg->cmd_type
    uint8 ctrl_msg = msg->ctrl_msg;        //消息的类型cmd_buffer[2]为msg->cmd_msg
    uint8 control_type = msg->control_type;  //控件类型cmd_buffer[7]为msg->control_type
    uint16 screen_id = PTR2U16(&msg->screen_id); //画面IDcmd_buffer[3] cmd_buffer[4]
    uint16 control_id = PTR2U16(&msg->control_id); //控件IDcmd_buffer[5] cmd_buffer[6]
    uint32 value = PTR2U32(msg->param);             //数值cmd_buffer[8] cmd_buffer[9] cmd_buffer[10] cmd_buffer[11]

    switch(cmd_type)
    {  
    case NOTIFY_TOUCH_PRESS:                                                        //触摸屏按下
    case NOTIFY_TOUCH_RELEASE:                                                      //触摸屏松开
        NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
        break;                                                                    
    case NOTIFY_WRITE_FLASH_OK:                                                     //写FLASH成功
        NotifyWriteFlash(1);                                         
    case NOTIFY_WRITE_FLASH_FAILD:                                                  //写FLASH失败
        NotifyWriteFlash(0);                                                          
    case NOTIFY_READ_FLASH_OK:                                                      //读取FLASH成功
        NotifyReadFlash(1,cmd_buffer,30);                                                        
    case NOTIFY_READ_FLASH_FAILD:                                                   //读取FLASH失败
        NotifyReadFlash(0,cmd_buffer,30);                                                          
    case NOTIFY_CONTROL:
        {
            if(ctrl_msg==MSG_GET_CURRENT_SCREEN)                                    //画面ID变化通知
            {
                NotifyScreen(screen_id);                                            //画面切换调动的函数
            }
            else
            {
                switch(control_type)
                {
                case kCtrlButton:                                                   //按钮控件
                    NotifyButton(screen_id,control_id,msg->param[1]);
                    break;                                                             
                case kCtrlText:                                                     //文本控件
                    NotifyText(screen_id,control_id,msg->param);                       
                    break;                                                             
                case kCtrlProgress:                                                 //进度条控件
                    NotifyProgress(screen_id,control_id,value);                        
                    break;                                                             
                case kCtrlSlider:                                                   //滑动条控件
                    NotifySlider(screen_id,control_id,value);
                    break;                                                                                                                          
                case kCtrlRTC:                                                      //倒计时控件
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


/*! 
*  \brief  画面切换通知
*  \details  当前画面改变时(或调用GetScreen)，执行此函数
*  \param screen_id 当前画面ID
*/
void NotifyScreen(uint16 screen_id)
{
    u8 i;
    if (screen_id==1)//画面切到电机设置，再读取一次当前界面的参数，避免使用上一界面的参数
    {
        for(i=1;i<22;i++)
        {
            GetControlValue(screen_id,i);
        }
    }
    if (screen_id==0)//画面切到电机设置，再读取一次当前界面的参数，避免使用上一界面的参数
    {
        for(i=2;i<5;i++)
        {
            GetControlValue(screen_id,i);
        }
    }

}

/*! 
*  \brief  触摸坐标事件响应
*  \param press 1按下触摸屏，3松开触摸屏
*  \param x x坐标
*  \param y y坐标
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{ 
    //TODO: 添加用户代码
}


/*! 
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{
    u8 i;
    int16 value1;
    if(screen_id==1)//电机驱动板1                                                              
    {
        switch (control_id)
        {
            case 9://按下X1电机运行按钮

                break;
            case 21://按下Y1电机运行按钮

                break;
            case 33://按下Z1电机运行按钮

                break;
            case 45://按下A1电机运行按钮

                break;
            case 11://按下X1电机保存按钮

                break;
            case 23://按下Y1电机保存按钮

                break;
            case 35://按下Z1电机保存按钮

                break;
            case 47://按下A1电机保存按钮

                break;
            default:
                break;
        }
    }
    if(screen_id==0)//三轴运动控制                                                             
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
    if(screen_id==2)//泵阀设置                                                             
    {
        for(i=0;i<45;i++)
        {
            if(control_id==2*i+1)//阀门选中按钮
            {
                ucg_ValveSwitch[i]=state;
                ucg_ValveDir[i]=state;
            }
            if(control_id==2*i+2)//反向按钮按下反转，不按下正转
            {
                ucg_ValveDir[i]=state+1;//0停机，1正向运行，2反向运行
            }
        }
        if(control_id==93)//反向按钮按下，全部阀门反向
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

        if(control_id==94)//全选按钮按下
        {
            if(state==1)
            {
                SlaveaddressSwitchTest=1;
                // for(i=0;i<45;i++)
                // {
                //     ucg_ValveSwitch[i]=1;
                //     SetButtonValue(2,2*i+1,1);
                // }
            }
            if(state==0)
            {
                SlaveaddressSwitchTest=0;
                // for(i=0;i<45;i++)
                // {
                //     ucg_ValveSwitch[i]=0;
                //     SetButtonValue(2,2*i+1,0);
                // }
            }
        }

        if(control_id==92)//停止按钮按下，全部阀门关闭
        {
            for(i=0;i<45;i++)
            {
                ucg_ValveDir[i]=0;
            } 
            ucg_ValveStopBtn=1;
            xSemaphoreGive(ValvePumpBianry);   
        }
        if(control_id==91)//运行按钮按下
        {
            ucg_ValveRunBtn=1;
            xSemaphoreGive(ValvePumpBianry);     
        }    
    }
    if(screen_id==3)//主界面
    {
        if(control_id==4&&state==1)
        {
            ucg_BaundSlaveAddressSetBtn=1;
        }
    }
    if(screen_id==4)//电机驱动板2                                                              
    {
        switch (control_id)
        {
            case 9://按下X2电机运行按钮
                
                break;
            case 21://按下Y2电机运行按钮
                
                break;
            case 33://按下Z2电机运行按钮
               
                break;
            case 45://按下A2电机运行按钮
                
                break;
            case 11://按下X2电机保存按钮
                
                break;
            case 23://按下Y2电机保存按钮
                break;
            case 35://按下Z2电机保存按钮
                break;
            case 47://按下A2电机保存按钮
                break;
            default:
                break;
        }
    }
    if(screen_id==5)//电机驱动板3                                                             
    {
        switch (control_id)
        {
            case 9://按下X3电机运行按钮

                break;
            case 21://按下Y3电机运行按钮

                break;
            case 33://按下Z3电机运行按钮

                break;
            case 45://按下A3电机运行按钮

                break;
            case 11://按下X3电机保存按钮
                break;
            case 23://按下Y3电机保存按钮
                break;
            case 35://按下Z3电机保存按钮
                break;
            case 47://按下A3电机保存按钮
                break;
            default:
                break;
        }
    }
    if(screen_id==6)//机械臂位置设置
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
        if (control_id==119)//测试用
        {
            MODH_ReadParam_03H(1,Moto1Location,2);
            printf("x当前位置为：%d\r\n",MotoLocation[0]);
            DelayMS(600/portTICK_RATE_MS);
            MODH_ReadParam_03H(1,Moto1Status,1);
            printf("x当前状态为：%d\r\n",MotoStatus[0]);
        }
        if (control_id==122)//停机
        {
            X1MotoRunMode=4;//停机模式
            ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
            ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);
            Y1MotoRunMode=4;//停机模式
            ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
            ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);
            Z1MotoRunMode=4;//停机模式
            ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
            ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);
            ucg_X1Y1Z1StopBtn=1;

        }
        if (control_id==124)//复位
        {
            X1MotoRunMode=3;//复位模式
            ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
            ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);
            Y1MotoRunMode=3;//复位模式
            ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
            ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);
            Z1MotoRunMode=3;//复位模式
            Z1MotoSpeed=10;
            ucg_Z1MotoModBuf[12]=GET_BYTE1(Z1MotoSpeed);
            ucg_Z1MotoModBuf[13]=GET_BYTE0(Z1MotoSpeed);
            ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
            ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);
            ucg_X1Y1Z1RstBtn=1;
        }
    }
    if(screen_id==7)//开盖位置设置
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
            X2MotoSingleStep=uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][0];
            Y2MotoSingleStep=uhwg_UncapPosition_Compose[ucg_UncapPosition_flag][1];
            ucg_X2Y2Z2RunBtn=1;
            
        }
    }
    if(screen_id==8)
    {
        switch (control_id)
        {
            case 8:
                ucg_X1MotoRun1Btn=1;//相对模式
                break;
            case 9:
                ucg_X1MotoRun2Btn=1;//绝对模式
                break;
            case 10://回零
                ucg_X1RstBtn=1;
                break;
            case 18:
                ucg_Y1MotoRun1Btn=1;//相对模式
                break;
            case 19:
                ucg_Y1MotoRun2Btn=1;//绝对模式
                break;

            case 20://回零
                ucg_Y1RstBtn=1;
                break;

            case 28:
                ucg_Z1MotoRun1Btn=1;//相对模式
                break;
            case 29:
                ucg_Z1MotoRun2Btn=1;//绝对模式
                break;

            case 30://回零
                ucg_Z1RstBtn=1;
                break;

            case 47:
                ucg_X2MotoRun1Btn=1;
                break;

            case 48:
                ucg_X2MotoRun2Btn=1;
                break;

            case 49://回零
                ucg_X2RstBtn=1;
                break;

            case 57:
                ucg_Y2MotoRun1Btn=1;
                break;

            case 58:
                ucg_Y2MotoRun2Btn=1;
                break;

            case 59://回零
                ucg_Y2RstBtn=1;
                break;

            case 67:
                ucg_Z2MotoRun1Btn=1;
                break;
            case 68:
                ucg_Z2MotoRun2Btn=1;
                break;

            case 69://回零
                ucg_Z2RstBtn=1;
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
                break;
            case 35:
                ucg_Y1SaveBtn=1;
                break;
            case 36:
                ucg_Z1SaveBtn=1;
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
                break;
            case 74:
                ucg_Y2SaveBtn=1;
                break;
            case 75:
                ucg_Z2SaveBtn=1;
                break;
        }
    }
    if(screen_id==9)
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
                    MotoTaskFlag=1;//任务挂起恢复标志
                    xSemaphoreGive(MotoMonitBianry);
                } 
                if(state==1)
                {
                    MotoTaskFlag=2;//任务挂起标志
                    xSemaphoreGive(MotoMonitBianry);
                }
                break;
            case 5:
                ucg_BasketCapRunBtn=1;//运行每个缸的开盖动作和吊放蓝动作测试
                break;
            case 12:
                if(state==0)
                {
                    MotoTaskFlag=3;//任务终止后重启标志
                    xSemaphoreGive(MotoMonitBianry);
                } 
                if(state==1)
                {
                    MotoTaskFlag=4;//任务终止标志
                    xSemaphoreGive(MotoMonitBianry);
                }
                break;
            case 13:
                ucg_X1Y1Z1StopBtn=1;
                break;
        }
    }    
    if(screen_id==10)
    {
        switch (control_id)
        {
            case 40:
                TempControlFlag=1;//温度控制启动标志
                xSemaphoreGive(TempControlBianry);
                break;
            case 41:
                TempControlFlag=2;//温度控制停止标志
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
            case 72://发送PID参数
                ucg_SendPidFlag=1;
                xSemaphoreGive(TempControlBianry);
                break;
            
        }
    }

}

/*! 
*  \brief  文本控件通知
*  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
*  \details  文本控件的内容以字符串形式下发到MCU，如果文本控件内容是浮点值，
*  \details  则需要在此函数中将下发字符串重新转回浮点值。
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param str 文本控件内容
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
	int32 value1;
	int32 value2;
    float value3;

    u8 i;
    if(screen_id==1)                                                             
    {
        switch (control_id)
        {
            case 1:
                sscanf(str,"%d",&value1);
		        X1MotoMaxStep=value1;
                ucg_X1MotoModBuf[0]=GET_BYTE3(X1MotoMaxStep);
                ucg_X1MotoModBuf[1]=GET_BYTE2(X1MotoMaxStep);
                ucg_X1MotoModBuf[2]=GET_BYTE1(X1MotoMaxStep);
                ucg_X1MotoModBuf[3]=GET_BYTE0(X1MotoMaxStep);
                printf("X1MotoMaxStep=%d\r\n",X1MotoMaxStep);
                break;
            case 2:
                sscanf(str,"%d",&value1);
		        X1MotoSingleStep=value1;
                ucg_X1MotoModBuf[4]=GET_BYTE3(X1MotoSingleStep);
                ucg_X1MotoModBuf[5]=GET_BYTE2(X1MotoSingleStep);
                ucg_X1MotoModBuf[6]=GET_BYTE1(X1MotoSingleStep);
                ucg_X1MotoModBuf[7]=GET_BYTE0(X1MotoSingleStep);
                printf("X1MotoSingleStep=%d\r\n",X1MotoSingleStep);
                break;
            case 3:
                sscanf(str,"%d",&value2);
		        X1MotoAcc=value2;
                ucg_X1MotoModBuf[8]=GET_BYTE1(X1MotoAcc);
                ucg_X1MotoModBuf[9]=GET_BYTE0(X1MotoAcc);
                printf("X1MotoAcc=%d\r\n",X1MotoAcc);
                break;
            case 4:
                sscanf(str,"%d",&value2);
		        X1MotoDec=value2;
                ucg_X1MotoModBuf[10]=GET_BYTE1(X1MotoDec);
                ucg_X1MotoModBuf[11]=GET_BYTE0(X1MotoDec);
                printf("X1MotoDec=%d\r\n",X1MotoDec);
                break;
            case 5:
                sscanf(str,"%d",&value2);
		        X1MotoSpeed=value2;
                ucg_X1MotoModBuf[12]=GET_BYTE1(X1MotoSpeed);
                ucg_X1MotoModBuf[13]=GET_BYTE0(X1MotoSpeed);
                printf("X1MotoSpeed=%d\r\n",X1MotoSpeed);
                break; 
            case 6:
                sscanf(str,"%d",&value2);
		        X1MotoCurrent=value2;
                ucg_X1MotoModBuf[14]=GET_BYTE1(X1MotoCurrent);
                ucg_X1MotoModBuf[15]=GET_BYTE0(X1MotoCurrent);
                printf("X1MotoCurrent=%d\r\n",X1MotoCurrent);
                break;
            case 7:
                sscanf(str,"%d",&value2);
		        X1MotoRunMode=value2;
                ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
                ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);
                printf("X1MotoRunMode=%d\r\n",X1MotoRunMode);
                break;
            case 8:
                sscanf(str,"%d",&value2);
                uhwg_X1ResetOffest=value2;
                ucg_X1MotoModBuf[18]=GET_BYTE1(uhwg_X1ResetOffest);
                ucg_X1MotoModBuf[19]=GET_BYTE0(uhwg_X1ResetOffest);
                printf("uhwg_X1ResetOffest=%d\r\n",uhwg_X1ResetOffest);
            case 13:
                sscanf(str,"%d",&value1);
		        Y1MotoMaxStep=value1;
                ucg_Y1MotoModBuf[0]=GET_BYTE3(Y1MotoMaxStep);
                ucg_Y1MotoModBuf[1]=GET_BYTE2(Y1MotoMaxStep);
                ucg_Y1MotoModBuf[2]=GET_BYTE1(Y1MotoMaxStep);
                ucg_Y1MotoModBuf[3]=GET_BYTE0(Y1MotoMaxStep);
                break;
            case 14:
                sscanf(str,"%d",&value1);
		        Y1MotoSingleStep=value1;
                ucg_Y1MotoModBuf[4]=GET_BYTE3(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[5]=GET_BYTE2(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[6]=GET_BYTE1(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[7]=GET_BYTE0(Y1MotoSingleStep);
                break;
            case 15:
                sscanf(str,"%d",&value2);
		        Y1MotoAcc=value2;
                ucg_Y1MotoModBuf[8]=GET_BYTE1(Y1MotoAcc);
                ucg_Y1MotoModBuf[9]=GET_BYTE0(Y1MotoAcc);
                break;
            case 16:
                sscanf(str,"%d",&value2);
		        Y1MotoDec=value2;
                ucg_Y1MotoModBuf[10]=GET_BYTE1(Y1MotoDec);
                ucg_Y1MotoModBuf[11]=GET_BYTE0(Y1MotoDec);
                break;
            case 17:
                sscanf(str,"%d",&value2);
		        Y1MotoSpeed=value2;
                ucg_Y1MotoModBuf[12]=GET_BYTE1(Y1MotoSpeed);
                ucg_Y1MotoModBuf[13]=GET_BYTE0(Y1MotoSpeed);
                break;
            case 18:
                sscanf(str,"%d",&value2);
		        Y1MotoCurrent=value2;
                ucg_Y1MotoModBuf[14]=GET_BYTE1(Y1MotoCurrent);
                ucg_Y1MotoModBuf[15]=GET_BYTE0(Y1MotoCurrent);
                break;
            case 19:
                sscanf(str,"%d",&value2);
		        Y1MotoRunMode=value2;
                ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
                ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);
                break;
            case 20:
                sscanf(str,"%d",&value2);
                uhwg_Y1ResetOffest=value2;
                ucg_Y1MotoModBuf[18]=GET_BYTE1(uhwg_Y1ResetOffest);
                ucg_Y1MotoModBuf[19]=GET_BYTE0(uhwg_Y1ResetOffest); 
            case 25:
                sscanf(str,"%d",&value1);
		        Z1MotoMaxStep=value1;
                ucg_Z1MotoModBuf[0]=GET_BYTE3(Z1MotoMaxStep);
                ucg_Z1MotoModBuf[1]=GET_BYTE2(Z1MotoMaxStep);
                ucg_Z1MotoModBuf[2]=GET_BYTE1(Z1MotoMaxStep);
                ucg_Z1MotoModBuf[3]=GET_BYTE0(Z1MotoMaxStep);
                break;
            case 26:
                sscanf(str,"%d",&value1);
		        Z1MotoSingleStep=value1;
                ucg_Z1MotoModBuf[4]=GET_BYTE3(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[5]=GET_BYTE2(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[6]=GET_BYTE1(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[7]=GET_BYTE0(Z1MotoSingleStep);
                break;
            case 27:
                sscanf(str,"%d",&value2);
		        Z1MotoAcc=value2;
                ucg_Z1MotoModBuf[8]=GET_BYTE1(Z1MotoAcc);
                ucg_Z1MotoModBuf[9]=GET_BYTE0(Z1MotoAcc);
                break;
            case 28:
                sscanf(str,"%d",&value2);
		        Z1MotoDec=value2;
                ucg_Z1MotoModBuf[10]=GET_BYTE1(Z1MotoDec);
                ucg_Z1MotoModBuf[11]=GET_BYTE0(Z1MotoDec);
                break;
            case 29:
                sscanf(str,"%d",&value2);
		        Z1MotoSpeed=value2;
                ucg_Z1MotoModBuf[12]=GET_BYTE1(Z1MotoSpeed);
                ucg_Z1MotoModBuf[13]=GET_BYTE0(Z1MotoSpeed);
                break;
            case 30:
                sscanf(str,"%d",&value2);
		        Z1MotoCurrent=value2;
                ucg_Z1MotoModBuf[14]=GET_BYTE1(Z1MotoCurrent);
                ucg_Z1MotoModBuf[15]=GET_BYTE0(Z1MotoCurrent);
                break;
            case 31:
                sscanf(str,"%d",&value2);
		        Z1MotoRunMode=value2;
                ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
                ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);
                break; 
            case 32:
                sscanf(str,"%d",&value2);
                uhwg_Z1ResetOffest=value2;
                ucg_Z1MotoModBuf[18]=GET_BYTE1(uhwg_Z1ResetOffest);
                ucg_Z1MotoModBuf[19]=GET_BYTE0(uhwg_Z1ResetOffest);
            case 37:
                sscanf(str,"%d",&value1);
		        A1MotoMaxStep=value1;
                ucg_A1MotoModBuf[0]=GET_BYTE3(A1MotoMaxStep);
                ucg_A1MotoModBuf[1]=GET_BYTE2(A1MotoMaxStep);
                ucg_A1MotoModBuf[2]=GET_BYTE1(A1MotoMaxStep);
                ucg_A1MotoModBuf[3]=GET_BYTE0(A1MotoMaxStep);
                break;
            case 38:
                sscanf(str,"%d",&value1);
		        A1MotoSingleStep=value1;
                ucg_A1MotoModBuf[4]=GET_BYTE3(A1MotoSingleStep);
                ucg_A1MotoModBuf[5]=GET_BYTE2(A1MotoSingleStep);
                ucg_A1MotoModBuf[6]=GET_BYTE1(A1MotoSingleStep);
                ucg_A1MotoModBuf[7]=GET_BYTE0(A1MotoSingleStep);
                break;
            case 39:
                sscanf(str,"%d",&value2);
		        A1MotoAcc=value2;
                ucg_A1MotoModBuf[8]=GET_BYTE1(A1MotoAcc);
                ucg_A1MotoModBuf[9]=GET_BYTE0(A1MotoAcc);
                break;
            case 40:
                sscanf(str,"%d",&value2);
		        A1MotoDec=value2;
                ucg_A1MotoModBuf[10]=GET_BYTE1(A1MotoDec);
                ucg_A1MotoModBuf[11]=GET_BYTE0(A1MotoDec);
                break;
            case 41:
                sscanf(str,"%d",&value2);
		        A1MotoSpeed=value2;
                ucg_A1MotoModBuf[12]=GET_BYTE1(A1MotoSpeed);
                ucg_A1MotoModBuf[13]=GET_BYTE0(A1MotoSpeed);
                break;
            case 42:
                sscanf(str,"%d",&value2);
		        A1MotoCurrent=value2;
                ucg_A1MotoModBuf[14]=GET_BYTE1(A1MotoCurrent);
                ucg_A1MotoModBuf[15]=GET_BYTE0(A1MotoCurrent);
                break;
            case 43:
                sscanf(str,"%d",&value2);
		        A1MotoRunMode=value2;
                ucg_A1MotoModBuf[16]=GET_BYTE1(A1MotoRunMode);
                ucg_A1MotoModBuf[17]=GET_BYTE0(A1MotoRunMode);
                break; 
            case 44:
                sscanf(str,"%d",&value2);
                uhwg_A1ResetOffest=value2;
                ucg_A1MotoModBuf[18]=GET_BYTE1(uhwg_A1ResetOffest);
                ucg_A1MotoModBuf[19]=GET_BYTE0(uhwg_A1ResetOffest);
            default:
                break;
        }
    }
    if(screen_id==0)                                                             
    {
        switch (control_id)
        {
            case 2:
                sscanf(str,"%d",&value1);
		        X1MotoSingleStep=value1;
                ucg_X1MotoModBuf[4]=GET_BYTE3(X1MotoSingleStep);
                ucg_X1MotoModBuf[5]=GET_BYTE2(X1MotoSingleStep);
                ucg_X1MotoModBuf[6]=GET_BYTE1(X1MotoSingleStep);
                ucg_X1MotoModBuf[7]=GET_BYTE0(X1MotoSingleStep);
                break;
            case 3:
                sscanf(str,"%d",&value1);
		        Y1MotoSingleStep=value1;
                ucg_Y1MotoModBuf[4]=GET_BYTE3(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[5]=GET_BYTE2(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[6]=GET_BYTE1(Y1MotoSingleStep);
                ucg_Y1MotoModBuf[7]=GET_BYTE0(Y1MotoSingleStep);
                break;
            case 4:
                sscanf(str,"%d",&value1);
		        Z1MotoSingleStep=value1;
                ucg_Z1MotoModBuf[4]=GET_BYTE3(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[5]=GET_BYTE2(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[6]=GET_BYTE1(Z1MotoSingleStep);
                ucg_Z1MotoModBuf[7]=GET_BYTE0(Z1MotoSingleStep);
                break;
        }
    }
    if(screen_id==3)
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
    if(screen_id==4)                                                             
    {
        switch (control_id)
        {
            case 1:
                sscanf(str,"%d",&value1);
		        X2MotoMaxStep=value1;
                ucg_X2MotoModBuf[0]=GET_BYTE3(X2MotoMaxStep);
                ucg_X2MotoModBuf[1]=GET_BYTE2(X2MotoMaxStep);
                ucg_X2MotoModBuf[2]=GET_BYTE1(X2MotoMaxStep);
                ucg_X2MotoModBuf[3]=GET_BYTE0(X2MotoMaxStep);
                break;
            case 2:
                sscanf(str,"%d",&value1);
		        X2MotoSingleStep=value1;
                ucg_X2MotoModBuf[4]=GET_BYTE3(X2MotoSingleStep);
                ucg_X2MotoModBuf[5]=GET_BYTE2(X2MotoSingleStep);
                ucg_X2MotoModBuf[6]=GET_BYTE1(X2MotoSingleStep);
                ucg_X2MotoModBuf[7]=GET_BYTE0(X2MotoSingleStep);
                break;
            case 3:
                sscanf(str,"%d",&value2);
		        X2MotoAcc=value2;
                ucg_X2MotoModBuf[8]=GET_BYTE1(X2MotoAcc);
                ucg_X2MotoModBuf[9]=GET_BYTE0(X2MotoAcc);
                break;
            case 4:
                sscanf(str,"%d",&value2);
		        X2MotoDec=value2;
                ucg_X2MotoModBuf[10]=GET_BYTE1(X2MotoDec);
                ucg_X2MotoModBuf[11]=GET_BYTE0(X2MotoDec);
                break;
            case 5:
                sscanf(str,"%d",&value2);
		        X2MotoSpeed=value2;
                ucg_X2MotoModBuf[12]=GET_BYTE1(X2MotoSpeed);
                ucg_X2MotoModBuf[13]=GET_BYTE0(X2MotoSpeed);
                break; 
            case 6:
                sscanf(str,"%d",&value2);
		        X2MotoCurrent=value2;
                ucg_X2MotoModBuf[14]=GET_BYTE1(X2MotoCurrent);
                ucg_X2MotoModBuf[15]=GET_BYTE0(X2MotoCurrent);
                break;
            case 7:
                sscanf(str,"%d",&value2);
		        X2MotoRunMode=value2;
                ucg_X2MotoModBuf[16]=GET_BYTE1(X2MotoRunMode);
                ucg_X2MotoModBuf[17]=GET_BYTE0(X2MotoRunMode);
                break;
            case 8:
                sscanf(str,"%d",&value2);
                uhwg_X2ResetOffest=value2;
                ucg_X2MotoModBuf[18]=GET_BYTE1(uhwg_X2ResetOffest);
                ucg_X2MotoModBuf[19]=GET_BYTE0(uhwg_X2ResetOffest);
            case 13:
                sscanf(str,"%d",&value1);
		        Y2MotoMaxStep=value1;
                ucg_Y2MotoModBuf[0]=GET_BYTE3(Y2MotoMaxStep);
                ucg_Y2MotoModBuf[1]=GET_BYTE2(Y2MotoMaxStep);
                ucg_Y2MotoModBuf[2]=GET_BYTE1(Y2MotoMaxStep);
                ucg_Y2MotoModBuf[3]=GET_BYTE0(Y2MotoMaxStep);
                break;
            case 14:
                sscanf(str,"%d",&value1);
		        Y2MotoSingleStep=value1;
                ucg_Y2MotoModBuf[4]=GET_BYTE3(Y2MotoSingleStep);
                ucg_Y2MotoModBuf[5]=GET_BYTE2(Y2MotoSingleStep);
                ucg_Y2MotoModBuf[6]=GET_BYTE1(Y2MotoSingleStep);
                ucg_Y2MotoModBuf[7]=GET_BYTE0(Y2MotoSingleStep);
                break;
            case 15:
                sscanf(str,"%d",&value2);
		        Y2MotoAcc=value2;
                ucg_Y2MotoModBuf[8]=GET_BYTE1(Y2MotoAcc);
                ucg_Y2MotoModBuf[9]=GET_BYTE0(Y2MotoAcc);
                break;
            case 16:
                sscanf(str,"%d",&value2);
		        Y2MotoDec=value2;
                ucg_Y2MotoModBuf[10]=GET_BYTE1(Y2MotoDec);
                ucg_Y2MotoModBuf[11]=GET_BYTE0(Y2MotoDec);
                break;
            case 17:
                sscanf(str,"%d",&value2);
		        Y2MotoSpeed=value2;
                ucg_Y2MotoModBuf[12]=GET_BYTE1(Y2MotoSpeed);
                ucg_Y2MotoModBuf[13]=GET_BYTE0(Y2MotoSpeed);
                break;
            case 18:
                sscanf(str,"%d",&value2);
		        Y2MotoCurrent=value2;
                ucg_Y2MotoModBuf[14]=GET_BYTE1(Y2MotoCurrent);
                ucg_Y2MotoModBuf[15]=GET_BYTE0(Y2MotoCurrent);
                break;
            case 19:
                sscanf(str,"%d",&value2);
		        Y2MotoRunMode=value2;
                ucg_Y2MotoModBuf[16]=GET_BYTE1(Y2MotoRunMode);
                ucg_Y2MotoModBuf[17]=GET_BYTE0(Y2MotoRunMode);
                break;
            case 20:
                sscanf(str,"%d",&value2);
                uhwg_Y2ResetOffest=value2;
                ucg_Y2MotoModBuf[18]=GET_BYTE1(uhwg_Y2ResetOffest);
                ucg_Y2MotoModBuf[19]=GET_BYTE0(uhwg_Y2ResetOffest); 
            case 25:
                sscanf(str,"%d",&value1);
		        Z2MotoMaxStep=value1;
                ucg_Z2MotoModBuf[0]=GET_BYTE3(Z2MotoMaxStep);
                ucg_Z2MotoModBuf[1]=GET_BYTE2(Z2MotoMaxStep);
                ucg_Z2MotoModBuf[2]=GET_BYTE1(Z2MotoMaxStep);
                ucg_Z2MotoModBuf[3]=GET_BYTE0(Z2MotoMaxStep);
                break;
            case 26:
                sscanf(str,"%d",&value1);
		        Z2MotoSingleStep=value1;
                ucg_Z2MotoModBuf[4]=GET_BYTE3(Z2MotoSingleStep);
                ucg_Z2MotoModBuf[5]=GET_BYTE2(Z2MotoSingleStep);
                ucg_Z2MotoModBuf[6]=GET_BYTE1(Z2MotoSingleStep);
                ucg_Z2MotoModBuf[7]=GET_BYTE0(Z2MotoSingleStep);
                break;
            case 27:
                sscanf(str,"%d",&value2);
		        Z2MotoAcc=value2;
                ucg_Z2MotoModBuf[8]=GET_BYTE1(Z2MotoAcc);
                ucg_Z2MotoModBuf[9]=GET_BYTE0(Z2MotoAcc);
                break;
            case 28:
                sscanf(str,"%d",&value2);
		        Z2MotoDec=value2;
                ucg_Z2MotoModBuf[10]=GET_BYTE1(Z2MotoDec);
                ucg_Z2MotoModBuf[11]=GET_BYTE0(Z2MotoDec);
                break;
            case 29:
                sscanf(str,"%d",&value2);
		        Z2MotoSpeed=value2;
                ucg_Z2MotoModBuf[12]=GET_BYTE1(Z2MotoSpeed);
                ucg_Z2MotoModBuf[13]=GET_BYTE0(Z2MotoSpeed);
                break;
            case 30:
                sscanf(str,"%d",&value2);
		        Z2MotoCurrent=value2;
                ucg_Z2MotoModBuf[14]=GET_BYTE1(Z2MotoCurrent);
                ucg_Z2MotoModBuf[15]=GET_BYTE0(Z2MotoCurrent);
                break;
            case 31:
                sscanf(str,"%d",&value2);
		        Z2MotoRunMode=value2;
                ucg_Z2MotoModBuf[16]=GET_BYTE1(Z2MotoRunMode);
                ucg_Z2MotoModBuf[17]=GET_BYTE0(Z2MotoRunMode);
                break; 
            case 32:
                sscanf(str,"%d",&value2);
                uhwg_Z2ResetOffest=value2;
                ucg_Z2MotoModBuf[18]=GET_BYTE1(uhwg_Z2ResetOffest);
                ucg_Z2MotoModBuf[19]=GET_BYTE0(uhwg_Z2ResetOffest); 
            case 37:
                sscanf(str,"%d",&value1);
		        A2MotoMaxStep=value1;
                ucg_A2MotoModBuf[0]=GET_BYTE3(A2MotoMaxStep);
                ucg_A2MotoModBuf[1]=GET_BYTE2(A2MotoMaxStep);
                ucg_A2MotoModBuf[2]=GET_BYTE1(A2MotoMaxStep);
                ucg_A2MotoModBuf[3]=GET_BYTE0(A2MotoMaxStep);
                break;
            case 38:
                sscanf(str,"%d",&value1);
		        A2MotoSingleStep=value1;
                ucg_A2MotoModBuf[4]=GET_BYTE3(A2MotoSingleStep);
                ucg_A2MotoModBuf[5]=GET_BYTE2(A2MotoSingleStep);
                ucg_A2MotoModBuf[6]=GET_BYTE1(A2MotoSingleStep);
                ucg_A2MotoModBuf[7]=GET_BYTE0(A2MotoSingleStep);
                break;
            case 39:
                sscanf(str,"%d",&value2);
		        A2MotoAcc=value2;
                ucg_A2MotoModBuf[8]=GET_BYTE1(A2MotoAcc);
                ucg_A2MotoModBuf[9]=GET_BYTE0(A2MotoAcc);
                break;
            case 40:
                sscanf(str,"%d",&value2);
		        A2MotoDec=value2;
                ucg_A2MotoModBuf[10]=GET_BYTE1(A2MotoDec);
                ucg_A2MotoModBuf[11]=GET_BYTE0(A2MotoDec);
                break;
            case 41:
                sscanf(str,"%d",&value2);
		        A2MotoSpeed=value2;
                ucg_A2MotoModBuf[12]=GET_BYTE1(A2MotoSpeed);
                ucg_A2MotoModBuf[13]=GET_BYTE0(A2MotoSpeed);
                break;
            case 42:
                sscanf(str,"%d",&value2);
		        A2MotoCurrent=value2;
                ucg_A2MotoModBuf[14]=GET_BYTE1(A2MotoCurrent);
                ucg_A2MotoModBuf[15]=GET_BYTE0(A2MotoCurrent);
                break;
            case 43:
                sscanf(str,"%d",&value2);
		        A2MotoRunMode=value2;
                ucg_A2MotoModBuf[16]=GET_BYTE1(A2MotoRunMode);
                ucg_A2MotoModBuf[17]=GET_BYTE0(A2MotoRunMode);
                break; 
            case 44:
                sscanf(str,"%d",&value2);
                uhwg_A2ResetOffest=value2;
                ucg_A2MotoModBuf[18]=GET_BYTE1(uhwg_A2ResetOffest);
                ucg_A2MotoModBuf[19]=GET_BYTE0(uhwg_A2ResetOffest); 
            default:
                break;
        }
    }
    if(screen_id==5)                                                             
    {
        switch (control_id)
        {
            case 1:
                sscanf(str,"%d",&value1);
		        X3MotoMaxStep=value1;
                ucg_X3MotoModBuf[0]=GET_BYTE3(X3MotoMaxStep);
                ucg_X3MotoModBuf[1]=GET_BYTE2(X3MotoMaxStep);
                ucg_X3MotoModBuf[2]=GET_BYTE1(X3MotoMaxStep);
                ucg_X3MotoModBuf[3]=GET_BYTE0(X3MotoMaxStep);
                break;
            case 2:
                sscanf(str,"%d",&value1);
		        X3MotoSingleStep=value1;
                ucg_X3MotoModBuf[4]=GET_BYTE3(X3MotoSingleStep);
                ucg_X3MotoModBuf[5]=GET_BYTE2(X3MotoSingleStep);
                ucg_X3MotoModBuf[6]=GET_BYTE1(X3MotoSingleStep);
                ucg_X3MotoModBuf[7]=GET_BYTE0(X3MotoSingleStep);
                break;
            case 3:
                sscanf(str,"%d",&value2);
		        X3MotoAcc=value2;
                ucg_X3MotoModBuf[8]=GET_BYTE1(X3MotoAcc);
                ucg_X3MotoModBuf[9]=GET_BYTE0(X3MotoAcc);
                break;
            case 4:
                sscanf(str,"%d",&value2);
		        X3MotoDec=value2;
                ucg_X3MotoModBuf[10]=GET_BYTE1(X3MotoDec);
                ucg_X3MotoModBuf[11]=GET_BYTE0(X3MotoDec);
                break;
            case 5:
                sscanf(str,"%d",&value2);
		        X3MotoSpeed=value2;
                ucg_X3MotoModBuf[12]=GET_BYTE1(X3MotoSpeed);
                ucg_X3MotoModBuf[13]=GET_BYTE0(X3MotoSpeed);
                break; 
            case 6:
                sscanf(str,"%d",&value2);
		        X3MotoCurrent=value2;
                ucg_X3MotoModBuf[14]=GET_BYTE1(X3MotoCurrent);
                ucg_X3MotoModBuf[15]=GET_BYTE0(X3MotoCurrent);
                break;
            case 7:
                sscanf(str,"%d",&value2);
		        X3MotoRunMode=value2;
                ucg_X3MotoModBuf[16]=GET_BYTE1(X3MotoRunMode);
                ucg_X3MotoModBuf[17]=GET_BYTE0(X3MotoRunMode);
                break;
            case 8:
                sscanf(str,"%d",&value2);
                uhwg_X3ResetOffest=value2;
                ucg_X3MotoModBuf[18]=GET_BYTE1(uhwg_X3ResetOffest);
                ucg_X3MotoModBuf[19]=GET_BYTE0(uhwg_X3ResetOffest); 
            case 13:
                sscanf(str,"%d",&value1);
		        Y3MotoMaxStep=value1;
                ucg_Y3MotoModBuf[0]=GET_BYTE3(Y3MotoMaxStep);
                ucg_Y3MotoModBuf[1]=GET_BYTE2(Y3MotoMaxStep);
                ucg_Y3MotoModBuf[2]=GET_BYTE1(Y3MotoMaxStep);
                ucg_Y3MotoModBuf[3]=GET_BYTE0(Y3MotoMaxStep);
                break;
            case 14:
                sscanf(str,"%d",&value1);
		        Y3MotoSingleStep=value1;
                ucg_Y3MotoModBuf[4]=GET_BYTE3(Y3MotoSingleStep);
                ucg_Y3MotoModBuf[5]=GET_BYTE2(Y3MotoSingleStep);
                ucg_Y3MotoModBuf[6]=GET_BYTE1(Y3MotoSingleStep);
                ucg_Y3MotoModBuf[7]=GET_BYTE0(Y3MotoSingleStep);
                break;
            case 15:
                sscanf(str,"%d",&value2);
		        Y3MotoAcc=value2;
                ucg_Y3MotoModBuf[8]=GET_BYTE1(Y3MotoAcc);
                ucg_Y3MotoModBuf[9]=GET_BYTE0(Y3MotoAcc);
                break;
            case 16:
                sscanf(str,"%d",&value2);
		        Y3MotoDec=value2;
                ucg_Y3MotoModBuf[10]=GET_BYTE1(Y3MotoDec);
                ucg_Y3MotoModBuf[11]=GET_BYTE0(Y3MotoDec);
                break;
            case 17:
                sscanf(str,"%d",&value2);
		        Y3MotoSpeed=value2;
                ucg_Y3MotoModBuf[12]=GET_BYTE1(Y3MotoSpeed);
                ucg_Y3MotoModBuf[13]=GET_BYTE0(Y3MotoSpeed);
                break;
            case 18:
                sscanf(str,"%d",&value2);
		        Y3MotoCurrent=value2;
                ucg_Y3MotoModBuf[14]=GET_BYTE1(Y3MotoCurrent);
                ucg_Y3MotoModBuf[15]=GET_BYTE0(Y3MotoCurrent);
                break;
            case 19:
                sscanf(str,"%d",&value2);
		        Y3MotoRunMode=value2;
                ucg_Y3MotoModBuf[16]=GET_BYTE1(Y3MotoRunMode);
                ucg_Y3MotoModBuf[17]=GET_BYTE0(Y3MotoRunMode);
                break;
            case 20:
                sscanf(str,"%d",&value2);
                uhwg_Y3ResetOffest=value2;
                ucg_Y3MotoModBuf[18]=GET_BYTE1(uhwg_Y3ResetOffest);
                ucg_Y3MotoModBuf[19]=GET_BYTE0(uhwg_Y3ResetOffest);  
            case 25:
                sscanf(str,"%d",&value1);
		        Z3MotoMaxStep=value1;
                ucg_Z3MotoModBuf[0]=GET_BYTE3(Z3MotoMaxStep);
                ucg_Z3MotoModBuf[1]=GET_BYTE2(Z3MotoMaxStep);
                ucg_Z3MotoModBuf[2]=GET_BYTE1(Z3MotoMaxStep);
                ucg_Z3MotoModBuf[3]=GET_BYTE0(Z3MotoMaxStep);
                break;
            case 26:
                sscanf(str,"%d",&value1);
		        Z3MotoSingleStep=value1;
                ucg_Z3MotoModBuf[4]=GET_BYTE3(Z3MotoSingleStep);
                ucg_Z3MotoModBuf[5]=GET_BYTE2(Z3MotoSingleStep);
                ucg_Z3MotoModBuf[6]=GET_BYTE1(Z3MotoSingleStep);
                ucg_Z3MotoModBuf[7]=GET_BYTE0(Z3MotoSingleStep);
                break;
            case 27:
                sscanf(str,"%d",&value2);
		        Z3MotoAcc=value2;
                ucg_Z3MotoModBuf[8]=GET_BYTE1(Z3MotoAcc);
                ucg_Z3MotoModBuf[9]=GET_BYTE0(Z3MotoAcc);
                break;
            case 28:
                sscanf(str,"%d",&value2);
		        Z3MotoDec=value2;
                ucg_Z3MotoModBuf[10]=GET_BYTE1(Z3MotoDec);
                ucg_Z3MotoModBuf[11]=GET_BYTE0(Z3MotoDec);
                break;
            case 29:
                sscanf(str,"%d",&value2);
		        Z3MotoSpeed=value2;
                ucg_Z3MotoModBuf[12]=GET_BYTE1(Z3MotoSpeed);
                ucg_Z3MotoModBuf[13]=GET_BYTE0(Z3MotoSpeed);
                break;
            case 30:
                sscanf(str,"%d",&value2);
		        Z3MotoCurrent=value2;
                ucg_Z3MotoModBuf[14]=GET_BYTE1(Z3MotoCurrent);
                ucg_Z3MotoModBuf[15]=GET_BYTE0(Z3MotoCurrent);
                break;
            case 31:
                sscanf(str,"%d",&value2);
		        Z3MotoRunMode=value2;
                ucg_Z3MotoModBuf[16]=GET_BYTE1(Z3MotoRunMode);
                ucg_Z3MotoModBuf[17]=GET_BYTE0(Z3MotoRunMode);
                break; 
            case 32:
                sscanf(str,"%d",&value2);
                uhwg_Z3ResetOffest=value2;
                ucg_Z3MotoModBuf[18]=GET_BYTE1(uhwg_Z3ResetOffest);
                ucg_Z3MotoModBuf[19]=GET_BYTE0(uhwg_Z3ResetOffest); 
            case 37:
                sscanf(str,"%d",&value1);
		        A3MotoMaxStep=value1;
                ucg_A3MotoModBuf[0]=GET_BYTE3(A3MotoMaxStep);
                ucg_A3MotoModBuf[1]=GET_BYTE2(A3MotoMaxStep);
                ucg_A3MotoModBuf[2]=GET_BYTE1(A3MotoMaxStep);
                ucg_A3MotoModBuf[3]=GET_BYTE0(A3MotoMaxStep);
                break;
            case 38:
                sscanf(str,"%d",&value1);
		        A3MotoSingleStep=value1;
                ucg_A3MotoModBuf[4]=GET_BYTE3(A3MotoSingleStep);
                ucg_A3MotoModBuf[5]=GET_BYTE2(A3MotoSingleStep);
                ucg_A3MotoModBuf[6]=GET_BYTE1(A3MotoSingleStep);
                ucg_A3MotoModBuf[7]=GET_BYTE0(A3MotoSingleStep);
                break;
            case 39:
                sscanf(str,"%d",&value2);
		        A3MotoAcc=value2;
                ucg_A3MotoModBuf[8]=GET_BYTE1(A3MotoAcc);
                ucg_A3MotoModBuf[9]=GET_BYTE0(A3MotoAcc);
                break;
            case 40:
                sscanf(str,"%d",&value2);
		        A3MotoDec=value2;
                ucg_A3MotoModBuf[10]=GET_BYTE1(A3MotoDec);
                ucg_A3MotoModBuf[11]=GET_BYTE0(A3MotoDec);
                break;
            case 41:
                sscanf(str,"%d",&value2);
		        A3MotoSpeed=value2;
                ucg_A3MotoModBuf[12]=GET_BYTE1(A3MotoSpeed);
                ucg_A3MotoModBuf[13]=GET_BYTE0(A3MotoSpeed);
                break;
            case 42:
                sscanf(str,"%d",&value2);
		        A3MotoCurrent=value2;
                ucg_A3MotoModBuf[14]=GET_BYTE1(A3MotoCurrent);
                ucg_A3MotoModBuf[15]=GET_BYTE0(A3MotoCurrent);
                break;
            case 43:
                sscanf(str,"%d",&value2);
		        A3MotoRunMode=value2;
                ucg_A3MotoModBuf[16]=GET_BYTE1(A3MotoRunMode);
                ucg_A3MotoModBuf[17]=GET_BYTE0(A3MotoRunMode);
                break; 
            case 44:
                sscanf(str,"%d",&value2);
                uhwg_A3ResetOffest=value2;
                ucg_A3MotoModBuf[18]=GET_BYTE1(uhwg_A3ResetOffest);
                ucg_A3MotoModBuf[19]=GET_BYTE0(uhwg_A3ResetOffest); 
            default:
                break;
        }
    }
    if(screen_id==6)
    {
        if(control_id>=30&&control_id<=116)
        {
            sscanf(str,"%d",&value2);
            for (i=0;i<30;i++)
            {
                if ((control_id-30)/3==i)
                {
                    if((control_id-30)%3==0)
                    uhwg_MotionPosition_Offest[i][0]=value2;
                    if((control_id-30)%3==1)
                    uhwg_MotionPosition_Offest[i][1]=value2;
                    if((control_id-30)%3==2)
                    uhwg_MotionPosition_Offest[i][2]=value2;
                }
            }
        } 
    }
    if(screen_id==7)
    {
        if(control_id>=30&&control_id<=107)
        {
            sscanf(str,"%d",&value2);
            for (i=0;i<26;i++)
            {
                if ((control_id-30)/3==i)
                {
                    if((control_id-30)%3==0)
                    uhwg_MotionPosition_Offest[i][0]=value2;
                    if((control_id-30)%3==1)
                    uhwg_MotionPosition_Offest[i][1]=value2;
                    if((control_id-30)%3==2)
                    uhwg_MotionPosition_Offest[i][2]=value2;
                }
            }
        } 
    }
    if(screen_id==8)
    {
        sscanf(str,"%d",&value1);
        switch (control_id)
        {
            case 1:
                X1MotoMaxStep=value1;
                break;
            case 2:
                X1MotoSingleStep=value1;
                break;
            case 3:
                X1MotoAcc=value1;
                MODH_WriteParam_06H(1,0x6204,X1MotoAcc);
                break;
            case 4:
                X1MotoDec=value1;
                MODH_WriteParam_06H(1,0x6205,X1MotoDec);
                break;
            case 5:
                X1MotoSpeed=value1;
                MODH_WriteParam_06H(1,0x6203,X1MotoSpeed);
                break;
            case 6:
                X1MotoCurrent=value1;
                break;
            case 7:
                uhwg_X1ResetOffest=value1;
                break;

            case 11:
                Y1MotoMaxStep=value1;
                break;
            case 12:
                Y1MotoSingleStep=value1;
                break;
            case 13:
                Y1MotoAcc=value1;
                MODH_WriteParam_06H(2,0x6204,Y1MotoAcc);
                break;
            case 14:
                Y1MotoDec=value1;
                MODH_WriteParam_06H(2,0x6205,Y1MotoDec);
                break;
            case 15:
                Y1MotoSpeed=value1;
                MODH_WriteParam_06H(2,0x6203,Y1MotoSpeed);
                break;
            case 16:
                Y1MotoCurrent=value1;
                break;
            case 17:
                uhwg_Y1ResetOffest=value1;
                break;

            case 21:
                Z1MotoMaxStep=value1;
                break;
            case 22:
                Z1MotoSingleStep=value1;
                break;
            case 23:
                Z1MotoAcc=value1;
                MODH_WriteParam_06H(3,0x6204,Z1MotoAcc);//设定PR0加速度 ms/Krpm
                break;
            case 24:
                Z1MotoDec=value1;
                MODH_WriteParam_06H(3,0x6205,Z1MotoDec);//设定PR0减速度 ms/Krpm
                break;
            case 25:
                Z1MotoSpeed=value1;
                MODH_WriteParam_06H(3,0x6203,Z1MotoSpeed);//设定PR0速度rpm
                break;
            case 26:
                Z1MotoCurrent=value1;
                break;
            case 27:
                uhwg_Z1ResetOffest=value1;
                break;

            case 40:
                X2MotoMaxStep=value1;
                break;
            case 41:
                X2MotoSingleStep=value1;
                break;
            case 42:
                X2MotoAcc=value1;
                MODH_WriteParam_06H(4,0x6204,X2MotoAcc);
                break;
            case 43:
                X2MotoDec=value1;
                MODH_WriteParam_06H(4,0x6205,X2MotoDec);
                break;
            case 44:
                X2MotoSpeed=value1;
                MODH_WriteParam_06H(4,0x6203,X2MotoSpeed);
                break;
            case 45:
                X2MotoCurrent=value1;
                break;
            case 46:
                uhwg_X2ResetOffest=value1;
                break;

            case 50:
                Y2MotoMaxStep=value1;
                break;
            case 51:
                Y2MotoSingleStep=value1;
                break;
            case 52:
                Y2MotoAcc=value1;
                MODH_WriteParam_06H(5,0x6204,Y2MotoAcc);
                break;
            case 53:
                Y2MotoDec=value1;
                MODH_WriteParam_06H(5,0x6205,Y2MotoDec);
                break;
            case 54:
                Y2MotoSpeed=value1;
                MODH_WriteParam_06H(5,0x6203,Y2MotoSpeed);
                break;
            case 55:
                Y2MotoCurrent=value1;
                break;
            case 56:
                uhwg_Y2ResetOffest=value1;
                break;

            case 60:
                Z2MotoMaxStep=value1;
                break;
            case 61:
                Z2MotoSingleStep=value1;
                break;
            case 62:
                Z2MotoAcc=value1;
                MODH_WriteParam_06H(6,0x6204,Z2MotoAcc);//设定PR0加速度 ms/Krpm
                break;
            case 63:
                Z2MotoDec=value1;
                MODH_WriteParam_06H(6,0x6205,Z2MotoDec);//设定PR0减速度 ms/Krpm
                break;
            case 64:
                Z2MotoSpeed=value1;
                MODH_WriteParam_06H(6,0x6203,Z2MotoSpeed);//设定PR0速度rpm
                break;
            case 65:
                Z2MotoCurrent=value1;
                break;
            case 66:
                uhwg_Z2ResetOffest=value1;
                break;
        }
            
    }
    if(screen_id==9)
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
    if(screen_id==10)
    {
        switch (control_id)
        {
            case 8:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[0]=(u16)(((value3*10+0.5)/10.0)*10);//(value3*10+0.5)/10.0)先对value3进行小数点后一位的四舍五入，温控板协议30.0度需要发送300，因此再*10,最后强制转换为整数
                break;
            case 11:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[1]=(u16)(((value3*10+0.5)/10.0)*10);
                break;
            case 23:
                sscanf(str,"%f",&value3);
                uhwg_SetTemp[2]=(u16)(((value3*10+0.5)/10.0)*10);
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
        }
    }
}                                                                                
/**************************************************************** */

/*!                                                                              
*  \brief  进度条控件通知                                                       
*  \details  调用GetControlValue时，执行此函数                                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)           
{   
}                                                                                

/*!                                                                              
*  \brief  滑动条控件通知                                                       
*  \details  当滑动条改变(或调用GetControlValue)时，执行此函数                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)             
{	
}



/*! 
*  \brief  定时器超时通知处理
*  \param screen_id 画面ID
*  \param control_id 控件ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{
}

/*! 
*  \brief  读取用户FLASH状态返回
*  \param status 0失败，1成功
*  \param _data 返回数据
*  \param length 数据长度
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
        printf("读flash失败");
    }
}

/*! 
*  \brief  写用户FLASH状态返回
*  \param status 0失败，1成功
*/
void NotifyWriteFlash(uint8 status)
{
    if(status==0)
    {
        printf("flash写入失败\r\n");
    }
    if(status==1)
    {
        printf("flash写入成功\r\n");
    }

}
void Him_Init()
{
    u8 j;
    u8 str[20];
    ucg_X1MotoModBuf[0]=GET_BYTE3(X1MotoMaxStep);
    ucg_X1MotoModBuf[1]=GET_BYTE2(X1MotoMaxStep);
    ucg_X1MotoModBuf[2]=GET_BYTE1(X1MotoMaxStep);
    ucg_X1MotoModBuf[3]=GET_BYTE0(X1MotoMaxStep);
    ucg_X1MotoModBuf[4]=GET_BYTE3(X1MotoSingleStep);
    ucg_X1MotoModBuf[5]=GET_BYTE2(X1MotoSingleStep);
    ucg_X1MotoModBuf[6]=GET_BYTE1(X1MotoSingleStep);
    ucg_X1MotoModBuf[7]=GET_BYTE0(X1MotoSingleStep);
    ucg_X1MotoModBuf[8]=GET_BYTE1(X1MotoAcc);
    ucg_X1MotoModBuf[9]=GET_BYTE0(X1MotoAcc);
    ucg_X1MotoModBuf[10]=GET_BYTE1(X1MotoDec);
    ucg_X1MotoModBuf[11]=GET_BYTE0(X1MotoDec);
    ucg_X1MotoModBuf[12]=GET_BYTE1(X1MotoSpeed);
    ucg_X1MotoModBuf[13]=GET_BYTE0(X1MotoSpeed);
    ucg_X1MotoModBuf[14]=GET_BYTE1(X1MotoCurrent);
    ucg_X1MotoModBuf[15]=GET_BYTE0(X1MotoCurrent);
    ucg_X1MotoModBuf[16]=GET_BYTE1(X1MotoRunMode);
    ucg_X1MotoModBuf[17]=GET_BYTE0(X1MotoRunMode);

    ucg_Y1MotoModBuf[0]=GET_BYTE3(Y1MotoMaxStep);
    ucg_Y1MotoModBuf[1]=GET_BYTE2(Y1MotoMaxStep);
    ucg_Y1MotoModBuf[2]=GET_BYTE1(Y1MotoMaxStep);
    ucg_Y1MotoModBuf[3]=GET_BYTE0(Y1MotoMaxStep);
    ucg_Y1MotoModBuf[4]=GET_BYTE3(Y1MotoSingleStep);
    ucg_Y1MotoModBuf[5]=GET_BYTE2(Y1MotoSingleStep);
    ucg_Y1MotoModBuf[6]=GET_BYTE1(Y1MotoSingleStep);
    ucg_Y1MotoModBuf[7]=GET_BYTE0(Y1MotoSingleStep);
    ucg_Y1MotoModBuf[8]=GET_BYTE1(Y1MotoAcc);
    ucg_Y1MotoModBuf[9]=GET_BYTE0(Y1MotoAcc);
    ucg_Y1MotoModBuf[10]=GET_BYTE1(Y1MotoDec);
    ucg_Y1MotoModBuf[11]=GET_BYTE0(Y1MotoDec);
    ucg_Y1MotoModBuf[12]=GET_BYTE1(Y1MotoSpeed);
    ucg_Y1MotoModBuf[13]=GET_BYTE0(Y1MotoSpeed);
    ucg_Y1MotoModBuf[14]=GET_BYTE1(Y1MotoCurrent);
    ucg_Y1MotoModBuf[15]=GET_BYTE0(Y1MotoCurrent);
    ucg_Y1MotoModBuf[16]=GET_BYTE1(Y1MotoRunMode);
    ucg_Y1MotoModBuf[17]=GET_BYTE0(Y1MotoRunMode);

    ucg_Z1MotoModBuf[0]=GET_BYTE3(Z1MotoMaxStep);
    ucg_Z1MotoModBuf[1]=GET_BYTE2(Z1MotoMaxStep);
    ucg_Z1MotoModBuf[2]=GET_BYTE1(Z1MotoMaxStep);
    ucg_Z1MotoModBuf[3]=GET_BYTE0(Z1MotoMaxStep);
    ucg_Z1MotoModBuf[4]=GET_BYTE3(Z1MotoSingleStep);
    ucg_Z1MotoModBuf[5]=GET_BYTE2(Z1MotoSingleStep);
    ucg_Z1MotoModBuf[6]=GET_BYTE1(Z1MotoSingleStep);
    ucg_Z1MotoModBuf[7]=GET_BYTE0(Z1MotoSingleStep);
    ucg_Z1MotoModBuf[8]=GET_BYTE1(Z1MotoAcc);
    ucg_Z1MotoModBuf[9]=GET_BYTE0(Z1MotoAcc);
    ucg_Z1MotoModBuf[10]=GET_BYTE1(Z1MotoDec);
    ucg_Z1MotoModBuf[11]=GET_BYTE0(Z1MotoDec);
    ucg_Z1MotoModBuf[12]=GET_BYTE1(Z1MotoSpeed);
    ucg_Z1MotoModBuf[13]=GET_BYTE0(Z1MotoSpeed);
    ucg_Z1MotoModBuf[14]=GET_BYTE1(Z1MotoCurrent);
    ucg_Z1MotoModBuf[15]=GET_BYTE0(Z1MotoCurrent);
    ucg_Z1MotoModBuf[16]=GET_BYTE1(Z1MotoRunMode);
    ucg_Z1MotoModBuf[17]=GET_BYTE0(Z1MotoRunMode);

    ucg_A1MotoModBuf[0]=GET_BYTE3(A1MotoMaxStep);
    ucg_A1MotoModBuf[1]=GET_BYTE2(A1MotoMaxStep);
    ucg_A1MotoModBuf[2]=GET_BYTE1(A1MotoMaxStep);
    ucg_A1MotoModBuf[3]=GET_BYTE0(A1MotoMaxStep);
    ucg_A1MotoModBuf[4]=GET_BYTE3(A1MotoSingleStep);
    ucg_A1MotoModBuf[5]=GET_BYTE2(A1MotoSingleStep);
    ucg_A1MotoModBuf[6]=GET_BYTE1(A1MotoSingleStep);
    ucg_A1MotoModBuf[7]=GET_BYTE0(A1MotoSingleStep);
    ucg_A1MotoModBuf[8]=GET_BYTE1(A1MotoAcc);
    ucg_A1MotoModBuf[9]=GET_BYTE0(A1MotoAcc);
    ucg_A1MotoModBuf[10]=GET_BYTE1(A1MotoDec);
    ucg_A1MotoModBuf[11]=GET_BYTE0(A1MotoDec);
    ucg_A1MotoModBuf[12]=GET_BYTE1(A1MotoSpeed);
    ucg_A1MotoModBuf[13]=GET_BYTE0(A1MotoSpeed);
    ucg_A1MotoModBuf[14]=GET_BYTE1(A1MotoCurrent);
    ucg_A1MotoModBuf[15]=GET_BYTE0(A1MotoCurrent);
    ucg_A1MotoModBuf[16]=GET_BYTE1(A1MotoRunMode);
    ucg_A1MotoModBuf[17]=GET_BYTE0(A1MotoRunMode);


    ucg_X2MotoModBuf[0]=GET_BYTE3(X2MotoMaxStep);
    ucg_X2MotoModBuf[1]=GET_BYTE2(X2MotoMaxStep);
    ucg_X2MotoModBuf[2]=GET_BYTE1(X2MotoMaxStep);
    ucg_X2MotoModBuf[3]=GET_BYTE0(X2MotoMaxStep);
    ucg_X2MotoModBuf[4]=GET_BYTE3(X2MotoSingleStep);
    ucg_X2MotoModBuf[5]=GET_BYTE2(X2MotoSingleStep);
    ucg_X2MotoModBuf[6]=GET_BYTE1(X2MotoSingleStep);
    ucg_X2MotoModBuf[7]=GET_BYTE0(X2MotoSingleStep);
    ucg_X2MotoModBuf[8]=GET_BYTE1(X2MotoAcc);
    ucg_X2MotoModBuf[9]=GET_BYTE0(X2MotoAcc);
    ucg_X2MotoModBuf[10]=GET_BYTE1(X2MotoDec);
    ucg_X2MotoModBuf[11]=GET_BYTE0(X2MotoDec);
    ucg_X2MotoModBuf[12]=GET_BYTE1(X2MotoSpeed);
    ucg_X2MotoModBuf[13]=GET_BYTE0(X2MotoSpeed);
    ucg_X2MotoModBuf[14]=GET_BYTE1(X2MotoCurrent);
    ucg_X2MotoModBuf[15]=GET_BYTE0(X2MotoCurrent);
    ucg_X2MotoModBuf[16]=GET_BYTE1(X2MotoRunMode);
    ucg_X2MotoModBuf[17]=GET_BYTE0(X2MotoRunMode);

    ucg_Y2MotoModBuf[0]=GET_BYTE3(Y2MotoMaxStep);
    ucg_Y2MotoModBuf[1]=GET_BYTE2(Y2MotoMaxStep);
    ucg_Y2MotoModBuf[2]=GET_BYTE1(Y2MotoMaxStep);
    ucg_Y2MotoModBuf[3]=GET_BYTE0(Y2MotoMaxStep);
    ucg_Y2MotoModBuf[4]=GET_BYTE3(Y2MotoSingleStep);
    ucg_Y2MotoModBuf[5]=GET_BYTE2(Y2MotoSingleStep);
    ucg_Y2MotoModBuf[6]=GET_BYTE1(Y2MotoSingleStep);
    ucg_Y2MotoModBuf[7]=GET_BYTE0(Y2MotoSingleStep);
    ucg_Y2MotoModBuf[8]=GET_BYTE1(Y2MotoAcc);
    ucg_Y2MotoModBuf[9]=GET_BYTE0(Y2MotoAcc);
    ucg_Y2MotoModBuf[10]=GET_BYTE1(Y2MotoDec);
    ucg_Y2MotoModBuf[11]=GET_BYTE0(Y2MotoDec);
    ucg_Y2MotoModBuf[12]=GET_BYTE1(Y2MotoSpeed);
    ucg_Y2MotoModBuf[13]=GET_BYTE0(Y2MotoSpeed);
    ucg_Y2MotoModBuf[14]=GET_BYTE1(Y2MotoCurrent);
    ucg_Y2MotoModBuf[15]=GET_BYTE0(Y2MotoCurrent);
    ucg_Y2MotoModBuf[16]=GET_BYTE1(Y2MotoRunMode);
    ucg_Y2MotoModBuf[17]=GET_BYTE0(Y2MotoRunMode);

    ucg_Z2MotoModBuf[0]=GET_BYTE3(Z2MotoMaxStep);
    ucg_Z2MotoModBuf[1]=GET_BYTE2(Z2MotoMaxStep);
    ucg_Z2MotoModBuf[2]=GET_BYTE1(Z2MotoMaxStep);
    ucg_Z2MotoModBuf[3]=GET_BYTE0(Z2MotoMaxStep);
    ucg_Z2MotoModBuf[4]=GET_BYTE3(Z2MotoSingleStep);
    ucg_Z2MotoModBuf[5]=GET_BYTE2(Z2MotoSingleStep);
    ucg_Z2MotoModBuf[6]=GET_BYTE1(Z2MotoSingleStep);
    ucg_Z2MotoModBuf[7]=GET_BYTE0(Z2MotoSingleStep);
    ucg_Z2MotoModBuf[8]=GET_BYTE1(Z2MotoAcc);
    ucg_Z2MotoModBuf[9]=GET_BYTE0(Z2MotoAcc);
    ucg_Z2MotoModBuf[10]=GET_BYTE1(Z2MotoDec);
    ucg_Z2MotoModBuf[11]=GET_BYTE0(Z2MotoDec);
    ucg_Z2MotoModBuf[12]=GET_BYTE1(Z2MotoSpeed);
    ucg_Z2MotoModBuf[13]=GET_BYTE0(Z2MotoSpeed);
    ucg_Z2MotoModBuf[14]=GET_BYTE1(Z2MotoCurrent);
    ucg_Z2MotoModBuf[15]=GET_BYTE0(Z2MotoCurrent);
    ucg_Z2MotoModBuf[16]=GET_BYTE1(Z2MotoRunMode);
    ucg_Z2MotoModBuf[17]=GET_BYTE0(Z2MotoRunMode);

    ucg_A2MotoModBuf[0]=GET_BYTE3(A2MotoMaxStep);
    ucg_A2MotoModBuf[1]=GET_BYTE2(A2MotoMaxStep);
    ucg_A2MotoModBuf[2]=GET_BYTE1(A2MotoMaxStep);
    ucg_A2MotoModBuf[3]=GET_BYTE0(A2MotoMaxStep);
    ucg_A2MotoModBuf[4]=GET_BYTE3(A2MotoSingleStep);
    ucg_A2MotoModBuf[5]=GET_BYTE2(A2MotoSingleStep);
    ucg_A2MotoModBuf[6]=GET_BYTE1(A2MotoSingleStep);
    ucg_A2MotoModBuf[7]=GET_BYTE0(A2MotoSingleStep);
    ucg_A2MotoModBuf[8]=GET_BYTE1(A2MotoAcc);
    ucg_A2MotoModBuf[9]=GET_BYTE0(A2MotoAcc);
    ucg_A2MotoModBuf[10]=GET_BYTE1(A2MotoDec);
    ucg_A2MotoModBuf[11]=GET_BYTE0(A2MotoDec);
    ucg_A2MotoModBuf[12]=GET_BYTE1(A2MotoSpeed);
    ucg_A2MotoModBuf[13]=GET_BYTE0(A2MotoSpeed);
    ucg_A2MotoModBuf[14]=GET_BYTE1(A2MotoCurrent);
    ucg_A2MotoModBuf[15]=GET_BYTE0(A2MotoCurrent);
    ucg_A2MotoModBuf[16]=GET_BYTE1(A2MotoRunMode);
    ucg_A2MotoModBuf[17]=GET_BYTE0(A2MotoRunMode);
    for (j = 0; j <=220; j++)//将flash中的数据显示到串口屏
	{
        if(j==0)
        {
            sprintf(str,"%d",X1MotoMaxStep);
	        SetTextValue(1,j+1,str);
            sprintf(str,"%d",X1MotoSingleStep);
	        SetTextValue(1,j+2,str);
            sprintf(str,"%d",X1MotoAcc);
	        SetTextValue(1,j+3,str);
            sprintf(str,"%d",X1MotoDec);
	        SetTextValue(1,j+4,str);
            sprintf(str,"%d",X1MotoSpeed);
	        SetTextValue(1,j+5,str);
            sprintf(str,"%d",X1MotoCurrent);
	        SetTextValue(1,j+6,str);
            sprintf(str,"%d",X1MotoRunMode);
	        SetTextValue(1,j+7,str);
            sprintf(str,"%d",uhwg_X1ResetOffest);
	        SetTextValue(1,j+8,str);
        }
        if(j==20)
        {
            sprintf(str,"%d",Y1MotoMaxStep);
	        SetTextValue(1,j-7,str);
            sprintf(str,"%d",Y1MotoSingleStep);
	        SetTextValue(1,j-6,str);
            sprintf(str,"%d",Y1MotoAcc);
	        SetTextValue(1,j-5,str);
            sprintf(str,"%d",Y1MotoDec);
	        SetTextValue(1,j-4,str);
            sprintf(str,"%d",Y1MotoSpeed);
	        SetTextValue(1,j-3,str);
            sprintf(str,"%d",Y1MotoCurrent);
	        SetTextValue(1,j-2,str);
            sprintf(str,"%d",Y1MotoRunMode);
	        SetTextValue(1,j-1,str);
            sprintf(str,"%d",uhwg_Y1ResetOffest);
	        SetTextValue(1,j-0,str);
        }
        if(j==40)
        {
            sprintf(str,"%d",Z1MotoMaxStep);
	        SetTextValue(1,j-15,str);
            sprintf(str,"%d",Z1MotoSingleStep);
	        SetTextValue(1,j-14,str);
            sprintf(str,"%d",Z1MotoAcc);
	        SetTextValue(1,j-13,str);
            sprintf(str,"%d",Z1MotoDec);
	        SetTextValue(1,j-12,str);
            sprintf(str,"%d",Z1MotoSpeed);
	        SetTextValue(1,j-11,str);
            sprintf(str,"%d",Z1MotoCurrent);
	        SetTextValue(1,j-10,str);
            sprintf(str,"%d",Z1MotoRunMode);
	        SetTextValue(1,j-9,str);
            sprintf(str,"%d",uhwg_Z1ResetOffest);
	        SetTextValue(1,j-8,str);

        }
        if(j==60)
        {
            sprintf(str,"%d",A1MotoMaxStep);
	        SetTextValue(1,j-23,str);
            sprintf(str,"%d",A1MotoSingleStep);
	        SetTextValue(1,j-22,str);
            sprintf(str,"%d",A1MotoAcc);
	        SetTextValue(1,j-21,str);
            sprintf(str,"%d",A1MotoDec);
	        SetTextValue(1,j-20,str);
            sprintf(str,"%d",A1MotoSpeed);
	        SetTextValue(1,j-19,str);
            sprintf(str,"%d",A1MotoCurrent);
	        SetTextValue(1,j-18,str);
            sprintf(str,"%d",A1MotoRunMode);
	        SetTextValue(1,j-17,str);
            sprintf(str,"%d",uhwg_A1ResetOffest);
	        SetTextValue(1,j-16,str);
        }
        if(j==80)
        {
            sprintf(str,"%d",X2MotoMaxStep);
	        SetTextValue(4,j-79,str);
            sprintf(str,"%d",X2MotoSingleStep);
	        SetTextValue(4,j-78,str);
            sprintf(str,"%d",X2MotoAcc);
	        SetTextValue(4,j-77,str);
            sprintf(str,"%d",X2MotoDec);
	        SetTextValue(4,j-76,str);
            sprintf(str,"%d",X2MotoSpeed);
	        SetTextValue(4,j-75,str);
            sprintf(str,"%d",X2MotoCurrent);
	        SetTextValue(4,j-74,str);
            sprintf(str,"%d",X2MotoRunMode);
	        SetTextValue(4,j-73,str);
            sprintf(str,"%d",uhwg_X2ResetOffest);
	        SetTextValue(4,j-72,str);
        }
        if(j==100)
        {

            sprintf(str,"%d",Y2MotoMaxStep);
	        SetTextValue(4,j-87,str);
            sprintf(str,"%d",Y2MotoSingleStep);
	        SetTextValue(4,j-86,str);
            sprintf(str,"%d",Y2MotoAcc);
	        SetTextValue(4,j-85,str);
            sprintf(str,"%d",Y2MotoDec);
	        SetTextValue(4,j-84,str);
            sprintf(str,"%d",Y2MotoSpeed);
	        SetTextValue(4,j-83,str);
            sprintf(str,"%d",Y2MotoCurrent);
	        SetTextValue(4,j-82,str);
            sprintf(str,"%d",Y2MotoRunMode);
	        SetTextValue(4,j-81,str);
            sprintf(str,"%d",uhwg_Y2ResetOffest);
	        SetTextValue(4,j-80,str);
        }
        if(j==120)
        {
            sprintf(str,"%d",Z2MotoMaxStep);
	        SetTextValue(4,j-95,str);
            sprintf(str,"%d",Z2MotoSingleStep);
	        SetTextValue(4,j-94,str);
            sprintf(str,"%d",Z2MotoAcc);
	        SetTextValue(4,j-93,str);
            sprintf(str,"%d",Z2MotoDec);
	        SetTextValue(4,j-92,str);
            sprintf(str,"%d",Z2MotoSpeed);
	        SetTextValue(4,j-91,str);
            sprintf(str,"%d",Z2MotoCurrent);
	        SetTextValue(4,j-90,str);
            sprintf(str,"%d",Z2MotoRunMode);
	        SetTextValue(4,j-89,str);
            sprintf(str,"%d",uhwg_Z2ResetOffest);
	        SetTextValue(4,j-88,str);
        }
        if(j==140)
        {
           
            sprintf(str,"%d",A2MotoMaxStep);
	        SetTextValue(4,j-103,str);
            sprintf(str,"%d",A2MotoSingleStep);
	        SetTextValue(4,j-102,str);
            sprintf(str,"%d",A2MotoAcc);
	        SetTextValue(4,j-101,str);
            sprintf(str,"%d",A2MotoDec);
	        SetTextValue(4,j-100,str);
            sprintf(str,"%d",A2MotoSpeed);
	        SetTextValue(4,j-99,str);
            sprintf(str,"%d",A2MotoCurrent);
	        SetTextValue(4,j-98,str);
            sprintf(str,"%d",A2MotoRunMode);
	        SetTextValue(4,j-97,str);
            sprintf(str,"%d",uhwg_A2ResetOffest);
	        SetTextValue(4,j-96,str);
        }
        if(j==160)
        {
        

            sprintf(str,"%d",X3MotoMaxStep);
	        SetTextValue(5,j-159,str);
            sprintf(str,"%d",X3MotoSingleStep);
	        SetTextValue(5,j-158,str);
            sprintf(str,"%d",X3MotoAcc);
	        SetTextValue(5,j-157,str);
            sprintf(str,"%d",X3MotoDec);
	        SetTextValue(5,j-156,str);
            sprintf(str,"%d",X3MotoSpeed);
	        SetTextValue(5,j-155,str);
            sprintf(str,"%d",X3MotoCurrent);
	        SetTextValue(5,j-154,str);
            sprintf(str,"%d",X3MotoRunMode);
	        SetTextValue(5,j-153,str);
            sprintf(str,"%d",uhwg_X3ResetOffest);
	        SetTextValue(5,j-152,str);
        }
        if(j==180)
        {
            
            sprintf(str,"%d",Y3MotoMaxStep);
	        SetTextValue(5,j-167,str);
            sprintf(str,"%d",Y3MotoSingleStep);
	        SetTextValue(5,j-166,str);
            sprintf(str,"%d",Y3MotoAcc);
	        SetTextValue(5,j-165,str);
            sprintf(str,"%d",Y3MotoDec);
	        SetTextValue(5,j-164,str);
            sprintf(str,"%d",Y3MotoSpeed);
	        SetTextValue(5,j-163,str);
            sprintf(str,"%d",Y3MotoCurrent);
	        SetTextValue(5,j-162,str);
            sprintf(str,"%d",Y3MotoRunMode);
	        SetTextValue(5,j-161,str);
            sprintf(str,"%d",uhwg_Y3ResetOffest);
	        SetTextValue(5,j-160,str);
        }
        if(j==200)
        {
            sprintf(str,"%d",Z3MotoMaxStep);
	        SetTextValue(5,j-175,str);
            sprintf(str,"%d",Z3MotoSingleStep);
	        SetTextValue(5,j-174,str);
            sprintf(str,"%d",Z3MotoAcc);
	        SetTextValue(5,j-173,str);
            sprintf(str,"%d",Z3MotoDec);
	        SetTextValue(5,j-172,str);
            sprintf(str,"%d",Z3MotoSpeed);
	        SetTextValue(5,j-171,str);
            sprintf(str,"%d",Z3MotoCurrent);
	        SetTextValue(5,j-170,str);
            sprintf(str,"%d",Z3MotoRunMode);
	        SetTextValue(5,j-169,str);
            sprintf(str,"%d",uhwg_Z3ResetOffest);
	        SetTextValue(5,j-168,str);
        }
        if(j==220)
        {
            sprintf(str,"%d",A3MotoMaxStep);
	        SetTextValue(5,j-183,str);
            sprintf(str,"%d",A3MotoSingleStep);
	        SetTextValue(5,j-182,str);
            sprintf(str,"%d",A3MotoAcc);
	        SetTextValue(5,j-181,str);
            sprintf(str,"%d",A3MotoDec);
	        SetTextValue(5,j-180,str);
            sprintf(str,"%d",A3MotoSpeed);
	        SetTextValue(5,j-179,str);
            sprintf(str,"%d",A3MotoCurrent);
	        SetTextValue(5,j-178,str);
            sprintf(str,"%d",A3MotoRunMode);
	        SetTextValue(5,j-177,str);
            sprintf(str,"%d",uhwg_A3ResetOffest);
	        SetTextValue(5,j-176,str);
        }
    }


    // u8 j=0;
    // u8 str[20]; 
	// ReadUserFlash(2048,240);//0-2047地址存储大彩协议
	// delay_ms(100);//适当延时，使缓冲数组完全接收完从flash中读取的数据
	// while(cmd_buffer[1]!=0x0B)//flash读取成功返回0x0B
	// queue_find_cmd(cmd_buffer,CMD_MAX_SIZE);
    // SetControlVisiable(1,49,0);
    // SetControlVisiable(1,50,0);
    // SetControlVisiable(4,49,0);
    // SetControlVisiable(4,50,0);
    // SetControlVisiable(5,49,0);
    // SetControlVisiable(5,50,0);
    // for (j = 0; j < 240; j++)//将flash中的数据保存到数组
	// {
    //     if(j<=19)
    //     {
    //         ucg_X1MotoModBuf[j]=cmd_buffer[j+2];
    //     }
    //     if(j>=20&&j<=39)
    //     {
    //         ucg_Y1MotoModBuf[j-20]=cmd_buffer[j+2];
    //     }
    //     if(j>=40&&j<=59)
    //     {
    //         ucg_Z1MotoModBuf[j-40]=cmd_buffer[j+2];
    //     }
    //     if(j>=60&&j<=79)
    //     {
    //         ucg_A1MotoModBuf[j-60]=cmd_buffer[j+2];
    //     }
    //     //
    //     if(j>=80&&j<=99)
    //     {
    //         ucg_X2MotoModBuf[j-80]=cmd_buffer[j+2];
    //     }
    //     if(j>=100&&j<=119)
    //     {
    //         ucg_Y2MotoModBuf[j-100]=cmd_buffer[j+2];
    //     }
    //     if(j>=120&&j<=139)
    //     {
    //         ucg_Z2MotoModBuf[j-120]=cmd_buffer[j+2];
    //     }
    //     if(j>=140&&j<=159)
    //     {
    //         ucg_A2MotoModBuf[j-140]=cmd_buffer[j+2];
    //     }
    //     //
    //     if(j>=160&&j<=179)
    //     {
    //         ucg_X3MotoModBuf[j-160]=cmd_buffer[j+2];
    //     }
    //     if(j>=180&&j<=199)
    //     {
    //         ucg_Y3MotoModBuf[j-180]=cmd_buffer[j+2];
    //     }
    //     if(j>=200&&j<=219)
    //     {
    //         ucg_Z3MotoModBuf[j-200]=cmd_buffer[j+2];
    //     }
    //     if(j>=220&&j<=239)
    //     {
    //         ucg_A3MotoModBuf[j-220]=cmd_buffer[j+2];
    //     }
    // }
    // for (j = 0; j <=220; j++)//将flash中的数据显示到串口屏
	// {
    //     if(j==0)
    //     {
    //         X1MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         X1MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         X1MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         X1MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         X1MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         X1MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         X1MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_X1ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",X1MotoMaxStep);
	//         SetTextValue(1,j+1,str);
    //         sprintf(str,"%d",X1MotoSingleStep);
	//         SetTextValue(1,j+2,str);
    //         sprintf(str,"%d",X1MotoAcc);
	//         SetTextValue(1,j+3,str);
    //         sprintf(str,"%d",X1MotoDec);
	//         SetTextValue(1,j+4,str);
    //         sprintf(str,"%d",X1MotoSpeed);
	//         SetTextValue(1,j+5,str);
    //         sprintf(str,"%d",X1MotoCurrent);
	//         SetTextValue(1,j+6,str);
    //         sprintf(str,"%d",X1MotoRunMode);
	//         SetTextValue(1,j+7,str);
    //         sprintf(str,"%d",uhwg_X1ResetOffest);
	//         SetTextValue(1,j+8,str);
    //     }
    //     if(j==20)
    //     {
    //         Y1MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Y1MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Y1MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Y1MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Y1MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Y1MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Y1MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Y1ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",Y1MotoMaxStep);
	//         SetTextValue(1,j-7,str);
    //         sprintf(str,"%d",Y1MotoSingleStep);
	//         SetTextValue(1,j-6,str);
    //         sprintf(str,"%d",Y1MotoAcc);
	//         SetTextValue(1,j-5,str);
    //         sprintf(str,"%d",Y1MotoDec);
	//         SetTextValue(1,j-4,str);
    //         sprintf(str,"%d",Y1MotoSpeed);
	//         SetTextValue(1,j-3,str);
    //         sprintf(str,"%d",Y1MotoCurrent);
	//         SetTextValue(1,j-2,str);
    //         sprintf(str,"%d",Y1MotoRunMode);
	//         SetTextValue(1,j-1,str);
    //         sprintf(str,"%d",uhwg_Y1ResetOffest);
	//         SetTextValue(1,j-0,str);
    //     }
    //     if(j==40)
    //     {
    //         Z1MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Z1MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Z1MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Z1MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Z1MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Z1MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Z1MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Z1ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",Z1MotoMaxStep);
	//         SetTextValue(1,j-15,str);
    //         sprintf(str,"%d",Z1MotoSingleStep);
	//         SetTextValue(1,j-14,str);
    //         sprintf(str,"%d",Z1MotoAcc);
	//         SetTextValue(1,j-13,str);
    //         sprintf(str,"%d",Z1MotoDec);
	//         SetTextValue(1,j-12,str);
    //         sprintf(str,"%d",Z1MotoSpeed);
	//         SetTextValue(1,j-11,str);
    //         sprintf(str,"%d",Z1MotoCurrent);
	//         SetTextValue(1,j-10,str);
    //         sprintf(str,"%d",Z1MotoRunMode);
	//         SetTextValue(1,j-9,str);
    //         sprintf(str,"%d",uhwg_Z1ResetOffest);
	//         SetTextValue(1,j-8,str);
    //     }
    //     if(j==60)
    //     {
    //         A1MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         A1MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         A1MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         A1MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         A1MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         A1MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         A1MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_A1ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",A1MotoMaxStep);
	//         SetTextValue(1,j-23,str);
    //         sprintf(str,"%d",A1MotoSingleStep);
	//         SetTextValue(1,j-22,str);
    //         sprintf(str,"%d",A1MotoAcc);
	//         SetTextValue(1,j-21,str);
    //         sprintf(str,"%d",A1MotoDec);
	//         SetTextValue(1,j-20,str);
    //         sprintf(str,"%d",A1MotoSpeed);
	//         SetTextValue(1,j-19,str);
    //         sprintf(str,"%d",A1MotoCurrent);
	//         SetTextValue(1,j-18,str);
    //         sprintf(str,"%d",A1MotoRunMode);
	//         SetTextValue(1,j-17,str);
    //         sprintf(str,"%d",uhwg_A1ResetOffest);
	//         SetTextValue(1,j-16,str);
    //     }
    //     if(j==80)
    //     {
    //         X2MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         X2MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         X2MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         X2MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         X2MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         X2MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         X2MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_X2ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",X2MotoMaxStep);
	//         SetTextValue(1,j-79,str);
    //         sprintf(str,"%d",X2MotoSingleStep);
	//         SetTextValue(1,j-78,str);
    //         sprintf(str,"%d",X2MotoAcc);
	//         SetTextValue(1,j-77,str);
    //         sprintf(str,"%d",X2MotoDec);
	//         SetTextValue(1,j-76,str);
    //         sprintf(str,"%d",X2MotoSpeed);
	//         SetTextValue(1,j-75,str);
    //         sprintf(str,"%d",X2MotoCurrent);
	//         SetTextValue(1,j-74,str);
    //         sprintf(str,"%d",X2MotoRunMode);
	//         SetTextValue(1,j-73,str);
    //         sprintf(str,"%d",uhwg_X2ResetOffest);
	//         SetTextValue(1,j-72,str);
    //     }
    //     if(j==100)
    //     {
    //         Y2MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Y2MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Y2MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Y2MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Y2MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Y2MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Y2MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Y2ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",Y2MotoMaxStep);
	//         SetTextValue(1,j-87,str);
    //         sprintf(str,"%d",Y2MotoSingleStep);
	//         SetTextValue(1,j-86,str);
    //         sprintf(str,"%d",Y2MotoAcc);
	//         SetTextValue(1,j-85,str);
    //         sprintf(str,"%d",Y2MotoDec);
	//         SetTextValue(1,j-84,str);
    //         sprintf(str,"%d",Y2MotoSpeed);
	//         SetTextValue(1,j-83,str);
    //         sprintf(str,"%d",Y2MotoCurrent);
	//         SetTextValue(1,j-82,str);
    //         sprintf(str,"%d",Y2MotoRunMode);
	//         SetTextValue(1,j-81,str);
    //         sprintf(str,"%d",uhwg_Y2ResetOffest);
	//         SetTextValue(1,j-80,str);
    //     }
    //     if(j==120)
    //     {
    //         Z2MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Z2MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Z2MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Z2MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Z2MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Z2MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Z2MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Z2ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",Z2MotoMaxStep);
	//         SetTextValue(1,j-95,str);
    //         sprintf(str,"%d",Z2MotoSingleStep);
	//         SetTextValue(1,j-94,str);
    //         sprintf(str,"%d",Z2MotoAcc);
	//         SetTextValue(1,j-93,str);
    //         sprintf(str,"%d",Z2MotoDec);
	//         SetTextValue(1,j-92,str);
    //         sprintf(str,"%d",Z2MotoSpeed);
	//         SetTextValue(1,j-91,str);
    //         sprintf(str,"%d",Z2MotoCurrent);
	//         SetTextValue(1,j-90,str);
    //         sprintf(str,"%d",Z2MotoRunMode);
	//         SetTextValue(1,j-89,str);
    //         sprintf(str,"%d",uhwg_Z2ResetOffest);
	//         SetTextValue(1,j-88,str);
    //     }
    //     if(j==140)
    //     {
    //         A2MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         A2MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         A2MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         A2MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         A2MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         A2MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         A2MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_A2ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",A2MotoMaxStep);
	//         SetTextValue(1,j-103,str);
    //         sprintf(str,"%d",A2MotoSingleStep);
	//         SetTextValue(1,j-102,str);
    //         sprintf(str,"%d",A2MotoAcc);
	//         SetTextValue(1,j-101,str);
    //         sprintf(str,"%d",A2MotoDec);
	//         SetTextValue(1,j-100,str);
    //         sprintf(str,"%d",A2MotoSpeed);
	//         SetTextValue(1,j-99,str);
    //         sprintf(str,"%d",A2MotoCurrent);
	//         SetTextValue(1,j-98,str);
    //         sprintf(str,"%d",A2MotoRunMode);
	//         SetTextValue(1,j-97,str);
    //         sprintf(str,"%d",uhwg_A2ResetOffest);
	//         SetTextValue(1,j-96,str);
    //     }
    //     if(j==160)
    //     {
    //         X3MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         X3MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         X3MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         X3MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         X3MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         X3MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         X3MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_X3ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",X3MotoMaxStep);
	//         SetTextValue(1,j-159,str);
    //         sprintf(str,"%d",X3MotoSingleStep);
	//         SetTextValue(1,j-158,str);
    //         sprintf(str,"%d",X3MotoAcc);
	//         SetTextValue(1,j-157,str);
    //         sprintf(str,"%d",X3MotoDec);
	//         SetTextValue(1,j-156,str);
    //         sprintf(str,"%d",X3MotoSpeed);
	//         SetTextValue(1,j-155,str);
    //         sprintf(str,"%d",X3MotoCurrent);
	//         SetTextValue(1,j-154,str);
    //         sprintf(str,"%d",X3MotoRunMode);
	//         SetTextValue(1,j-153,str);
    //         sprintf(str,"%d",uhwg_X3ResetOffest);
	//         SetTextValue(1,j-152,str);
    //     }
    //     if(j==180)
    //     {
    //         Y3MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Y3MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Y3MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Y3MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Y3MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Y3MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Y3MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Y3ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];

    //         sprintf(str,"%d",Y3MotoMaxStep);
	//         SetTextValue(1,j-167,str);
    //         sprintf(str,"%d",Y3MotoSingleStep);
	//         SetTextValue(1,j-166,str);
    //         sprintf(str,"%d",Y3MotoAcc);
	//         SetTextValue(1,j-165,str);
    //         sprintf(str,"%d",Y3MotoDec);
	//         SetTextValue(1,j-164,str);
    //         sprintf(str,"%d",Y3MotoSpeed);
	//         SetTextValue(1,j-163,str);
    //         sprintf(str,"%d",Y3MotoCurrent);
	//         SetTextValue(1,j-162,str);
    //         sprintf(str,"%d",Y3MotoRunMode);
	//         SetTextValue(1,j-161,str);
    //         sprintf(str,"%d",uhwg_Y3ResetOffest);
	//         SetTextValue(1,j-160,str);
    //     }
    //     if(j==200)
    //     {
    //         Z3MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         Z3MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         Z3MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         Z3MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         Z3MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         Z3MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         Z3MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_Z3ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",Z3MotoMaxStep);
	//         SetTextValue(1,j-175,str);
    //         sprintf(str,"%d",Z3MotoSingleStep);
	//         SetTextValue(1,j-174,str);
    //         sprintf(str,"%d",Z3MotoAcc);
	//         SetTextValue(1,j-173,str);
    //         sprintf(str,"%d",Z3MotoDec);
	//         SetTextValue(1,j-172,str);
    //         sprintf(str,"%d",Z3MotoSpeed);
	//         SetTextValue(1,j-171,str);
    //         sprintf(str,"%d",Z3MotoCurrent);
	//         SetTextValue(1,j-170,str);
    //         sprintf(str,"%d",Z3MotoRunMode);
	//         SetTextValue(1,j-169,str);
    //         sprintf(str,"%d",uhwg_Z3ResetOffest);
	//         SetTextValue(1,j-168,str);
    //     }
    //     if(j==220)
    //     {
    //         A3MotoMaxStep=cmd_buffer[j+2]<<24|cmd_buffer[j+3]<<16|cmd_buffer[j+4]<<8|cmd_buffer[j+5];
    //         A3MotoSingleStep=cmd_buffer[j+6]<<24|cmd_buffer[j+7]<<16|cmd_buffer[j+8]<<8|cmd_buffer[j+9];
    //         A3MotoAcc=cmd_buffer[j+10]<<8|cmd_buffer[j+11];
    //         A3MotoDec=cmd_buffer[j+12]<<8|cmd_buffer[j+13];
    //         A3MotoSpeed=cmd_buffer[j+14]<<8|cmd_buffer[j+15];
    //         A3MotoCurrent=cmd_buffer[j+16]<<8|cmd_buffer[j+17];
    //         A3MotoRunMode=cmd_buffer[j+18]<<8|cmd_buffer[j+19];
    //         uhwg_A3ResetOffest=cmd_buffer[j+20]<<8|cmd_buffer[j+21];
    //         sprintf(str,"%d",A3MotoMaxStep);
	//         SetTextValue(1,j-183,str);
    //         sprintf(str,"%d",A3MotoSingleStep);
	//         SetTextValue(1,j-182,str);
    //         sprintf(str,"%d",A3MotoAcc);
	//         SetTextValue(1,j-181,str);
    //         sprintf(str,"%d",A3MotoDec);
	//         SetTextValue(1,j-180,str);
    //         sprintf(str,"%d",A3MotoSpeed);
	//         SetTextValue(1,j-179,str);
    //         sprintf(str,"%d",A3MotoCurrent);
	//         SetTextValue(1,j-178,str);
    //         sprintf(str,"%d",A3MotoRunMode);
	//         SetTextValue(1,j-177,str);
    //         sprintf(str,"%d",uhwg_A3ResetOffest);
	//         SetTextValue(1,j-176,str);
    //     }     
    // }
}