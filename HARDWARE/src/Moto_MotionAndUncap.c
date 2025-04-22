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
        if(ucg_X1MotoRun1Btn==1)//按下X1电机相对运行按钮
        {
            ucg_X1MotoRun1Btn=0;
            MODH_WriteOrReadParam(6,1,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            value1=GET_2BYTE_H(X1MotoSingleStep);
            MODH_WriteOrReadParam(6,1,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            value1=GET_2BYTE_L(X1MotoSingleStep); 
            MODH_WriteOrReadParam(6,1,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
        }
        if(ucg_Y1MotoRun1Btn==1)//按下Y1电机相对运行按钮
        {
            ucg_Y1MotoRun1Btn=0;

            MODH_WriteOrReadParam(6,2,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            value1=GET_2BYTE_H(Y1MotoSingleStep);
            MODH_WriteOrReadParam(6,2,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            value1=GET_2BYTE_L(Y1MotoSingleStep); 
            MODH_WriteOrReadParam(6,2,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
        }
        if(ucg_Z1MotoRun1Btn==1)//按下Z1电机相对运行按钮
        {
            ucg_Z1MotoRun1Btn=0;

            MODH_WriteOrReadParam(6,3,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            value1=GET_2BYTE_H(Z1MotoSingleStep);
            MODH_WriteOrReadParam(6,3,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            value1=GET_2BYTE_L(Z1MotoSingleStep); 
            MODH_WriteOrReadParam(6,3,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            MODH_WriteOrReadParam(6,3,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
        }
        if(ucg_X1MotoRun2Btn==1)//按下X1电机绝对运行按钮
        {
            ucg_X1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,1,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(X1MotoSingleStep);
            MODH_WriteOrReadParam(6,1,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(X1MotoSingleStep); 
            MODH_WriteOrReadParam(6,1,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Y1MotoRun2Btn==1)//按下Y1电机绝对运行按钮
        {
            ucg_Y1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,2,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(Y1MotoSingleStep);
            MODH_WriteOrReadParam(6,2,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Y1MotoSingleStep); 
            MODH_WriteOrReadParam(6,2,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Z1MotoRun2Btn==1)//按下Z1电机绝对运行按钮
        {
            ucg_Z1MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,3,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(Z1MotoSingleStep);
            MODH_WriteOrReadParam(6,3,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Z1MotoSingleStep); 
            MODH_WriteOrReadParam(6,3,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,3,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_X1RstBtn==1)
        {
            ucg_X1RstBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_Y1RstBtn==1)
        {
            ucg_Y1RstBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_Z1RstBtn==1)
        {
            ucg_Z1RstBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_X1StopBtn==1)
        {
            ucg_X1StopBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x6002,0x40,0,NULL,MODH_CmdMutex);//X1急停
            
        }
        if(ucg_Y1StopBtn==1)
        {
            ucg_Y1StopBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y1急停
            
        }
        if(ucg_Z1StopBtn==1)
        {
            ucg_Z1StopBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1急停
            
        }
        if(ucg_X1SaveBtn==1)
        {
            ucg_X1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,1,0x1801,0x2211,0,NULL,MODH_CmdMutex);//X1保存参数
            
        }
        if(ucg_Y1SaveBtn==1)
        {
            ucg_Y1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,2,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Y1保存参数
            
        }
        if(ucg_Z1SaveBtn==1)
        {
            ucg_Z1SaveBtn=0;
            
            MODH_WriteOrReadParam(6,3,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Z1保存参数
            
        }
        
        if(ucg_X2MotoRun1Btn==1)//按下X2电机相对运行按钮
        {
            ucg_X2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,4,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            
            value1=GET_2BYTE_H(X2MotoSingleStep);
            MODH_WriteOrReadParam(6,4,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(X2MotoSingleStep); 
            MODH_WriteOrReadParam(6,4,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,4,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Y2MotoRun1Btn==1)//按下Y2电机相对运行按钮
        {
            ucg_Y2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,5,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            
            value1=GET_2BYTE_H(Y2MotoSingleStep);
            MODH_WriteOrReadParam(6,5,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Y2MotoSingleStep); 
            MODH_WriteOrReadParam(6,5,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,5,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Z2MotoRun1Btn==1)//按下Z2电机相对运行按钮
        {
            ucg_Z2MotoRun1Btn=0;
            
            MODH_WriteOrReadParam(6,6,0x6200,0x41,0,NULL,MODH_CmdMutex);//设定PR0模式为相对模式
            
            value1=GET_2BYTE_H(Z2MotoSingleStep);
            MODH_WriteOrReadParam(6,6,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Z2MotoSingleStep); 
            MODH_WriteOrReadParam(6,6,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,6,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_X2MotoRun2Btn==1)//按下X2电机绝对运行按钮
        {
            ucg_X2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,4,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(X2MotoSingleStep);
            MODH_WriteOrReadParam(6,4,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(X2MotoSingleStep); 
            MODH_WriteOrReadParam(6,4,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,4,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Y2MotoRun2Btn==1)//按下Y2电机绝对运行按钮
        {
            ucg_Y2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,5,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(Y2MotoSingleStep);
            MODH_WriteOrReadParam(6,5,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Y2MotoSingleStep); 
            MODH_WriteOrReadParam(6,5,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,5,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_Z2MotoRun2Btn==1)//按下Z2电机绝对运行按钮
        {
            ucg_Z2MotoRun2Btn=0;
            
            MODH_WriteOrReadParam(6,6,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
            
            value1=GET_2BYTE_H(Z2MotoSingleStep);
            MODH_WriteOrReadParam(6,6,0x6201,value1,0,NULL,MODH_CmdMutex);//设定PR0位置高位
            
            value1=GET_2BYTE_L(Z2MotoSingleStep); 
            MODH_WriteOrReadParam(6,6,0x6202,value1,0,NULL,MODH_CmdMutex);//设定PR0位置低位
            
            MODH_WriteOrReadParam(6,6,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
            
        }
        if(ucg_X2RstBtn==1)
        {
            ucg_X2RstBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_Y2RstBtn==1)
        {
            ucg_Y2RstBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_Z2RstBtn==1)
        {
            ucg_Z2RstBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发回零
            

        }
        if(ucg_X2StopBtn==1)
        {
            ucg_X2StopBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x6002,0x40,0,NULL,MODH_CmdMutex);//X2急停
            
        }
        if(ucg_Y2StopBtn==1)
        {
            ucg_Y2StopBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y2急停
            
        }
        if(ucg_Z2StopBtn==1)
        {
            ucg_Z2StopBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z2急停
            
        }
        if(ucg_X2SaveBtn==1)
        {
            ucg_X2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,4,0x1801,0x2211,0,NULL,MODH_CmdMutex);//X2保存参数
            
        }
        if(ucg_Y2SaveBtn==1)
        {
            ucg_Y2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,5,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Y2保存参数
            
        }
        if(ucg_Z2SaveBtn==1)
        {
            ucg_Z2SaveBtn=0;
            
            MODH_WriteOrReadParam(6,6,0x1801,0x2211,0,NULL,MODH_CmdMutex);//Z2保存参数
            
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
            
            MODH_WriteOrReadParam(6,1,0x6002,0x40,0,NULL,MODH_CmdMutex);//X1急停
            
            MODH_WriteOrReadParam(6,2,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y1急停
            
            MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1急停
            
            MODH_WriteOrReadParam(6,4,0x6002,0x40,0,NULL,MODH_CmdMutex);//X2急停
            
            MODH_WriteOrReadParam(6,5,0x6002,0x40,0,NULL,MODH_CmdMutex);//Y2急停
            
            MODH_WriteOrReadParam(6,6,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z2急停
            
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
*	函 数 名: UpDownBasket
*	功能说明: 控制Z1轴、Y1轴运动取放吊篮
*	形    参: Dir : 1取吊篮，2放吊篮
*             StainingNumber：选中的仓体序号
*			  ShakeWaterFlag：是否开启抖水
*			  millisecond：抖水时间
*	返 回 值: 无

*	返 回 值: 无
*********************************************************************************************************/
void UpDownBasket(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond)
{
    delay_ms(2);//使任务挂起防止饿死低优先级任务
    if(Dir==1)//取吊篮
    {
        /*降下吊臂*/
        if(StainingNumber>=29)//由于仓体序号大于等于29的仓体没有盖子，不需要开盖，所以要等X1Y1Z1MotoMove运动完再进行吊篮取放
        {
            WaitMotoStop(1,LSMotoStatus,2);//等待X1轴完成
            WaitMotoStop(2,LSMotoStatus,2);//等待Y1轴完成
            WaitMotoStop(3,LSMotoStatus,2);//等待Z1轴完成
        }
        MODH_WriteOrReadParam(6,3,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成
        MODH_WriteOrReadParam(6,3,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成

        /*钩住吊篮*/
        MODH_WriteOrReadParam(6,2,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(2,LSMotoStatus,2);//等待Y轴完成

        if (ShakeWaterFlag==1) //是否开启抖水
        {
            ShakeWater(millisecond);//抖毫秒

        }

        /*升起吊篮至负限位*/
        MODH_WriteOrReadParam(6,3,0x6002,0x15,0,NULL,MODH_CmdMutex);  //立即运行PR5
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成
        MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z1回零
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成
        StainingPodStatus[StainingNumber]=0;//仓状态设为空闲
            
    }
    if(Dir==2)//放吊篮
    {
        /*移动到吊篮槽的正上方*/
        if(StainingNumber>=29)//由于仓体序号大于等于29的仓体没有盖子，不需要开盖，所以要等X1Y1Z1MotoMove运动完再进行吊篮取放
        {
            WaitMotoStop(1,LSMotoStatus,2);//等待X1轴完成
            WaitMotoStop(2,LSMotoStatus,2);//等待Y1轴完成
            WaitMotoStop(3,LSMotoStatus,2);//等待Z1轴完成
        }
        MODH_WriteOrReadParam(6,2,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(2,LSMotoStatus,2);//等待Y轴完成
        
        /*放下吊篮至吊篮槽内*/
        MODH_WriteOrReadParam(6,3,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成
        MODH_WriteOrReadParam(6,3,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成

        /*脱钩*/
        MODH_WriteOrReadParam(6,2,0x6002,0x12,0,NULL,MODH_CmdMutex);  //立即运行PR2
        WaitMotoStop(2,LSMotoStatus,2);//等待Y轴完成

        /*升起吊臂*/
        MODH_WriteOrReadParam(6,3,0x6002,0x15,0,NULL,MODH_CmdMutex);  //立即运行PR5
        WaitMotoStop(3,LSMotoStatus,2);//等待Z1轴完成
        MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z1回零
        WaitMotoStop(3,LSMotoStatus,2);//等待z1轴完成
        StainingPodStatus[StainingNumber]=1;
        
    }    
}

/********************************************************************************************************
*	函 数 名: UpDownCap
*	功能说明: 控制X2轴、Y2轴、Z2轴运动开关舱门
*	形    参: Dir : 1开舱门，2关舱门
*			  StainingNumber : 选中的仓体序号 
*	返 回 值: 无
*********************************************************************************************************/
void UpDownCap(u8 Dir,u8 StainingNumber)
{
    delay_ms(2);//使任务挂起防止饿死低优先级任务
    if(Dir==1)//开舱门
    {
        if(StainingNumber>=13&&StainingNumber<29)//前后排染色仓开盖方向不一样，X2往前移动的小短距离方向相反
        {
        /*X2移动到开盖处*/
        MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
        MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
        MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成

        /*Y2移动到盖子悬臂下方*/
        MODH_WriteOrReadParam(6,5,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(5,LSMotoStatus,2);//等待X2轴完成

        /*Z2升起将盖子开口角度顶至垂直*/
        MODH_WriteOrReadParam(6,6,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成

        /*X2往后移动一小段，防止开盖时盖子角度小于90°*/
        MODH_WriteOrReadParam(6,4,0x6002,0x12,0,NULL,MODH_CmdMutex);  //立即运行PR2
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成
        }
        if(StainingNumber<13)//前后排染色仓开盖方向不一样，X2往前移动的小短距离方向相反
        {
        /*X2移动到开盖处*/
        MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
        MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
        MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成

        /*Y2移动到盖子悬臂下方*/
        MODH_WriteOrReadParam(6,5,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(5,LSMotoStatus,2);//等待X2轴完成

        /*Z2升起将盖子开口角度顶至垂直*/
        MODH_WriteOrReadParam(6,6,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成

        /*X2往前移动一小段，防止开盖时盖子角度小于90°*/
        MODH_WriteOrReadParam(6,4,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成
        }
    }
    if(Dir==2)//关舱门
    {
        if(StainingNumber>=13&&StainingNumber<29)//前后排染色仓开盖方向不一样，X2往前移动的小短距离方向相反
        {
        /*X2往前移动一小段，防止Z2下降的时候盖子角度超过90°*/
        MODH_WriteOrReadParam(6,4,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成
        
        /*Z2回零，下降到最低*/
        MODH_WriteOrReadParam(6,6,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1运行到500的位置再回零
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成
        MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z2回零
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成

        /*Y2回零,并移动到Y2轴中间*/
        MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Y2回零
        WaitMotoStop(5,LSMotoStatus,2);//等待Y2轴完成
        }  
        if(StainingNumber<13)//前后排染色仓开盖方向不一样，X2往前移动的小短距离方向相反
        {
        /*X2往后移动一小段，防止开盖时盖子角度小于90°*/
        MODH_WriteOrReadParam(6,4,0x6002,0x12,0,NULL,MODH_CmdMutex);  //立即运行PR2
        WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成
        
        /*Z2回零，下降到最低*/
        MODH_WriteOrReadParam(6,6,0x6002,0x11,0,NULL,MODH_CmdMutex);  //立即运行PR1运行到500的位置再回零
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成
        MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z2回零
        WaitMotoStop(6,LSMotoStatus,2);//等待Z2轴完成

        /*Y2移动到Y2轴中间*/
        MODH_WriteOrReadParam(6,5,0x6002,0x12,0,NULL,MODH_CmdMutex);  //立即运行PR2
        WaitMotoStop(5,LSMotoStatus,2);//等待X2轴完成
        }  
    }
}
/********************************************************************************************************
*	函 数 名: TakeGetSample
*	功能说明: 控制X1、Y1、Z1、X2、Y2、Z2运动将样品从选定的仓体内放入或者取出
*	形    参: Dir : 1取样品，2放样品
*			  StainingNumber : 选中的仓体序号 
*			  ShakeWaterFlag : 是否需要抖水
*			  millisecond : 抖水时间
*	返 回 值: 无
*	返 回 值: 无
*********************************************************************************************************/
void TakeGetSample(u8 Dir,u8 StainingNumber,u8 ShakeWaterFlag,u16 millisecond)
{
    
    X1Y1MotoMove(StainingNumber);
    if(Dir==1)
    {
        if(StainingPodStatus[StainingNumber]==1)//仓内有样品，可以取出
        {
            UpDownCap(1,StainingNumber);//开盖
            UpDownBasket(Dir,StainingNumber,ShakeWaterFlag,millisecond);//取吊篮
            UpDownCap(2,StainingNumber);//关盖
        }
    }
 
    if(Dir==2)
    {
        if(StainingPodStatus[StainingNumber]==0)//仓内没样品，可以放入
        {
            UpDownCap(1,StainingNumber);//开盖
            UpDownBasket(Dir,StainingNumber,ShakeWaterFlag,millisecond);  //放吊篮
            UpDownCap(2,StainingNumber);//关盖
        }
    }
}

/**********************************************************************************************************
*	函 数 名: WaitMotoStop
*	功能说明: 查询电机运行状态，如果正在运行则一直等待
*	形    参: SlaveAddress : 从站地址
*			  reg : 寄存器地址
*			  num : 寄存器个数
*	返 回 值: 无
**********************************************************************************************************/
void WaitMotoStop(u8 SlaveAddress,u16 reg,u16 num)
{
    const TickType_t max_time_ticks = pdMS_TO_TICKS(Moto_OverTime); //超时时间转化tick数
    TickType_t start_time = xTaskGetTickCount();
    uint64_t Last_Time=0;
    uint64_t Now_Time=0;
    Last_Time=millis();
    Now_Time=millis();
    uint16_t crc1;
    u8 i;
    u8 Monitorstatus=0x04;
        /*Monitorstatus
        Bit0=1 故障
        Bit1=1 使能
        Bit2=1 运行
        Bit3=1 无效
        Bit4=1 指令完成
        Bit5=1 路径完成
        Bit6=1 回零完成*/
    MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);

    for (i=0;i<3;i++)//如果第一次接收出错，则重新读取接收    
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

        if ((MotoStatus[SlaveAddress-1]&Monitorstatus)==Monitorstatus)//指令完成状态位为1
        {
            MODH_WriteOrReadParam(3,SlaveAddress,LSMotoStatus,0,2,NULL,MODH_CmdMutex);
            MODH_WriteOrReadParam(3,SlaveAddress,LSMotoLocation,0,2,NULL,MODH_CmdMutex);
            #if 0
            if (SlaveAddress==1)
            {
                printf("X1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==2)
            {
                printf("Y1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==3)
            {
                printf("Z1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==4)
            {
                printf("X2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==7)
            {
                printf("A1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==8)
            {
                printf("A2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            #endif 
        }
        else
        {
            #if 0
            if (SlaveAddress==1)
            {
                printf("X1电机停止\r\n");
            }
            if (SlaveAddress==2)
            {
                printf("Y1电机停止\r\n");
            }
            if (SlaveAddress==3)
            {
                printf("Z1电机停止\r\n"); 
            }
            if (SlaveAddress==4)
            {
                printf("X2电机停止\r\n"); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2电机停止\r\n"); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2电机停止\r\n");
            }
            if (SlaveAddress==7)
            {
                printf("A1电机停止\r\n");
            }
            if (SlaveAddress==8)
            {
                printf("A2电机停止\r\n");
            }
            #endif
            return;
        }
        //Now_Time=millis();
        if(xTaskGetTickCount()-start_time>=Moto_OverTime)
        {
            printf("等待超时\r\n");
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
        Bit0=1 故障
        Bit1=1 使能
        Bit2=1 运行
        Bit3=1 无效
        Bit4=1 指令完成
        Bit5=1 路径完成
        Bit6=1 回零完成*/
    MODH_ReadParam_03H(SlaveAddress,reg,num);
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);

    for (i=0;i<3;i++)//如果第一次接收出错，则重新读取接收    
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

        if ((MotoStatus[SlaveAddress-1]&Monitorstatus)==Monitorstatus)//指令完成状态位为1
        {
            delay_ms(delaytime);
            MODH_ReadParam_03H(SlaveAddress,reg,num);
            delay_ms(delaytime);
            MODH_ReadParam_03H(SlaveAddress,LSMotoLocation,num);
            #if 1
            if (SlaveAddress==1)
            {
                printf("X1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==2)
            {
                printf("Y1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==3)
            {
                printf("Z1电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==4)
            {
                printf("X2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2电机运行，当前位置为：%d\r\n",MotoLocation[SlaveAddress-1]); 
            }
            #endif 
        }
        else
        {
            #if 1
            if (SlaveAddress==1)
            {
                printf("X1电机停止\r\n");
            }
            if (SlaveAddress==2)
            {
                printf("Y1电机停止\r\n");
            }
            if (SlaveAddress==3)
            {
                printf("Z1电机停止\r\n"); 
            }
            if (SlaveAddress==4)
            {
                printf("X2电机停止\r\n"); 
            }
            if (SlaveAddress==5)
            {
                printf("Y2电机停止\r\n"); 
            }
            if (SlaveAddress==6)
            {
                printf("Z2电机停止\r\n");
            }
            #endif
            return;
        }
        Now_Time=millis();
        if(Now_Time-Last_Time>=Moto_OverTime)
        {
            printf("等待超时\r\n");
        }
    }
}

/**********************************************************************************************************
*	函 数 名: WaitMotoStop
*	功能说明: 查询电机位置，如果没到指定位置则一直等待
*	形    参: SlaveAddress : 从站地址
*			  reg : 电机位置寄存器地址
*			  num : 寄存器个数
              Position: 指定位置
              Dir：2 大于指定位置结束等待，1 小于指定位置结束等待
*	返 回 值: 无
**********************************************************************************************************/
void WaitMotoPosition(u8 SlaveAddress,u16 reg,u16 num,int32 Position,u8 Dir)
{
    const TickType_t max_time_ticks = pdMS_TO_TICKS(Moto_OverTime); //超时时间转化tick数
    TickType_t start_time = xTaskGetTickCount(); //获取当前时间
    uint16_t crc1;
    u8 i;

    for (i=0;i<3;i++)//如果第一次接收出错，则重新读取接收    
    {
        if ((g_tModH.RxCount < 4)|(crc1 != 0))
        {
            MODH_WriteOrReadParam(3,SlaveAddress,reg,0,num,NULL,MODH_CmdMutex);
            crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
        }
        else
        break;
    }

    while (xTaskGetTickCount()-start_time<Moto_OverTime)//等待时间最大20s
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
        printf("等待超时\r\n");
    }
}

/********************************************************************************************************
*	函 数 名: XYZMotoMove
*	功能说明: 控制XYZ轴运动，XY轴停止后才运行Z轴，Z轴停止后才结束该函数
*	形    参: StainingNumber : 选中的仓体序号
*	返 回 值: 无
*********************************************************************************************************/
void X1Y1MotoMove(u8 StainingNumber)
{
    MODH_WriteOrReadParam(6,1,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
    MODH_WriteOrReadParam(6,1,0x6201,GET_2BYTE_H(uhwg_MotionPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
    MODH_WriteOrReadParam(6,1,0x6202,GET_2BYTE_L(uhwg_MotionPosition_Compose[StainingNumber][0]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
    MODH_WriteOrReadParam(6,1,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行

    MODH_WriteOrReadParam(6,2,0x6200,0X01,0,NULL,MODH_CmdMutex);//设定PR0模式为绝对模式
    MODH_WriteOrReadParam(6,2,0x6201,GET_2BYTE_H(uhwg_MotionPosition_Compose[StainingNumber][1]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
    MODH_WriteOrReadParam(6,2,0x6202,GET_2BYTE_L(uhwg_MotionPosition_Compose[StainingNumber][1]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
    MODH_WriteOrReadParam(6,2,0x6207,0x10,0,NULL,MODH_CmdMutex);  //立即运行
}
void MotoShakeWaterInit(u16 ZstepShakeinterval,u16 Z1ShakeSpeed,u16 Z1ShakeAcc,u16 Z1ShakeDec)//初始化抖水路径PR3\PR4
{   
    //PR3运行完成后自动跳转至PR4,PR4运行完成后自动跳转至PR3,循环运动直到急停
    u16 Z1ShakeStop=10;//两段运动之间的停顿时间

    MODH_WriteOrReadParam(6,3,0x6218,0X4441,0,NULL,MODH_CmdMutex);//设定PR3模式为相对模式，运行完成后自动跳转至PR4
    MODH_WriteOrReadParam(6,3,0x6219,GET_2BYTE_H(-ZstepShakeinterval),0,NULL,MODH_CmdMutex);//设定PR3位置高位
    MODH_WriteOrReadParam(6,3,0x621A,GET_2BYTE_L(-ZstepShakeinterval),0,NULL,MODH_CmdMutex);//设定PR3位置低位
    MODH_WriteOrReadParam(6,3,0x621B,Z1ShakeSpeed,0,NULL,MODH_CmdMutex);//设定PR3速度 rpm
    MODH_WriteOrReadParam(6,3,0x621C,Z1ShakeAcc,0,NULL,MODH_CmdMutex);//设定PR3加速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x621D,Z1ShakeDec,0,NULL,MODH_CmdMutex);//设定PR3减速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x621E,Z1ShakeStop,0,NULL,MODH_CmdMutex);//设定PR3停顿时间ms

    MODH_WriteOrReadParam(6,3,0x6220,0X4341,0,NULL,MODH_CmdMutex);//设定PR4模式为相对模式,运行完成后自动跳转至PR3
    MODH_WriteOrReadParam(6,3,0x6221,GET_2BYTE_H(ZstepShakeinterval),0,NULL,MODH_CmdMutex);//设定PR4位置高位
    MODH_WriteOrReadParam(6,3,0x6222,GET_2BYTE_L(ZstepShakeinterval),0,NULL,MODH_CmdMutex);//设定PR4位置低位
    MODH_WriteOrReadParam(6,3,0x6223,Z1ShakeSpeed,0,NULL,MODH_CmdMutex);//设定PR4速度 rpm
    MODH_WriteOrReadParam(6,3,0x6224,Z1ShakeAcc,0,NULL,MODH_CmdMutex);//设定PR4加速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6225,Z1ShakeDec,0,NULL,MODH_CmdMutex);//设定PR4减速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6226,Z1ShakeStop,0,NULL,MODH_CmdMutex);//设定PR4停顿时间ms
}
void MotoBasketCapInit()//初始化取放吊篮和开关盖路径
{
    int32 Z1step1=3530;//吊篮降下时的绝对位置，Z2轴正限位3500，为了防止中途掉步，设置大一点使电机能完全到达正限位位置
    int32 Z1step2=100;//吊篮升起时停止的绝对位置
    int32 Z1stepShake=1500;//取吊篮升起然后开始抖动沥水的位置
    int32 Y1Interval=1100;//Y2轴取蓝和放蓝时脱钩的距离
    int32 Z2step1=90000;//开盖时Z2第一步上升的距离
    int32 Y2step1=0;//当仓体序号>=13时，也就是后排染色缸开盖时Y2第一步的距离
    int32 Y2step2=7900;//当仓体序号<13时，也就是前排染色缸开盖时Y2第一步的距离
    int32 Y2step3=3500;//当仓体序号<13时，也就是前排染色缸开盖时Y2第二步的距离
    int32 X2Interval=2200;//X2开关盖时移动的小段距离，防止Z2下降时开盖角度大于90°
    u8 X2Intervalspeed=50;//X2开关盖时移动的小段距离的速度

    #if 1//Y1路径配置
    MODH_WriteOrReadParam(6,2,0x6208,0X41,0,NULL,MODH_CmdMutex);//设定PR1模式为相对模式
    MODH_WriteOrReadParam(6,2,0x6209,GET_2BYTE_H(Y1Interval),0,NULL,MODH_CmdMutex);//设定PR1位置高位
    MODH_WriteOrReadParam(6,2,0x620A,GET_2BYTE_L(Y1Interval),0,NULL,MODH_CmdMutex);//设定PR1位置低位
    MODH_WriteOrReadParam(6,2,0x620B,Y1MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR1速度 rpm
    MODH_WriteOrReadParam(6,2,0x620C,Y1MotoAcc,0,NULL,MODH_CmdMutex);//设定PR1加速度 ms/Krpm
    MODH_WriteOrReadParam(6,2,0x620D,Y1MotoDec,0,NULL,MODH_CmdMutex);//设定PR1减速度 ms/Krpm
    MODH_WriteOrReadParam(6,2,0x620E,10,0,NULL,MODH_CmdMutex);//设定PR1停顿时间ms

    MODH_WriteOrReadParam(6,2,0x6210,0X41,0,NULL,MODH_CmdMutex);//设定PR2模式为相对模式
    MODH_WriteOrReadParam(6,2,0x6211,GET_2BYTE_H(-Y1Interval),0,NULL,MODH_CmdMutex);//设定PR2位置高位
    MODH_WriteOrReadParam(6,2,0x6212,GET_2BYTE_L(-Y1Interval),0,NULL,MODH_CmdMutex);//设定PR2位置低位
    MODH_WriteOrReadParam(6,2,0x6213,Y1MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR2速度 rpm
    MODH_WriteOrReadParam(6,2,0x6214,Y1MotoAcc,0,NULL,MODH_CmdMutex);//设定PR2加速度 ms/Krpm
    MODH_WriteOrReadParam(6,2,0x6215,Y1MotoDec,0,NULL,MODH_CmdMutex);//设定PR2减速度 ms/Krpm
    MODH_WriteOrReadParam(6,2,0x6216,10,0,NULL,MODH_CmdMutex);//设定PR2停顿时间ms

    #endif

    #if 1//Z1路径配置
    MODH_WriteOrReadParam(6,3,0x6208,0X01,0,NULL,MODH_CmdMutex);//设定PR1模式为绝对模式
    MODH_WriteOrReadParam(6,3,0x6209,GET_2BYTE_H(Z1step1),0,NULL,MODH_CmdMutex);//设定PR1位置高位
    MODH_WriteOrReadParam(6,3,0x620A,GET_2BYTE_L(Z1step1),0,NULL,MODH_CmdMutex);//设定PR1位置低位
    MODH_WriteOrReadParam(6,3,0x620B,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR1速度 rpm
    MODH_WriteOrReadParam(6,3,0x620C,1000,0,NULL,MODH_CmdMutex);//设定PR1加速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x620D,1000,0,NULL,MODH_CmdMutex);//设定PR1减速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x620E,10,0,NULL,MODH_CmdMutex);//设定PR1停顿时间ms

    MODH_WriteOrReadParam(6,3,0x6210,0X01,0,NULL,MODH_CmdMutex);//设定PR2模式为绝对模式
    MODH_WriteOrReadParam(6,3,0x6211,GET_2BYTE_H(Z1stepShake),0,NULL,MODH_CmdMutex);//设定PR2位置高位
    MODH_WriteOrReadParam(6,3,0x6212,GET_2BYTE_L(Z1stepShake),0,NULL,MODH_CmdMutex);//设定PR2位置低位
    MODH_WriteOrReadParam(6,3,0x6213,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR2速度 rpm
    MODH_WriteOrReadParam(6,3,0x6214,1000,0,NULL,MODH_CmdMutex);//设定PR2加速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6215,1000,0,NULL,MODH_CmdMutex);//设定PR2减速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x6216,10,0,NULL,MODH_CmdMutex);//设定PR2停顿时间ms

    MODH_WriteOrReadParam(6,3,0x6228,0X01,0,NULL,MODH_CmdMutex);//设定PR5模式为绝对模式
    MODH_WriteOrReadParam(6,3,0x6229,GET_2BYTE_H(Z1step2),0,NULL,MODH_CmdMutex);//设定PR5位置高位
    MODH_WriteOrReadParam(6,3,0x622A,GET_2BYTE_L(Z1step2),0,NULL,MODH_CmdMutex);//设定PR5位置低位
    MODH_WriteOrReadParam(6,3,0x622B,Z1MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR5速度 rpm
    MODH_WriteOrReadParam(6,3,0x622C,1000,0,NULL,MODH_CmdMutex);//设定PR5加速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x622D,1000,0,NULL,MODH_CmdMutex);//设定PR5减速度 ms/Krpm
    MODH_WriteOrReadParam(6,3,0x622E,10,0,NULL,MODH_CmdMutex);//设定PR5停顿时间ms
    #endif

    #if 1//X2路径配置
    MODH_WriteOrReadParam(6,4,0x6208,0X41,0,NULL,MODH_CmdMutex);//设定PR1模式为相对模式
    MODH_WriteOrReadParam(6,4,0x6209,GET_2BYTE_H(X2Interval),0,NULL,MODH_CmdMutex);//设定PR1位置高位
    MODH_WriteOrReadParam(6,4,0x620A,GET_2BYTE_L(X2Interval),0,NULL,MODH_CmdMutex);//设定PR1位置低位
    MODH_WriteOrReadParam(6,4,0x620B,X2Intervalspeed,0,NULL,MODH_CmdMutex);//设定PR1速度 rpm
    MODH_WriteOrReadParam(6,4,0x620C,1000,0,NULL,MODH_CmdMutex);//设定PR1加速度 ms/Krpm
    MODH_WriteOrReadParam(6,4,0x620D,1000,0,NULL,MODH_CmdMutex);//设定PR1减速度 ms/Krpm
    MODH_WriteOrReadParam(6,4,0x620E,10,0,NULL,MODH_CmdMutex);//设定PR1停顿时间ms


    MODH_WriteOrReadParam(6,4,0x6210,0X41,0,NULL,MODH_CmdMutex);//设定PR2模式为相对模式
    MODH_WriteOrReadParam(6,4,0x6211,GET_2BYTE_H(-X2Interval),0,NULL,MODH_CmdMutex);//设定PR2位置高位
    MODH_WriteOrReadParam(6,4,0x6212,GET_2BYTE_L(-X2Interval),0,NULL,MODH_CmdMutex);//设定PR2位置低位
    MODH_WriteOrReadParam(6,4,0x6213,X2Intervalspeed,0,NULL,MODH_CmdMutex);//设定PR2速度 rpm
    MODH_WriteOrReadParam(6,4,0x6214,1000,0,NULL,MODH_CmdMutex);//设定PR2加速度 ms/Krpm
    MODH_WriteOrReadParam(6,4,0x6215,1000,0,NULL,MODH_CmdMutex);//设定PR2减速度 ms/Krpm
    MODH_WriteOrReadParam(6,4,0x6216,10,0,NULL,MODH_CmdMutex);//设定PR2停顿时间ms
    #endif

    #if 1//Y2路径配置
    MODH_WriteOrReadParam(6,5,0x6208,0X01,0,NULL,MODH_CmdMutex);//设定PR1模式为绝对模式
    MODH_WriteOrReadParam(6,5,0x6209,GET_2BYTE_H(Y2step2),0,NULL,MODH_CmdMutex);//设定PR1位置高位
    MODH_WriteOrReadParam(6,5,0x620A,GET_2BYTE_L(Y2step2),0,NULL,MODH_CmdMutex);//设定PR1位置低位
    MODH_WriteOrReadParam(6,5,0x620B,Y2MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR1速度 rpm
    MODH_WriteOrReadParam(6,5,0x620C,Y2MotoAcc,0,NULL,MODH_CmdMutex);//设定PR1加速度 ms/Krpm
    MODH_WriteOrReadParam(6,5,0x620D,Y2MotoDec,0,NULL,MODH_CmdMutex);//设定PR1减速度 ms/Krpm
    MODH_WriteOrReadParam(6,5,0x620E,10,0,NULL,MODH_CmdMutex);//设定PR1停顿时间ms


    MODH_WriteOrReadParam(6,5,0x6210,0X01,0,NULL,MODH_CmdMutex);//设定PR2模式为绝对模式
    MODH_WriteOrReadParam(6,5,0x6211,GET_2BYTE_H(Y2step3),0,NULL,MODH_CmdMutex);//设定PR2位置高位
    MODH_WriteOrReadParam(6,5,0x6212,GET_2BYTE_L(Y2step3),0,NULL,MODH_CmdMutex);//设定PR2位置低位
    MODH_WriteOrReadParam(6,5,0x6213,Y2MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR2速度 rpm
    MODH_WriteOrReadParam(6,5,0x6214,Y2MotoAcc,0,NULL,MODH_CmdMutex);//设定PR2加速度 ms/Krpm
    MODH_WriteOrReadParam(6,5,0x6215,Y2MotoDec,0,NULL,MODH_CmdMutex);//设定PR2减速度 ms/Krpm
    MODH_WriteOrReadParam(6,5,0x6216,10,0,NULL,MODH_CmdMutex);//设定PR2停顿时间ms
    #endif

    #if 1//Z2路径配置
    MODH_WriteOrReadParam(6,6,0x6208,0X01,0,NULL,MODH_CmdMutex);//设定PR1模式为绝对模式
    MODH_WriteOrReadParam(6,6,0x6209,GET_2BYTE_H(500),0,NULL,MODH_CmdMutex);//设定PR1位置高位,如果位置设为0会速度过快会导致过冲撞到限位，所以设定500，之后再以慢速回零
    MODH_WriteOrReadParam(6,6,0x620A,GET_2BYTE_H(500),0,NULL,MODH_CmdMutex);//设定PR1位置低位
    MODH_WriteOrReadParam(6,6,0x620B,Z2MotoSpeed,0,NULL,MODH_CmdMutex);//设定PR1速度 rpm
    MODH_WriteOrReadParam(6,6,0x620C,Z2MotoAcc,0,NULL,MODH_CmdMutex);//设定PR1加速度 ms/Krpm
    MODH_WriteOrReadParam(6,6,0x620D,Z2MotoDec,0,NULL,MODH_CmdMutex);//设定PR1减速度 ms/Krpm
    MODH_WriteOrReadParam(6,6,0x620E,10,0,NULL,MODH_CmdMutex);//设定PR1停顿时间ms
    #endif
}
void ShakeWater(u32 ShakeTime)
{
    //ShakeTime为抖动时间
    u8 delaytime=25;
    //升起吊篮至抖水位置
    MODH_WriteOrReadParam(6,3,0x6002,0x12,0,NULL,MODH_CmdMutex);  //立即运行PR2
    WaitMotoStop(3,LSMotoStatus,2);//等待Z轴完成

    //开始抖动
    MODH_WriteOrReadParam(6,3,0x6002,0x13,0,NULL,MODH_CmdMutex);//立即运行PR3
    vTaskDelay(ShakeTime/portTICK_RATE_MS);
    MODH_WriteOrReadParam(6,3,0x6002,0x40,0,NULL,MODH_CmdMutex);//Z1急停
}
/********************************************************************************************************
*	函 数 名: MODH_WriteOrReadSingleParam
*	功能说明: 发送或者读取Modbus指令
*	形    参: WriteOrRead : 6写一个参数，3读参数,10写多个参数
*	          SlaveAddr : 从机地址
*	          _reg : 寄存器地址
*	          _value : 写入的值
*	          _num : 读取的个数
*             *_buf：写入的数组
*             SemaHandle：互斥信号量句柄
*	返 回 值: 无
*********************************************************************************************************/
void MODH_WriteOrReadParam(uint8_t WriteOrRead, uint8_t SlaveAddr, uint16_t _reg, uint16_t _value,uint16_t _num,uint8_t *_buf,SemaphoreHandle_t SemaHandle)
{
    u8 delaytime=50;
    BaseType_t xReturn = pdFALSE;
    
    xReturn = xSemaphoreTake(SemaHandle,portMAX_DELAY); /* 获取互斥信号量 */
    if (xReturn == pdTRUE)
    {
        
        taskENTER_CRITICAL();
        MODH_CmdMutexOwnership = xTaskGetCurrentTaskHandle();//记录互斥信号所有者，在挂起或删除任务时，释放信号量
        taskEXIT_CRITICAL();
        vTaskDelay(delaytime);//在MODBUS读写函数内延迟而不是在函数外延迟，防止实际任务切换时导致数据帧没有被时间分割，例如任务A使用完MODBUS发数据后，任务调度器切换到任务B，任务B之前的延时等待刚好到了，使用MODBUS发数据，这是数据A和数据B会接在一起，使得数据帧没被分割，导致MODBUS数据帧错误

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
        MODH_CmdMutexOwnership = NULL;  // 记录所有者
        taskEXIT_CRITICAL();
        xSemaphoreGive(SemaHandle); /* 释放互斥信号量 */
        if(ulTaskNotifyTake(pdTRUE, 0)== pdTRUE)//收到MonitorTasks通知，则回复MonitorTasks
        {
            xTaskNotifyGive(MonitorTasks_Handler); // 发送通知
        }
        
        vTaskDelay(10);
    }
    else
    {
        printf("MODH_WriteOrReadSingleParam:获取信号量失败!\r\n");
    }
}
void testliucheng()
{
    for(u8 i=0;i<30;i++)
    {
        StainingPodStatus[29]=1;//假设输入仓有样品
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
void ConvertStep(u8* p,int32 step)//将32位数据按照大端存储到8位的数组
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

    MODH_WriteOrReadParam(6,3,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z1回零
    WaitMotoStop(3,LSMotoStatus,2);//等待Z1轴复位完成
    MODH_WriteOrReadParam(6,1,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发X1回零
    MODH_WriteOrReadParam(6,2,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Y1回零
}
void X2Y2Z2GoHome()
{

    MODH_WriteOrReadParam(6,5,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Y2回零
    WaitMotoStop(5,LSMotoStatus,2);//等待Y2轴复位完成
    MODH_WriteOrReadParam(6,4,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发X2回零
    MODH_WriteOrReadParam(6,6,0x6002,0X20,0,NULL,MODH_CmdMutex);//触发Z2回零

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
    /*0X6006设置正限位高16位，0X6007设置正限位低16位
      0x6008设置负限位高16位，0x6009设置负限位低16位
      0x600A设定回零方式:反向，负限位信号回零，回零移动到指定位置
      0x600B设置零位位置高16位，0x600C设置零位位置低16位
      0x600D设置回零停止位置高16位，0x600E设置回零停止位置低16位
      0x600F设置回零高速，0x6010设置回零低速
      0x6011设置回零加速度，0x6012设置回零减速度
    */
   
    delay_ms(3000);
    //X1电机初始化参数
    u8 buf1[26]={0x00,0x01,0xAD,0xB0,//正限位110000
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x03,0xE8,//回零停止位置1000
               0x00,0x19,0x00,0x0A,//回零高速25，回零低速15
               0x03,0xE8,0x03,0xE8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(1,0x6006,13,buf1);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0191,35);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0003,2);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6203,X1MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6204,X1MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6205,X1MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(1,0x0173,5);//设定到位时位置误差软件消抖延时ms
    delay_ms(delaytime);

    //Y1电机初始化参数
    u8 buf2[26]={0x00,0x00,0x88,0xB8,//正限位35000
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x00,0xc8,//回零停止位置200
               0x00,0x19,0x00,0x0f,//回零高速50，回零低速25
               0x03,0xe8,0x03,0xe8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(2,0x6006,13,buf2);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0191,35);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0003,2);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6203,Y1MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6204,Y1MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6205,Y1MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x0173,5);//设定到位时位置误差软件消抖延时ms
    delay_ms(delaytime);

    //Z1电机初始化参数
    u8 buf3[26]={0x00,0x00,0x1F,0x40,//软件正限位8000，实际行程是3500，为了防止掉步直接一直运行直到正限位传感器的位置停下
               0x00,0x00,0x00,0x00,//软件负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x00,0x00,//回零停止位置0
               0x00,0x08,0x00,0x05,//回零高速8，回零低速5
               0x0B,0xB8,0x0B,0xB8//回零加速度3000，回零减速度3000
              };
    MODH_WriteParam_10H(3,0x6006,13,buf3);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0149,0x25);//DI3配置为正向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0191,16);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x0003,0);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6201,GET_2BYTE_H(200));//设定PR0位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6202,GET_2BYTE_L(200));//设定PR0位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6203,8);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6204,Z1MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6205,Z1MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);

    //X2电机初始化参数
    u8 buf4[26]={0x00,0x01,0x7E,0xD0,//正限位98000
               0xFF,0xFF,0xFB,0xB4,//负限位-1100
               0x00,0x06,//回零方式：反向，原点信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x03,0x20,//回零停止位置800
               0x00,0x19,0x00,0x0A,//回零高速25，回零低速10
               0x03,0xe8,0x03,0xe8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(4,0x6006,13,buf4);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0147,0x27);//DI2配置为原点信号
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0191,35);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0003,2);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6203,X2MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6204,X2MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6205,X2MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x0173,15);//设定到位时位置误差软件消抖延时ms
    delay_ms(delaytime);

    //Y2电机初始化参数
    u8 buf5[26]={0x00,0x00,0x21,0x34,//正限位8500
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x0D,0xAC,//回零停止位置3500
               0x00,0x32,0x00,0x19,//回零高速50，回零低速25
               0x03,0xe8,0x03,0xE8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(5,0x6006,13,buf5);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0191,5);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x0003,0);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6201,0);//设定PR0位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6202,0);//设定PR0位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6203,Y2MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6204,Y2MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6205,Y2MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);

    //Z2电机初始化参数
    u8 buf6[26]={0x00,0x01,0xE8,0x48,//正限位125000
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x00,0x00,//回零停止位置0
               0x00,0xc8,0x00,0x64,//回零高速200，回零低速100
               0x03,0xe8,0x03,0xE8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(6,0x6006,13,buf6);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0191,5);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x0003,0);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6201,GET_2BYTE_H(90000));//设定PR0位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6202,GET_2BYTE_L(90000));//设定PR0位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6203,Z2MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6204,Z2MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6205,Z2MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);

    //A1进仓电机初始化参数
    u8 buf7[26]={0x00,0x0F,0x42,0x40,//正限位1000000
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x00,0x00,//回零停止位置0
               0x00,0xC8,0x00,0x64,//回零高速200，回零低速100
               0x03,0xe8,0x03,0xE8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(7,0x6006,13,buf7);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0191,5);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x0003,0);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6201,GET_2BYTE_H(900000));//设定PR0位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6202,GET_2BYTE_L(900000));//设定PR0位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6203,A1MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6204,400);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6205,400);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6208,0X01);//设定PR1模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6209,GET_2BYTE_H(341881));//设定PR1位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620A,GET_2BYTE_L(341881));//设定PR1位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620B,A1MotoSpeed);//设定PR1速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620C,400);//设定PR1加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620D,1500);//设定PR1减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x620E,600);//设定停顿时间ms
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6210,0X41);//设定PR2模式为相对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6211,GET_2BYTE_H(-17463));//设定PR2位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6212,GET_2BYTE_L(-17463));//设定PR2位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6213,A1MotoSpeed);//设定PR2速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6214,1500);//设定PR2加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6215,1500);//设定PR2减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x6216,600);//设定停顿时间ms
    delay_ms(delaytime);

    //A2出仓电机初始化参数
    u8 buf8[26]={0x00,0x0A,0x75,0x30,//正限位30000
               0x00,0x00,0x00,0x00,//负限位0
               0x00,0x02,//回零方式：反向，负限位信号回零，回零移动到指定位置
               0x00,0x00,0x00,0x00,//零位位置0
               0x00,0x00,0x00,0x00,//回零停止位置0
               0x00,0x14,0x00,0x0A,//回零高速20，回零低速10
               0x03,0xe8,0x03,0xE8//回零加速度1000，回零减速度1000
              };
    MODH_WriteParam_10H(8,0x6006,13,buf8);//设置0x6006-0x6012 Pr参数，每个Pr参数地址对应一个16位寄存器值
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0147,0x26);//DI2配置为负向限位
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x1801,0x1111);//复位当前报警
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0191,8);//设置电机峰值电流，0-70，单位0.1A
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x0003,0);//设置电机开闭环模式，0开环，2闭环
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6200,0X01);//设定PR0模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6201,GET_2BYTE_H(25000));//设定PR0位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6202,GET_2BYTE_L(25000));//设定PR0位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6203,A2MotoSpeed);//设定PR0速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6204,A2MotoAcc);//设定PR0加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6205,A2MotoDec);//设定PR0减速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6206,20);//设定停顿时间ms
    delay_ms(delaytime);

    MODH_WriteParam_06H(8,0x6208,0X01);//设定PR1模式为绝对模式
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6209,GET_2BYTE_H(1000));//设定PR1位置高位
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620A,GET_2BYTE_L(1000));//设定PR1位置低位
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620B,A1MotoSpeed);//设定PR1速度rpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620C,A1MotoAcc);//设定PR1加速度 ms/Krpm
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x620D,A1MotoDec);//设定PR1减速度 ms/Krpm
    delay_ms(delaytime);

    #if 0
    MODH_WriteParam_06H(1,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(3,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(4,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(7,0x1801,0x2211);//保存参数
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x1801,0x2211);//保存参数
    delay_ms(8000);
    #endif

    MODH_WriteParam_06H(7,0x6002,0X20);//触发A1回零
    delay_ms(delaytime);
    MODH_WriteParam_06H(8,0x6002,0X20);//触发A2回零
    delay_ms(delaytime);
    MODH_WriteParam_06H(5,0x6002,0X20);//触发Y2回零
    delay_ms(delaytime);
    WaitMotoStop_WithoutRTOS(5,LSMotoStatus,2);//等待Y2轴复位完成
    MODH_WriteParam_06H(4,0x6002,0X20);//触发X2回零
    delay_ms(delaytime);
    MODH_WriteParam_06H(6,0x6002,0X20);//触发Z2回零
    delay_ms(delaytime);

    MODH_WriteParam_06H(3,0x6002,0X20);//触发Z1回零
    delay_ms(delaytime);
    WaitMotoStop_WithoutRTOS(3,LSMotoStatus,2);//等待Z1轴复位完成
    MODH_WriteParam_06H(1,0x6002,0X20);//触发X1回零
    delay_ms(delaytime);
    MODH_WriteParam_06H(2,0x6002,0X20);//触发Y1回零
    WaitMotoStop_WithoutRTOS(1,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(2,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(3,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(4,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(5,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(6,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(7,LSMotoStatus,2);
    WaitMotoStop_WithoutRTOS(8,LSMotoStatus,2);
}

