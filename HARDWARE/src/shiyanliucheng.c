/***************************************************
 * 1. 管理26个染色缸的协同操作
 * 2. 支持3个实验流程并行运行
 * 3. 动态优先级调度
 * 4. 用户按需启动实验
 ***********************************************/

/*------------------ 头文件包含 ------------------*/
#include "stm32f4xx.h"                  // STM32标准库
#include "FreeRTOS.h"                   // FreeRTOS内核
#include "task.h"                      // 任务管理
#include "semphr.h"                     // 信号量
#include "queue.h"                      // 队列
#include "string.h"                     // 字符串操作
#include "GlobalVariable.h"
#include "shiyanliucheng.h"
#include "Moto_MotionAndUncap.h"
#include "delay.h"
#include "Fuzzy_Pid.h"
TaskHandle_t CheckSYliuchengTask_Handler;
TaskHandle_t ExpEndProcessTask_Handler;
bool task_created_flag[NUM_EXPERIMENTS] = {0}; // 任务创建标记数组
uint64_t SY1_RunningTime[30];//用于存储实验一的每一步步骤的运行时间
uint64_t SY2_RunningTime[30];//用于存储实验二的每一步步骤的运行时间
uint64_t SY3_RunningTime[30];//用于存储实验三的每一步步骤的运行时间

/*----------- 步骤用时函数 ------------------*/
/**
 * @brief 每一步的实验步骤用时统计函数
 * @param pvParams 传入ExperimentCB指针
 */
void SY_StepTime(ExperimentCB *pvParams)
{
    ExperimentCB *cb = pvParams;
    if(cb->exp_id==1)
    {
        SY1_RunningTime[cb->current_step]=cb->runtime;
    }
    else if(cb->exp_id==2)
    {
        SY2_RunningTime[cb->current_step]=cb->runtime;
    }
    else if(cb->exp_id==3)
    {
        SY3_RunningTime[cb->current_step]=cb->runtime;
    }
}


/*------------------ 实验任务函数 ------------------*/
/**
 * @brief 实验流程任务
 * @param pvParams 传入ExperimentCB指针
 */
void ExperimentTask(void *pvParams) {
    ExperimentCB *cb = (ExperimentCB *)pvParams;
    cb->is_running = 1; // 标记任务开始
    StepConfig *step;
    uint64_t fanyingTime;//试剂反应时间
    TickType_t xLastTime;
    TickType_t xBegainTime = xTaskGetTickCount();


    /*如果混匀开关开启*/
    if(ucg_X3Y3Z3A3RunBtn==1&&HYMotoStatus==0)
    {
        HYMotoStatus=1;
        AxisMotors[8].RunMode=HYMode;//永动模式
        AxisMotors[8].MotoModBuf[16]=GET_BYTE1(AxisMotors[8].RunMode);
        AxisMotors[8].MotoModBuf[17]=GET_BYTE0(AxisMotors[8].RunMode);
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//写入X3电机行程，单次步数，加速度，减速度，速度，电流，运行模式
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x0C,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//写入Y3电机行程，单次步数，加速度，减速度，速度，电流，运行模式
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x18,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//写入Z3电机行程，单次步数，加速度，减速度，速度，电流，运行模式
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x24,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//写入A3电机行程，单次步数，加速度，减速度，速度，电流，运行模式
        delay_ms(60);
    }
    
    if(cb->exp_id == 2)
    {
        vTaskDelay(600000 / portTICK_RATE_MS);//第二个任务间隔10分钟
    }
    if(cb->exp_id == 3)
    {
        vTaskDelay(1200000 / portTICK_RATE_MS);//第三个任务间隔20分钟
    }

    // 如果开启烤片，将样品从输入仓放至烤片仓
    for (u8 i = 0; i < 3; i++) 
    {
        u8 checkIdx = i * 2;  // 是否烤片索引：实验1:0, 实验2:2, 实验3:4
        if (kaopian[checkIdx] == 1 && cb->exp_id == (i+1)) 
        {
            u8 bakeTank = 26 + i;  // 烤片仓号，实验1:26, 实验2:27, 实验3:28
            u8 delayIdx = 1 + (i*2); // 烤片时间索引，实验1:1, 实验2:3, 实验3:5
            
            xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);// 申请取样互斥锁
            StainingPodStatus[29] = 1;          // 假设输入仓有样品
            TakeGetSample(1, 29, 0, 0);         // 从输入仓取样
            TakeGetSample(2, bakeTank, 0, 0);   // 放样到烤片仓
            xSemaphoreGive(TakeGetSampleMutex); // 释放取样互斥锁
            
            xLastTime = xTaskGetTickCount(); //记录烤片开始时间
            vTaskDelay((kaopian[delayIdx] * 1000) / portTICK_RATE_MS);
            break;  // 只处理当前实验的烤片任务
        }
        else
        StainingPodStatus[29] = 1;          // 假设输入仓有样品
        

    }
   

    for (cb->current_step = 0; cb->current_step < cb->Vaild_Step_Num; cb->current_step++) 
    {
        
        step = &cb->steps[cb->current_step]; 
        /* 动态调整任务优先级，也就是实验步骤的优先级 */
        vTaskPrioritySet(cb->handle, step->priority);
        /* 申请目标缸资源锁 */
        if (xSemaphoreTake(xVatMutex[TankAndReagenMapping(step->target_vat)], portMAX_DELAY) == pdPASS)//如果缸没被占用 
        {
            if(cb->current_step == 0)//如果是第一步
            {
                xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY); //申请取样互斥锁
                if((step->reaction_time*1000)<=15000)//(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2))//反应时间小于等于机械臂将样品放入试剂后的复位时间+样品出液面的时间
                {
                    /*为什么在不升机械臂的步骤中要先判断下一步中的缸锁是否释放：
                    一旦某个实验流程不升起机械臂不关盖，会一直占用取样锁，
                    其他流程的步骤肯定得不到执行，直到该实验流程的一个或者
                    连续多个不升机械臂的步骤完成，执行了下一个升起机械臂的步骤，
                    才能释放取样锁，其他流程才能执行，如果该流程是连续多个不升
                    机械臂的步骤，一定要先判断下一个步骤的缸锁是否释放，
                    如果下一步的缸锁没有释放，就进行这一步骤，
                    该步骤一直占用取样锁，本该在下一步释放取样锁的，
                    由于下一步骤又一直在等待缸锁，从而释放不了取样锁，
                    其他流程没有取样锁其占用的缸锁便无法释放，从而造成死锁，所有的流程阻塞*/
                    
                    
                    xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//等待下一步的缸锁释放如果被占用的话
                    if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)//如果后一步的步骤时间小于复位时间
                    xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//等待后边第二步的缸锁释放
                    

                    /* 只有当当前实验ID对应的kaopian标志为1时才执行取样操作 */
                    if(kaopian[(cb->exp_id-1)*2] == 1)  // 实验1对应kaopian[0]，实验2对应kaopian[2]，实验3对应kaopian[4]
                    {
                        /* 计算烤片仓编号：实验1→26，实验2→27，实验3→28 */
                        u8 bakeTank = 25 + cb->exp_id;
                        /* 从指定烤片仓取样 */
                        TakeGetSample(1, bakeTank, 0, 0);
                        cb->runtime = (((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ)-ActionTime1;                        
                        SY_StepTime(cb);
                    }
                    else TakeGetSample(1,29,0,0);//否则从输入仓取样


                    TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);//吊臂下降，将吊篮放入试剂缸，不升起吊臂，不关闭盖子
                    xLastTime=xTaskGetTickCount();//记录放样的时刻
                    printf("%s di(1)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                    
                    xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                    if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)
                    xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//释放缸锁
                    
                    if((step->reaction_time*1000)>(ActionTime3+ActionTime4/2))//反应时间大于电机指令响应时间
                    {
                        fanyingTime=(step->reaction_time*1000)-ActionTime3-ActionTime4/2;//延时时间=反应时间-电机响应时间-样品离开液面时间
                        vTaskDelay(fanyingTime/portTICK_RATE_MS);
                    }
                    cb->NoCloseCapFlag=1;//标记未关盖
                }
                else
                {
    
                    /* 只有当当前实验ID对应的kaopian标志为1时才执行取样操作 */
                    if(kaopian[(cb->exp_id-1)*2] == 1)  // 实验1对应kaopian[0]，实验2对应kaopian[2]，实验3对应kaopian[4]
                    {
                        /* 计算烤片仓编号：实验1→26，实验2→27，实验3→28 */
                        u8 bakeTank = 25 + cb->exp_id;
                        /* 从指定烤片仓取样 */
                        TakeGetSample(1, bakeTank, 0, 0);
                        cb->runtime = ((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ;                        
                        SY_StepTime(cb);
                    }
                    else TakeGetSample(1,29,0,0);//否则从输入仓取样

                    TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                    xLastTime=xTaskGetTickCount();//记录放样的时刻
                    printf("%s di(2)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                    fanyingTime=(step->reaction_time*1000)-(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2);
                    xSemaphoreGive(TakeGetSampleMutex); //释放取样互斥锁
                    vTaskDelay(fanyingTime/portTICK_RATE_MS);
                }
            }
            else//第一步之后
            {                    
                if((step->reaction_time*1000)<=15000)//(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2))//反应时间小于等于机械臂将样品放入试剂后的复位时间
                {
                    if(cb->NoCloseCapFlag==1)//如果上一步机械臂没升起
                    {
                        /***********************************/
                        //如果后两步不升机械臂等待后两步的缸体空闲
                        if(cb->current_step<(23-2))
                        {
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//等待下一步的缸锁释放如果被占用的话
                            if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//最多连续3步的不升机械臂的步骤
                        }
                        /***********************************/

                        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),0,Z1Shaketime);//从上一步的缸中取样品
                        cb->runtime=(((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ);//滴答数转毫秒，任务完成用时
                        
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//释放上一步骤的试剂缸锁
                        TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);
                        xLastTime=xTaskGetTickCount();//记录放样的时刻
                        printf("%s (1)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        /***********************************/
                        //释放后两步的缸体锁
                        if(cb->current_step<(23-2))
                        {
                            
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                            if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//释放缸锁
                        }
                        /***********************************/

                        if((step->reaction_time*1000)>ActionTime3+ActionTime4/2)//反应时间大于电机指令响应时间
                        {
                            fanyingTime=(step->reaction_time*1000)-ActionTime3-ActionTime4/2;                               
                            vTaskDelay(fanyingTime/portTICK_RATE_MS);
                        }

                        
                    }
                    else
                    {
                        cb->NoCloseCapFlag=1;

                        /***********************************/
                        //如果后两步不升机械臂等待后两步的缸体空闲
                        if(cb->current_step<(23-2))//实验流程有23步
                        {
                            
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//等待下一步的缸锁释放如果被占用的话
                            if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//最多连续3步的不升机械臂的步骤
                        }
                        /***********************************/

                        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
                        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),0,Z1Shaketime);//从上一步的缸中取样品
                        cb->runtime=(((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ);//滴答数转毫秒，任务完成用时
                        
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//释放上一步骤的试剂缸锁
                        TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);
                        xLastTime=xTaskGetTickCount();//记录放样的时刻
                        printf("%s (2)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        /***********************************/
                        //释放后两步的缸体锁
                        if(cb->current_step<(23-2))
                        {
                            
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                            if(((&cb->steps[cb->current_step+1])->reaction_time*1000)<=ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//释放缸锁
                        }
                        /***********************************/

                        if((step->reaction_time*1000)>(ActionTime3+ActionTime4/2))//反应时间大于电机指令响应时间
                        {
                            fanyingTime=(step->reaction_time*1000)-ActionTime3-ActionTime4/2;                               
                            vTaskDelay(fanyingTime/portTICK_RATE_MS);
                        }

                        
                    }
                    
                }
                else//反应时间大于机械臂将样品放入试剂后的复位时间
                {
                    if(cb->NoCloseCapFlag==1)//如果上一步机械臂没升起
                    {
                        cb->NoCloseCapFlag=0;
                        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),0,Z1Shaketime);//从上一步的缸中取样品                           
                        cb->runtime=((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ;//滴答数转毫秒，任务完成用时
                        
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//释放上一步骤的试剂缸锁
                        TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                        xLastTime=xTaskGetTickCount();//记录放样的时刻
                        printf("%s (3)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        xSemaphoreGive(TakeGetSampleMutex);
                        fanyingTime=(step->reaction_time*1000)-(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2);
                        vTaskDelay(fanyingTime/portTICK_RATE_MS);
                    }
                    else
                    {
                        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
                        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),0,Z1Shaketime);//从上一步的缸中取样品
                        cb->runtime=((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ;//滴答数转毫秒，任务完成用时
                        
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//释放上一步骤的试剂缸锁
                        TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                        xLastTime=xTaskGetTickCount();//记录放样的时刻                        
                        printf("%s (4)当前步骤：%d 反应缸体：%d 反应时间：%d 当前步骤优先级：%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        xSemaphoreGive(TakeGetSampleMutex);
                        fanyingTime=(step->reaction_time*1000)-(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2);    
                        vTaskDelay(fanyingTime/portTICK_RATE_MS);

                    }                        
                }
                
                /*每一步骤反应时间统计*/
                if((&cb->steps[cb->current_step-1])->reaction_time*1000<=15000)//(ActionTime1+ActionTime2+2*ActionTime3+ActionTime4/2)) //如果反应时间小于机械动作必要时间，则样品放入缸后不升起机械臂
                {
                    //此时，统计时间多了取样后的上升关盖时间，需要减去
                    cb->runtime-=(ActionTime5-ActionTime4/2)-ActionTime6;
                    SY_StepTime(cb);   
                }
                else
                {
                    cb->runtime-=(ActionTime5-ActionTime4/2-ActionTime1)-ActionTime6;
                    SY_StepTime(cb);
                }
                
                
            }                                        
        }
        /* 恢复默认优先级 */
        vTaskPrioritySet(cb->handle, PRIO_LOWEST);   
    }

    if(cb->NoCloseCapFlag==1)//如果最后一步完成后机械臂没升起
    {
        cb->NoCloseCapFlag=0;
        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat),0,Z1Shaketime);//从最后一步的缸中取样品
        //计算最后一步的使时间
        cb->runtime=(((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ)-(ActionTime5-ActionTime4/2)+ActionTime6;
        SY_StepTime(cb);

        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat)]);//释放最后一步的试剂缸锁
        if(cb->exp_id==1)
        TakeGetSample(2,30,0,0);//将样品放到输出仓1
        else if(cb->exp_id==2)
        TakeGetSample(2,31,0,0);//将样品放到输出仓2
        else if(cb->exp_id==3)
        TakeGetSample(2,32,0,0);//将样品放到输出仓3
        xSemaphoreGive(TakeGetSampleMutex);
    }
    else
    {
        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat),0,Z1Shaketime);//从最后一步的缸中取样品
        //计算最后一步的使时间
        cb->runtime=(((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ)-(ActionTime5-ActionTime4/2-ActionTime1)+ActionTime6;
        SY_StepTime(cb);

        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat)]);//释放最后一步的试剂缸锁
        if(cb->exp_id==1)
        TakeGetSample(2,30,0,0);//将样品放到输出仓1
        else if(cb->exp_id==2)
        TakeGetSample(2,31,0,0);//将样品放到输出仓2
        else if(cb->exp_id==3)
        TakeGetSample(2,32,0,0);//将样品放到输出仓3
        xSemaphoreGive(TakeGetSampleMutex);
    }
    cb->runtime=((xTaskGetTickCount() - xBegainTime)*1000)/configTICK_RATE_HZ;//滴答数转毫秒，任务完成用时
    printf("%s完成，用时：%d ms\r\n",cb->task_name,cb->runtime);
    // if(cb->exp_id==1)
    // {
    //     SetScreen(12);//流程结束弹出取走样品对话框
    // }
    // else if(cb->exp_id==2)
    // {
    //     SetScreen(13);//流程结束弹出取走样品对话框
    // }
    // else if(cb->exp_id==3)
    // {
    //     SetScreen(14);//流程结束弹出取走样品对话框
    // }
    cb->is_running = 0; // 标记任务完成
    cb->handle = NULL; // 清空任务句柄
    vTaskDelete(NULL);  // 删除自身任务
    
}



/*------------------ 实验任务结束后处理 ------------------*/
/**
 * @brief 所有实验流程结束后对机械臂进行复位
 * @param pvParams 传入ExperimentCB指针
 */
void ExperimentEndProcessTask(void *pvParams) 
{
    /*全部实验流程任务结束检测 */
    bool should_cleanup = false;
    bool has_created_task = false;
    ExperimentCB *experiments= (ExperimentCB *)pvParams;
    while(1)
    {
        for (int i = 0; i < NUM_EXPERIMENTS; i++) 
        {
            if (task_created_flag[i]) 
            { // 先检查是否有任务被创建过
                has_created_task = true;
                if (experiments[i].is_running == 1) //如果任务还在运行
                {
                    should_cleanup = false;
                    break;
                }
                should_cleanup = true; // 所有已创建任务都完成
            }
        }
        
        if (has_created_task && should_cleanup)//所有实验流程完成进行相应的复位
        {
            should_cleanup = false;
            has_created_task = false;
            /*X2移动到26号缸再进行复位，防止在某些位置复位时撞到盖臂*/
            MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
            MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
            MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
            WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成

            SetScreen(11);//流程结束弹出取走样品对话框
            StainingPodStatus[30]=0;//恢复输出仓空闲状态
            StainingPodStatus[31]=0;//恢复输出仓空闲状态
            StainingPodStatus[32]=0;//恢复输出仓空闲状态

            if(HYMotoStatus==1)//如果启动混匀电机
            {
                HYMotoStatus=0;
                AxisMotors[8].RunMode=StopMode;//混匀电机停机
                AxisMotors[8].MotoModBuf[16]=GET_BYTE1(AxisMotors[8].RunMode);
                AxisMotors[8].MotoModBuf[17]=GET_BYTE0(AxisMotors[8].RunMode);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//X3停机
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x0C,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x18,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x24,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
            }

            X1Y1Z1GoHome();//机械臂复位
            X2Y2Z2GoHome();
            WaitMotoStop(1,LSMotoStatus,2);
            WaitMotoStop(2,LSMotoStatus,2);
            WaitMotoStop(3,LSMotoStatus,2);
            WaitMotoStop(4,LSMotoStatus,2);
            WaitMotoStop(5,LSMotoStatus,2);
            WaitMotoStop(6,LSMotoStatus,2);
            memset(task_created_flag, 0, sizeof(task_created_flag)); // 重置标记
        }
        delay_ms(100);
    }   
}
/*------------------ 初始化函数 ------------------*/
/**
 * @brief 初始化实验配置
 * @param cb 实验控制块指针
 * @param exp_id 实验ID(1-3)
 */
void InitExperimentConfig(ExperimentCB *cb, uint8_t exp_id) {
    sprintf(cb->task_name, "Exp%d", exp_id);
    cb->is_running = 0;
    cb->current_step = 0;
    cb->exp_id = exp_id;
    u16 shuzu1[NUM_STEPS][2],shuzu2[NUM_STEPS][2],shuzu3[NUM_STEPS][2];
    u8 temp1_count=0,temp2_count=0,temp3_count=0;

    //将实验步骤中试剂号超出范围的无效步骤剔除
    for (int i = 0; i < NUM_STEPS; i++) 
    {
        if((cb->exp_id==1)&&(shiyan1Param[i][0]>=1)&&(shiyan1Param[i][0]<=26))
        {
            shuzu1[temp1_count][0] = shiyan1Param[i][0]; // 试剂号
            shuzu1[temp1_count][1] = shiyan1Param[i][1]; // 反应时间
            temp1_count++;
        }
        else if((cb->exp_id==2)&&(shiyan2Param[i][0]>=1)&&(shiyan2Param[i][0]<=26))
        {
            shuzu2[temp2_count][0] = shiyan2Param[i][0]; // 试剂号
            shuzu2[temp2_count][1] = shiyan2Param[i][1]; // 反应时间
            temp2_count++;
        }
        else if((cb->exp_id==3)&&(shiyan3Param[i][0]>=1)&&(shiyan3Param[i][0]<=26))
        {
            shuzu3[temp3_count][0] = shiyan3Param[i][0]; // 试剂号
            shuzu3[temp3_count][1] = shiyan3Param[i][1]; // 反应时间
            temp3_count++;
        }
    }

    if(cb->exp_id==1)
    cb->Vaild_Step_Num=temp1_count;
    else if(cb->exp_id==2)
    cb->Vaild_Step_Num=temp2_count;
    else if(cb->exp_id==3)
    cb->Vaild_Step_Num=temp3_count;

    /* 初始化所有步骤 */
    for (int i = 0; i < NUM_STEPS; i++) 
    {
        if((cb->exp_id==1)&&(i<temp1_count))
        {
            cb->steps[i].target_vat = shuzu1[i][0]; // 试剂号
            cb->steps[i].reaction_time = shuzu1[i][1]; // 反应时间
        }
        else if((cb->exp_id==2)&&(i<temp2_count))
        {
            cb->steps[i].target_vat = shuzu2[i][0]; // 试剂号
            cb->steps[i].reaction_time = shuzu2[i][1]; // 反应时间
        }
        else if((cb->exp_id==3)&&(i<temp3_count))
        {
            cb->steps[i].target_vat = shuzu3[i][0]; // 试剂号
            cb->steps[i].reaction_time = shuzu3[i][1]; // 反应时间
        } 
        
        
        /* 设置步骤优先级 物理步骤被剔除后原来的优先级条件需要重新评估有效性*/
        if (i == 10 || i == 15) 
        {
            cb->steps[i].priority = PRIO_HIGHEST;
        } 
        else if (i == 16|| i == 17) 
        {
            cb->steps[i].priority = PRIO_MID;
        } 
        else 
        {
            cb->steps[i].priority = PRIO_LOWEST;
        }
    }
}

/*------------------ 命令处理任务 ------------------*/
/**
 * @brief 处理用户启动命令
 * @param pvParams 未使用
 */
void CheckAndStartSYliuchengTask(void *pvParams) {
    UserCommand cmd;
    static ExperimentCB experiments[NUM_EXPERIMENTS];

    MotoShakeWaterInit(ZstepShakeinterval,Z1ShakeSpeed,Z1ShakeAcc,Z1ShakeDec);
    MotoBasketCapInit();
    /* 初始化实验配置 */
    for (int i = 0; i < NUM_EXPERIMENTS; i++) {
        InitExperimentConfig(&experiments[i], i+1);
    }
    
    xTaskCreate((TaskFunction_t )ExperimentEndProcessTask,
                (const char*    )"ExpEndProcessTask", 
                (uint16_t       )100, 
                (void*          )&experiments, 
                (UBaseType_t    )PRIO_LOWEST, 
                (TaskHandle_t*  )&ExpEndProcessTask_Handler);


    while (1) 
    {
        /* 等待用户命令 */
        if (xQueueReceive(shiyanliuchengQueue, &cmd, portMAX_DELAY) == pdPASS) 
        {
            xSemaphoreTake(xExpCreateMutex, portMAX_DELAY);
            for (int i = 0; i < NUM_EXPERIMENTS; i++)//如果用户更改了实验流程参数，重新加载参数
            {
                if (shiyanParamChangeFlag[i] == 1) 
                {
                    shiyanParamChangeFlag[i] = 0;
                    InitExperimentConfig(&experiments[i], i+1);
                }
            }
            uint8_t exp_idx = cmd - 1;
            if (!experiments[exp_idx].is_running) {
                /* 创建实验任务 */
                xTaskCreate(ExperimentTask,
                          experiments[exp_idx].task_name,
                          500,
                          &experiments[exp_idx],
                          PRIO_LOWEST,
                          &experiments[exp_idx].handle);
                task_created_flag[exp_idx] = 1; // 标记任务已创建
            }
            xSemaphoreGive(xExpCreateMutex);
        }
    } 
}

/*------------------ 加液排液任务 ------------------*/
/**
 * @brief 根据用户选择实验流程开启前进行试剂加液，流程结束后进行试剂回收，以及清洗管道
 * @param pvParams 未使用
 */
void AddAndEmptyLiquid(void *pvParams) 
{
    while(1)
    {


    }
    
}



/*------------------ 用户接口函数 ------------------*/
/**
 * @brief 发送实验启动命令
 * @param exp_id 要启动的实验ID(1-3)
 */
void StartExperiment(uint8_t exp_id) 
{
    UserCommand cmd = (UserCommand)exp_id;
    xQueueSend(shiyanliuchengQueue, &cmd, portMAX_DELAY);
}

