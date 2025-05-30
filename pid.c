#include "pid.h"
#include "stdio.h"
#include "GlobalVariable.h"

void pid_init_initial(void)
{
    temp_pid.SetPoint = TempPidSetPoint;       /* 设定目标值 */
    temp_pid.ActualValue = 0.0;  /* 期望输出值 */
    temp_pid.SumError = 0.0;     /* 积分值 */
    temp_pid.Error = 0.0;        /* Error[1] */
    temp_pid.LastError = 0.0;    /* Error[-1] */
    temp_pid.PrevError = 0.0;    /* Error[-2] */
    temp_pid.Proportion = KP;    /* 比例常数 Proportional Const */
    temp_pid.Integral = KI;      /* 积分常数 Integral Const */
    temp_pid.Derivative = KD;    /* 微分常数 Derivative Const */ 
    temp_pid.pwmcycle = 300.0;     //pid计算后的值超过此值就等于此值
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
float increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{

    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
#if  INCR_LOCT_SELECT                                                       /* 增量式PID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
    PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
    PID->LastError = PID->Error;
    
#else                                                                       /* 位置式PID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                       + (PID->Integral * PID->SumError)                    /* 积分环节 */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
    PID->LastError = PID->Error;
#endif
//对输出进行限幅
    if(PID->ActualValue>=temp_pid.pwmcycle)
    {
        PID->ActualValue=temp_pid.pwmcycle;
    }
    if(PID->ActualValue<=0)
    {
        PID->ActualValue=0.0;
    }
    return (PID->ActualValue);                                   /* 返回计算后输出的数值 */
}
