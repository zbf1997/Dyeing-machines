#include "pid.h"
#include "stdio.h"
#include "GlobalVariable.h"

void pid_init_initial(void)
{
    temp_pid.SetPoint = TempPidSetPoint;       /* �趨Ŀ��ֵ */
    temp_pid.ActualValue = 0.0;  /* �������ֵ */
    temp_pid.SumError = 0.0;     /* ����ֵ */
    temp_pid.Error = 0.0;        /* Error[1] */
    temp_pid.LastError = 0.0;    /* Error[-1] */
    temp_pid.PrevError = 0.0;    /* Error[-2] */
    temp_pid.Proportion = KP;    /* �������� Proportional Const */
    temp_pid.Integral = KI;      /* ���ֳ��� Integral Const */
    temp_pid.Derivative = KD;    /* ΢�ֳ��� Derivative Const */ 
    temp_pid.pwmcycle = 300.0;     //pid������ֵ������ֵ�͵��ڴ�ֵ
}

/**
 * @brief       pid�ջ�����
 * @param       *PID��PID�ṹ�������ַ
 * @param       Feedback_value����ǰʵ��ֵ
 * @retval      �������ֵ
 */
float increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{

    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* ����ƫ�� */
#if  INCR_LOCT_SELECT                                                       /* ����ʽPID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* �������� */
                        + (PID->Integral * PID->Error)                                             /* ���ֻ��� */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* ΢�ֻ��� */
    
    PID->PrevError = PID->LastError;                                        /* �洢ƫ������´μ��� */
    PID->LastError = PID->Error;
    
#else                                                                       /* λ��ʽPID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* �������� */
                       + (PID->Integral * PID->SumError)                    /* ���ֻ��� */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* ΢�ֻ��� */
    PID->LastError = PID->Error;
#endif
//����������޷�
    if(PID->ActualValue>=temp_pid.pwmcycle)
    {
        PID->ActualValue=temp_pid.pwmcycle;
    }
    if(PID->ActualValue<=0)
    {
        PID->ActualValue=0.0;
    }
    return (PID->ActualValue);                                   /* ���ؼ�����������ֵ */
}
