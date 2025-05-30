#ifndef __PID_H
#define __PID_H

#include "sys.h"

/******************************************************************************************/
/* PID��ز��� */

#define  INCR_LOCT_SELECT  0         /* 0��λ��ʽ ��1������ʽ */

#if INCR_LOCT_SELECT

/* ����ʽPID������غ� */
#define  KP      8.50f               /* P����*/
#define  KI      5.00f               /* I����*/
#define  KD      0.10f               /* D����*/
#define  SMAPLSE_PID_SPEED  2500       /* �������� ��λms*/

#else

/* λ��ʽPID������غ� */
#define  KP      15.00f               /* P����*/
#define  KI      0.01f               /* I����*/
#define  KD      0.01f                /* D����*/
#define  Calu_TEMP_PID  400          /* PID�������� ��λms*/
 
#endif

/* PID�����ṹ�� */
typedef struct
{
    __IO float  SetPoint;            /* �趨Ŀ�� */
    __IO float  ActualValue;         /* �������ֵ */
    __IO float  SumError;            /* ����ۼ� */
    __IO float  Proportion;          /* �������� P */
    __IO float  Integral;            /* ���ֳ��� I */
    __IO float  Derivative;          /* ΢�ֳ��� D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
    float pwmcycle;               //pwm����
} PID_TypeDef;


/******************************************************************************************/

void pid_init_initial(void);                 /* pid��ʼ�� */
float increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);      /* pid�ջ����� */

#endif
