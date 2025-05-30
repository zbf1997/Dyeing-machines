#ifndef __PID_H
#define __PID_H

#include "sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  0         /* 0：位置式 ，1：增量式 */

#if INCR_LOCT_SELECT

/* 增量式PID参数相关宏 */
#define  KP      8.50f               /* P参数*/
#define  KI      5.00f               /* I参数*/
#define  KD      0.10f               /* D参数*/
#define  SMAPLSE_PID_SPEED  2500       /* 采样周期 单位ms*/

#else

/* 位置式PID参数相关宏 */
#define  KP      15.00f               /* P参数*/
#define  KI      0.01f               /* I参数*/
#define  KD      0.01f                /* D参数*/
#define  Calu_TEMP_PID  400          /* PID计算周期 单位ms*/
 
#endif

/* PID参数结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
    float pwmcycle;               //pwm周期
} PID_TypeDef;


/******************************************************************************************/

void pid_init_initial(void);                 /* pid初始化 */
float increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);      /* pid闭环控制 */

#endif
