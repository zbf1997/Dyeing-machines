#ifndef _shiyanliucheng_H
#define _shiyanliucheng_H

#include "FreeRTOS.h"
#include "task.h"
/*------------------ 系统配置 ------------------*/
#define NUM_VATS         26             // 染色缸数量
#define NUM_STEPS        40             // 每个实验步骤数
#define NUM_EXPERIMENTS  3              // 最大并行实验数
#define CMD_QUEUE_SIZE   5              // 命令队列容量


/* 优先级定义 */
#define PRIO_LOWEST      5              // 最低优先级
#define PRIO_MID         6              // 中等优先级(步骤17、18)
#define PRIO_HIGHEST     7              // 最高优先级(步骤11,16)

/*------------------ 类型定义 ------------------*/
/* 实验步骤配置 */
typedef struct {
    uint8_t target_vat;                 // 试剂瓶号(0-25)，试剂瓶与缸体有映射关系通过TankAndReagenMapping()函数将试剂瓶号转换为缸体号
    uint32_t reaction_time;             // 反应时间(ms)
    uint8_t priority;                   // 步骤优先级
} StepConfig;

/* 实验任务控制块 */
typedef struct {
    char task_name[10];                 // 任务名称
    StepConfig steps[NUM_STEPS];        // 步骤配置数组
    TaskHandle_t handle;                // 任务句柄
    uint8_t current_step;               // 当前步骤索引
    uint8_t is_running;                 // 运行状态标志
    uint8_t exp_id;                     // 实验ID(1-3)
    uint8_t NoCloseCapFlag;             // 机械臂未升起盖子未关闭标志  
    uint64_t runtime;                   // 运行时间(ms)
    uint8_t Vaild_Step_Num;            // 有效步骤数
} ExperimentCB;

/* 用户命令类型 */
typedef enum {
    CMD_START_EXP1 = 1,                // 启动实验1
    CMD_START_EXP2,                     // 启动实验2
    CMD_START_EXP3                      // 启动实验3
} UserCommand;



void StartExperiment(uint8_t exp_id);
void CheckAndStartSYliuchengTask(void *pvParams);
extern TaskHandle_t CheckSYliuchengTask_Handler;
#endif