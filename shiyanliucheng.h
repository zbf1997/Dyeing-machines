#ifndef _shiyanliucheng_H
#define _shiyanliucheng_H

#include "FreeRTOS.h"
#include "task.h"
/*------------------ ϵͳ���� ------------------*/
#define NUM_VATS         26             // Ⱦɫ������
#define NUM_STEPS        40             // ÿ��ʵ�鲽����
#define NUM_EXPERIMENTS  3              // �����ʵ����
#define CMD_QUEUE_SIZE   5              // �����������


/* ���ȼ����� */
#define PRIO_LOWEST      5              // ������ȼ�
#define PRIO_MID         6              // �е����ȼ�(����17��18)
#define PRIO_HIGHEST     7              // ������ȼ�(����11,16)

/*------------------ ���Ͷ��� ------------------*/
/* ʵ�鲽������ */
typedef struct {
    uint8_t target_vat;                 // �Լ�ƿ��(0-25)���Լ�ƿ�������ӳ���ϵͨ��TankAndReagenMapping()�������Լ�ƿ��ת��Ϊ�����
    uint32_t reaction_time;             // ��Ӧʱ��(ms)
    uint8_t priority;                   // �������ȼ�
} StepConfig;

/* ʵ��������ƿ� */
typedef struct {
    char task_name[10];                 // ��������
    StepConfig steps[NUM_STEPS];        // ������������
    TaskHandle_t handle;                // ������
    uint8_t current_step;               // ��ǰ��������
    uint8_t is_running;                 // ����״̬��־
    uint8_t exp_id;                     // ʵ��ID(1-3)
    uint8_t NoCloseCapFlag;             // ��е��δ�������δ�رձ�־  
    uint64_t runtime;                   // ����ʱ��(ms)
    uint8_t Vaild_Step_Num;            // ��Ч������
} ExperimentCB;

/* �û��������� */
typedef enum {
    CMD_START_EXP1 = 1,                // ����ʵ��1
    CMD_START_EXP2,                     // ����ʵ��2
    CMD_START_EXP3                      // ����ʵ��3
} UserCommand;



void StartExperiment(uint8_t exp_id);
void CheckAndStartSYliuchengTask(void *pvParams);
extern TaskHandle_t CheckSYliuchengTask_Handler;
#endif