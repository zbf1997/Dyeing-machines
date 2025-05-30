/***************************************************
 * 1. ����26��Ⱦɫ�׵�Эͬ����
 * 2. ֧��3��ʵ�����̲�������
 * 3. ��̬���ȼ�����
 * 4. �û���������ʵ��
 ***********************************************/

/*------------------ ͷ�ļ����� ------------------*/
#include "stm32f4xx.h"                  // STM32��׼��
#include "FreeRTOS.h"                   // FreeRTOS�ں�
#include "task.h"                      // �������
#include "semphr.h"                     // �ź���
#include "queue.h"                      // ����
#include "string.h"                     // �ַ�������
#include "GlobalVariable.h"
#include "shiyanliucheng.h"
#include "Moto_MotionAndUncap.h"
#include "delay.h"
TaskHandle_t CheckSYliuchengTask_Handler;

/*------------------ ʵ�������� ------------------*/
/**
 * @brief ʵ����������
 * @param pvParams ����ExperimentCBָ��
 */
void ExperimentTask(void *pvParams) {
    ExperimentCB *cb = (ExperimentCB *)pvParams;
    cb->is_running = 1; // �������ʼ
    StepConfig *step;
    u16 fanyingTime;//�Լ���Ӧʱ��
    TickType_t xLastTime = xTaskGetTickCount();


    /*������ȿ��ؿ���*/
    if(ucg_X3Y3Z3A3RunBtn==1&&HYMotoStatus==0)
    {
        HYMotoStatus=1;
        AxisMotors[8].RunMode=HYMode;//����ģʽ
        AxisMotors[8].MotoModBuf[16]=GET_BYTE1(AxisMotors[8].RunMode);
        AxisMotors[8].MotoModBuf[17]=GET_BYTE0(AxisMotors[8].RunMode);
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//д��X3����г̣����β��������ٶȣ����ٶȣ��ٶȣ�����������ģʽ
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x0C,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//д��Y3����г̣����β��������ٶȣ����ٶȣ��ٶȣ�����������ģʽ
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x18,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//д��Z3����г̣����β��������ٶȣ����ٶȣ��ٶȣ�����������ģʽ
        delay_ms(500);
        MODH_WriteOrReadParam(10,11,0x24,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//д��A3����г̣����β��������ٶȣ����ٶȣ��ٶȣ�����������ģʽ
        delay_ms(60);
    }
    // ���������Ƭ������ʵ�����̽���ȡ���ͷ���
    if (kaopian[0]==1 || kaopian[2]==1 || kaopian[4]==1) {
        u8 bakeTank = (cb->exp_id == 1) ? 26 : (cb->exp_id == 2) ? 27 : 28; // ���ʵ������Ϊ1����Ƭ��Ϊ26��ʵ������Ϊ2����Ƭ��Ϊ27��ʵ������Ϊ3����Ƭ��Ϊ28
        u8 delayIdx = (cb->exp_id == 1) ? 1 : (cb->exp_id == 2) ? 3 : 5;    // ���ʵ������Ϊ1����Ƭʱ�����������Ϊ1��ʵ������Ϊ2����Ƭʱ�����������Ϊ3��ʵ������Ϊ3����Ƭʱ�����������Ϊ5
        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);    // ����ȡ��������
        StainingPodStatus[29] = 1;                            // �������������Ʒ
        TakeGetSample(1, 29, 0, 0);                           // �������ȡ��
        TakeGetSample(2, bakeTank, 0, 0);                     // ��������Ƭ��
        xSemaphoreGive(TakeGetSampleMutex);                   // �ͷ�ȡ��������
        vTaskDelay((kaopian[delayIdx] * 1000) / portTICK_RATE_MS); // ��Ƭʱ��
    }
   

    for (cb->current_step = 0; cb->current_step < cb->Vaild_Step_Num; cb->current_step++) 
    {
        step = &cb->steps[cb->current_step]; 
        /* ��̬�����������ȼ���Ҳ����ʵ�鲽������ȼ� */
        vTaskPrioritySet(cb->handle, step->priority);
        /* ����Ŀ�����Դ�� */
        if (xSemaphoreTake(xVatMutex[TankAndReagenMapping(step->target_vat)], portMAX_DELAY) == pdPASS)//�����û��ռ�� 
        {
            if(cb->current_step == 0)//����ǵ�һ��
            {
                xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY); //����ȡ��������
                if(step->reaction_time<=(ActionTime1+ActionTime2+ActionTime3))//��Ӧʱ��С�ڵ��ڻ�е�۽���Ʒ�����Լ���ĸ�λʱ��
                {
                    /*Ϊʲô�ڲ�����е�۵Ĳ�����Ҫ���ж���һ���еĸ����Ƿ��ͷţ�
                    һ��ĳ��ʵ�����̲������е�۲��ظǣ���һֱռ��ȡ������
                    �������̵Ĳ���϶��ò���ִ�У�ֱ����ʵ�����̵�һ������
                    �������������е�۵Ĳ�����ɣ�ִ������һ�������е�۵Ĳ��裬
                    �����ͷ�ȡ�������������̲���ִ�У�����������������������
                    ��е�۵Ĳ��裬һ��Ҫ���ж���һ������ĸ����Ƿ��ͷţ�
                    �����һ���ĸ���û���ͷţ��ͽ�����һ���裬
                    �ò���һֱռ��ȡ��������������һ���ͷ�ȡ�����ģ�
                    ������һ������һֱ�ڵȴ��������Ӷ��ͷŲ���ȡ������
                    ��������û��ȡ������ռ�õĸ������޷��ͷţ��Ӷ�������������е���������*/
                    
                    
                    xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//�ȴ���һ���ĸ����ͷ������ռ�õĻ�
                    if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                    xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//�������3���Ĳ�����е�۵Ĳ���
                    

                    if(kaopian[0]==1 | kaopian[2]==1 | kaopian[4]==1)//���������Ƭ����ӿ�Ƭ��ȡ��
                    {
                        u8 bakeTank = (cb->exp_id == 1) ? 26 : (cb->exp_id == 2) ? 27 : 28;//���ʵ������Ϊ1����Ƭ��Ϊ26��ʵ������Ϊ2����Ƭ��Ϊ27��ʵ������Ϊ3����Ƭ��Ϊ28
                        TakeGetSample(1, bakeTank, 0, 0); // �ӿ�Ƭ��ȡ��
                    }
                    else TakeGetSample(1,29,0,0);//����������ȡ��


                    TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);//�����½��������������Լ��ף���������ۣ����رո���
                    printf("%s di(1)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                    
                    xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                    if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                    xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//�ͷŸ���
                    
                    if(step->reaction_time>ActionTime3)//��Ӧʱ����ڵ��ָ����Ӧʱ��
                    {
                        fanyingTime=step->reaction_time-ActionTime3;
                        vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                    }
                    cb->NoCloseCapFlag=1;//���δ�ظ�
                }
                else
                {
    
                    if(kaopian[0]==1 | kaopian[2]==1 | kaopian[4]==1)//���������Ƭ����ӿ�Ƭ��ȡ��
                    {
                        u8 bakeTank = (cb->exp_id == 1) ? 26 : (cb->exp_id == 2) ? 27 : 28;//���ʵ������Ϊ1����Ƭ��Ϊ26��ʵ������Ϊ2����Ƭ��Ϊ27��ʵ������Ϊ3����Ƭ��Ϊ28
                        TakeGetSample(1, bakeTank, 0, 0); // �ӿ�Ƭ��ȡ��
                    }
                    else TakeGetSample(1,29,0,0);//����������ȡ��

                    TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                    printf("%s di(2)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                    fanyingTime=step->reaction_time-(ActionTime1+ActionTime2+ActionTime3);
                    xSemaphoreGive(TakeGetSampleMutex); //�ͷ�ȡ��������
                    vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                }
            }
            else//��һ��֮��
            {                    
                if(step->reaction_time<=(ActionTime1+ActionTime2+ActionTime3))//��Ӧʱ��С�ڵ��ڻ�е�۽���Ʒ�����Լ���ĸ�λʱ��
                {
                    if(cb->NoCloseCapFlag==1)//�����һ����е��û����
                    {
                        /***********************************/
                        //���������������е�۵ȴ��������ĸ������
                        if(cb->current_step<(23-2))
                        {
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//�ȴ���һ���ĸ����ͷ������ռ�õĻ�
                            if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//�������3���Ĳ�����е�۵Ĳ���
                        }
                        /***********************************/

                        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),1,Z1Shaketime);//����һ���ĸ���ȡ��Ʒ
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//�ͷ���һ������Լ�����
                        TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);
                        printf("%s (1)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        /***********************************/
                        //�ͷź������ĸ�����
                        if(cb->current_step<(23-2))
                        {
                            
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                            if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//�ͷŸ���
                        }
                        /***********************************/

                        if(step->reaction_time>ActionTime3)//��Ӧʱ����ڵ��ָ����Ӧʱ��
                        {
                            fanyingTime=step->reaction_time-ActionTime3;                               
                            vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                        }

                        
                    }
                    else
                    {
                        cb->NoCloseCapFlag=1;

                        /***********************************/
                        //���������������е�۵ȴ��������ĸ������
                        if(cb->current_step<(23-2))//ʵ��������23��
                        {
                            
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)], portMAX_DELAY);//�ȴ���һ���ĸ����ͷ������ռ�õĻ�
                            if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                            xSemaphoreTake(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)], portMAX_DELAY);//�������3���Ĳ�����е�۵Ĳ���
                        }
                        /***********************************/

                        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
                        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),1,Z1Shaketime);//����һ���ĸ���ȡ��Ʒ
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//�ͷ���һ������Լ�����
                        TakeGetSampleNoCloseCap(2,TankAndReagenMapping(step->target_vat),0,0);
                        printf("%s (2)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        /***********************************/
                        //�ͷź������ĸ�����
                        if(cb->current_step<(23-2))
                        {
                            
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+1])->target_vat)]);
                            if(((&cb->steps[cb->current_step+2])->reaction_time)<=ActionTime1+ActionTime2+ActionTime3)
                            xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step+2])->target_vat)]);//�ͷŸ���
                        }
                        /***********************************/

                        if(step->reaction_time>ActionTime3)//��Ӧʱ����ڵ��ָ����Ӧʱ��
                        {
                            fanyingTime=step->reaction_time-ActionTime3;                               
                            vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                        }

                        
                    }
                    
                }
                else//��Ӧʱ����ڻ�е�۽���Ʒ�����Լ���ĸ�λʱ��
                {
                    if(cb->NoCloseCapFlag==1)//�����һ����е��û����
                    {
                        cb->NoCloseCapFlag=0;
                        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),1,Z1Shaketime);//����һ���ĸ���ȡ��Ʒ                           
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//�ͷ���һ������Լ�����
                        TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                        printf("%s (3)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        xSemaphoreGive(TakeGetSampleMutex);
                        fanyingTime=(step->reaction_time)-(ActionTime1+ActionTime2+ActionTime3);
                        vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                    }
                    else
                    {
                        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
                        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat),1,Z1Shaketime);//����һ���ĸ���ȡ��Ʒ
                        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->current_step-1])->target_vat)]);//�ͷ���һ������Լ�����
                        TakeGetSample(2,TankAndReagenMapping(step->target_vat),0,0);
                        printf("%s (4)��ǰ���裺%d ��Ӧ���壺%d ��Ӧʱ�䣺%d ��ǰ�������ȼ���%d\r\n",cb->task_name,(cb->current_step)+1,step->target_vat,step->reaction_time,step->priority);
                        xSemaphoreGive(TakeGetSampleMutex);
                        fanyingTime=(step->reaction_time)-(ActionTime1+ActionTime2+ActionTime3);    
                        vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);

                    }                        
                }
            }                                        
        }
        /* �ָ�Ĭ�����ȼ� */
        vTaskPrioritySet(cb->handle, PRIO_LOWEST);        
    }
    if(cb->NoCloseCapFlag==1)//������һ����ɺ��е��û����
    {
        cb->NoCloseCapFlag=0;
        TakeGetSampleNoCloseCap(1,TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat),1,Z1Shaketime);//�����һ���ĸ���ȡ��Ʒ
        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat)]);//�ͷ����һ�����Լ�����
        if(cb->exp_id==1)
        TakeGetSample(2,30,0,0);//����Ʒ�ŵ������1
        else if(cb->exp_id==2)
        TakeGetSample(2,31,0,0);//����Ʒ�ŵ������2
        else if(cb->exp_id==3)
        TakeGetSample(2,32,0,0);//����Ʒ�ŵ������3
        xSemaphoreGive(TakeGetSampleMutex);
    }
    else
    {
        xSemaphoreTake(TakeGetSampleMutex, portMAX_DELAY);
        TakeGetSample(1,TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat),1,Z1Shaketime);//�����һ���ĸ���ȡ��Ʒ
        xSemaphoreGive(xVatMutex[TankAndReagenMapping((&cb->steps[cb->Vaild_Step_Num-1])->target_vat)]);//�ͷ����һ�����Լ�����
        if(cb->exp_id==1)
        TakeGetSample(2,30,0,0);//����Ʒ�ŵ������1
        else if(cb->exp_id==2)
        TakeGetSample(2,31,0,0);//����Ʒ�ŵ������2
        else if(cb->exp_id==3)
        TakeGetSample(2,32,0,0);//����Ʒ�ŵ������3
        xSemaphoreGive(TakeGetSampleMutex);
    }
    
    cb->is_running = 0; // ����������
    cb->runtime=((xTaskGetTickCount() - xLastTime)*1000)/configTICK_RATE_HZ;//�δ���ת���룬���������ʱ
    printf("%s��ɣ���ʱ��%d ms\r\n",cb->task_name,cb->runtime);
    if(cb->exp_id==1)
    {
        SetScreen(12);//���̽�������ȡ����Ʒ�Ի���
    }
    else if(cb->exp_id==2)
    {
        SetScreen(13);//���̽�������ȡ����Ʒ�Ի���
    }
    else if(cb->exp_id==3)
    {
        SetScreen(14);//���̽�������ȡ����Ʒ�Ի���
    }
    cb->handle = NULL; // ���������
    vTaskDelete(NULL);  // ɾ����������
    
}

/*------------------ ��ʼ������ ------------------*/
/**
 * @brief ��ʼ��ʵ������
 * @param cb ʵ����ƿ�ָ��
 * @param exp_id ʵ��ID(1-3)
 */
void InitExperimentConfig(ExperimentCB *cb, uint8_t exp_id) {
    sprintf(cb->task_name, "Exp%d", exp_id);
    cb->is_running = 0;
    cb->current_step = 0;
    cb->exp_id = exp_id;
    u16 shuzu1[NUM_STEPS][2],shuzu2[NUM_STEPS][2],shuzu3[NUM_STEPS][2];
    u8 temp1_count=0,temp2_count=0,temp3_count=0;

    //��ʵ�鲽�����Լ��ų�����Χ����Ч�����޳�
    for (int i = 0; i < NUM_STEPS; i++) 
    {
        if((cb->exp_id==1)&&(shiyan1Param[i][0]>=1)&&(shiyan1Param[i][0]<=26))
        {
            shuzu1[temp1_count][0] = shiyan1Param[i][0]; // �Լ���
            shuzu1[temp1_count][1] = shiyan1Param[i][1]; // ��Ӧʱ��
            temp1_count++;
        }
        else if((cb->exp_id==2)&&(shiyan2Param[i][0]>=1)&&(shiyan2Param[i][0]<=26))
        {
            shuzu2[temp2_count][0] = shiyan2Param[i][0]; // �Լ���
            shuzu2[temp2_count][1] = shiyan2Param[i][1]; // ��Ӧʱ��
            temp2_count++;
        }
        else if((cb->exp_id==3)&&(shiyan3Param[i][0]>=1)&&(shiyan3Param[i][0]<=26))
        {
            shuzu3[temp3_count][0] = shiyan3Param[i][0]; // �Լ���
            shuzu3[temp3_count][1] = shiyan3Param[i][1]; // ��Ӧʱ��
            temp3_count++;
        }
    }

    if(cb->exp_id==1)
    cb->Vaild_Step_Num=temp1_count;
    else if(cb->exp_id==2)
    cb->Vaild_Step_Num=temp2_count;
    else if(cb->exp_id==3)
    cb->Vaild_Step_Num=temp3_count;

    /* ��ʼ�����в��� */
    for (int i = 0; i < NUM_STEPS; i++) 
    {
        if((cb->exp_id==1)&&(i<temp1_count))
        {
            cb->steps[i].target_vat = shuzu1[i][0]; // �Լ���
            cb->steps[i].reaction_time = shuzu1[i][1]; // ��Ӧʱ��
        }
        else if((cb->exp_id==2)&&(i<temp2_count))
        {
            cb->steps[i].target_vat = shuzu2[i][0]; // �Լ���
            cb->steps[i].reaction_time = shuzu2[i][1]; // ��Ӧʱ��
        }
        else if((cb->exp_id==3)&&(i<temp3_count))
        {
            cb->steps[i].target_vat = shuzu3[i][0]; // �Լ���
            cb->steps[i].reaction_time = shuzu3[i][1]; // ��Ӧʱ��
        } 
        
        
        /* ���ò������ȼ� �����豻�޳���ԭ�������ȼ�������Ҫ����������Ч��*/
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

/*------------------ ��������� ------------------*/
/**
 * @brief �����û���������
 * @param pvParams δʹ��
 */
void CheckAndStartSYliuchengTask(void *pvParams) {
    UserCommand cmd;
    static ExperimentCB experiments[NUM_EXPERIMENTS];
    bool task_created_flag[NUM_EXPERIMENTS] = {0}; // ���񴴽��������

    MotoShakeWaterInit(ZstepShakeinterval,Z1ShakeSpeed,Z1ShakeAcc,Z1ShakeDec);
    MotoBasketCapInit();
    /* ��ʼ��ʵ������ */
    for (int i = 0; i < NUM_EXPERIMENTS; i++) {
        InitExperimentConfig(&experiments[i], i+1);
    }
    
    while (1) 
    {
        /* �ȴ��û����� */
        if (xQueueReceive(shiyanliuchengQueue, &cmd, portMAX_DELAY) == pdPASS) {
            xSemaphoreTake(xExpCreateMutex, portMAX_DELAY);
            
            uint8_t exp_idx = cmd - 1;
            if (!experiments[exp_idx].is_running) {
                /* ����ʵ������ */
                xTaskCreate(ExperimentTask,
                          experiments[exp_idx].task_name,
                          500,
                          &experiments[exp_idx],
                          PRIO_LOWEST,
                          &experiments[exp_idx].handle);
                task_created_flag[exp_idx] = 1; // ��������Ѵ���
            }
            xSemaphoreGive(xExpCreateMutex);
        }

         /*ȫ��ʵ���������������� */
        bool should_cleanup = false;
        bool has_created_task = false;
        
        for (int i = 0; i < NUM_EXPERIMENTS; i++) {
            if (task_created_flag[i]) 
            { // �ȼ���Ƿ������񱻴�����
                has_created_task = true;
                if (experiments[i].handle != NULL) 
                {
                    should_cleanup = false;
                    break;
                }
                should_cleanup = true; // �����Ѵ����������
            }
        }
        
        if (has_created_task && should_cleanup)//����ʵ��������ɽ�����Ӧ�ĸ�λ
        {
            /*X2�ƶ���26�Ÿ��ٽ��и�λ����ֹ��ĳЩλ�ø�λʱײ���Ǳ�*/
            MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�ø�λ
            MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//�趨PR0λ�õ�λ
            MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������PR0
            WaitMotoStop(4,LSMotoStatus,2);//�ȴ�X2�����

            SetScreen(11);//���̽�������ȡ����Ʒ�Ի���
            StainingPodStatus[30]=0;//�ָ�����ֿ���״̬
            StainingPodStatus[31]=0;//�ָ�����ֿ���״̬
            StainingPodStatus[32]=0;//�ָ�����ֿ���״̬

            if(HYMotoStatus==1)//����������ȵ��
            {
                HYMotoStatus=0;
                AxisMotors[8].RunMode=StopMode;//���ȵ��ͣ��
                AxisMotors[8].MotoModBuf[16]=GET_BYTE1(AxisMotors[8].RunMode);
                AxisMotors[8].MotoModBuf[17]=GET_BYTE0(AxisMotors[8].RunMode);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);//X3ͣ��
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x0C,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x18,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
                delay_ms(500);
                MODH_WriteOrReadParam(10,11,0x24,0,9,AxisMotors[8].MotoModBuf,MODH_CmdMutex);
            }

            X1Y1Z1GoHome();//��е�۸�λ
            X2Y2Z2GoHome();
            WaitMotoStop(1,LSMotoStatus,2);
            WaitMotoStop(2,LSMotoStatus,2);
            WaitMotoStop(3,LSMotoStatus,2);
            WaitMotoStop(4,LSMotoStatus,2);
            WaitMotoStop(5,LSMotoStatus,2);
            WaitMotoStop(6,LSMotoStatus,2);
            memset(task_created_flag, 0, sizeof(task_created_flag)); // ���ñ��
        }
        delay_ms(100);
    } 
}

/*------------------ �û��ӿں��� ------------------*/
/**
 * @brief ����ʵ����������
 * @param exp_id Ҫ������ʵ��ID(1-3)
 */
void StartExperiment(uint8_t exp_id) 
{
    UserCommand cmd = (UserCommand)exp_id;
    xQueueSend(shiyanliuchengQueue, &cmd, portMAX_DELAY);
}

