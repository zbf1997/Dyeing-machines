#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "key.h"
#include "timer.h"
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"
#include "relay.h"
#include "cstring"
#include "semphr.h"
#include "GlobalVariable.h"
#include "bsp_usartx_fifo.h"
#include "modbus_host.h"
#include "Moto_MotionAndUncap.h"
#include "InputSlicDetect.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);
    delay_ms(3000);
    Uart1Init(115200);//����1�ʹ�����ͨѶ   
    Uart3Init(115200);//����3��������ڵ�������
	Usart_FIFO_Init(); //����2����modbusͨѶ                                                                   			                                                                                                                                                                               
	KEY_Init();
	Him_Init();
    Timer5_Init();
    bsp_InitHardTimer();
    PositionInit();
    InputSlicDetect_Init();
    MotoInit();
    TIM_Cmd(TIM5, DISABLE);//��ʱ��5����RTOSδ����ʱ�Ķ�ʱ,��ֹƵ�����жϵ���ϵͳ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    MODH_CmdMutex=xSemaphoreCreateMutex();//���������ź���
    UiCmdBianry=xSemaphoreCreateBinary();//������ֵ�ź���
    MotoMonitBianry=xSemaphoreCreateBinary();
    InputOutSlicBianry=xSemaphoreCreateBinary();
    TempControlBianry=xSemaphoreCreateBinary();
    shiyanlicuheng1Bianry=xSemaphoreCreateBinary();
    ValvePumpBianry=xSemaphoreCreateBinary();
  	xTaskCreate((TaskFunction_t )Process_message,     	
                (const char*    )"Process_message",   	
                (uint16_t       )Process_message_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )Process_message_PRIO,	
                (TaskHandle_t*  )&Process_message_Handler);   
				
	xTaskCreate((TaskFunction_t )moto_task,     
                (const char*    )"moto_task",   
                (uint16_t       )moto_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )moto_TASK_PRIO,
                (TaskHandle_t*  )&moto_Task_Handler);  

    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler); 

    xTaskCreate((TaskFunction_t )MotoStausCheck,     	
                (const char*    )"MotoStausCheck",   	
                (uint16_t       )MotoStausCheck_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )MotoStausCheck_TASK_PRIO,	
                (TaskHandle_t*  )&MotoStausCheckTask_Handler);

    xTaskCreate((TaskFunction_t )MonitorTasks,     	
                (const char*    )"MonitorTasks",   	
                (uint16_t       )MonitorTasks_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )MonitorTasks_TASK_PRIO,	
                (TaskHandle_t*  )&MonitorTasks_Handler);
    
    xTaskCreate((TaskFunction_t )InputOutSlic,     	
                (const char*    )"InputOutSlic",   	
                (uint16_t       )InputOutSlic_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )InputOutSlic_TASK_PRIO,	
                (TaskHandle_t*  )&InputOutSlic_Handler);

    xTaskCreate((TaskFunction_t )shiyanlicuheng1,     	
                (const char*    )"shiyanlicuheng1",   	
                (uint16_t       )shiyanlicuheng1_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )shiyanlicuheng1_TASK_PRIO,	
                (TaskHandle_t*  )&shiyanlicuheng1_Handler);

    xTaskCreate((TaskFunction_t )TempControl,     	
                (const char*    )"TempControl",   	
                (uint16_t       )TempControl_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )TempControl_TASK_PRIO,	
                (TaskHandle_t*  )&TempControl_Handler);

    xTaskCreate((TaskFunction_t )ValvePump,     	
                (const char*    )"ValvePump",   	
                (uint16_t       )ValvePump_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )ValvePump_TASK_PRIO,	
                (TaskHandle_t*  )&ValvePump_Handler);

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
}

void Process_message(void *pvParameters)
{
	qsize  size = 0;
    BaseType_t xReturn;
	while(1)
	{
        xReturn = xSemaphoreTake(UiCmdBianry,portMAX_DELAY); /* ��ȡ�ź��� */
        if (xReturn == pdTRUE)
        {
            size = queue_find_cmd(cmd_buffer,CMD_MAX_SIZE);
            if(size>0&&cmd_buffer[1]!=0x07)                                              //������в�Ϊ����ָ�Ϊ����ָ��
            {                                                                           
                ProcessMessage((PCTRL_MSG)cmd_buffer, size);                             //�Դ�����ָ����д���  
            }                                                                           
            else if(size>0&&cmd_buffer[1]==0x07)                                         //���������ָ��Ϊ0x07����ָ��˵�������������ˣ���ʱ��STM32��������������ٽ���һ����Ӳ���ĳ�ʼ��  
            {                                                                           
                __disable_fault_irq();                                                   
                NVIC_SystemReset();                                                                                                                                          
            }
        }
	}
}  

//LED������ 
void led0_task(void *pvParameters)
{
	LED_Init();
    while(1)
    {
		LED1=~LED1;
        vTaskDelay(500/portTICK_RATE_MS);
		LED0=~LED0;
        vTaskDelay(500/portTICK_RATE_MS);
        
    }
}
void MotoStausCheck(void *pvParameters)
{
    u8 str[20];
    u8 delaytime=150;
    TickType_t xLastTime, xLastTime1;
    while(1)
    {
        MODH_WriteOrReadParam(3,1,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯX1�ľ���λ��
        MODH_WriteOrReadParam(3,2,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯY1�ľ���λ��
        MODH_WriteOrReadParam(3,3,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯZ1�ľ���λ��
        MODH_WriteOrReadParam(3,4,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯX2�ľ���λ��
        MODH_WriteOrReadParam(3,5,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯY2�ľ���λ��
        MODH_WriteOrReadParam(3,6,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//��ѯZ2�ľ���λ��
        MODH_WriteOrReadParam(3,1,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯX1������״̬
        MODH_WriteOrReadParam(3,2,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯY1������״̬
        MODH_WriteOrReadParam(3,3,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯZ1������״̬
        MODH_WriteOrReadParam(3,4,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯX2������״̬
        MODH_WriteOrReadParam(3,5,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯY2������״̬
        MODH_WriteOrReadParam(3,6,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//��ѯZ2������״̬


        // ��MotoLocation�����е�ֵת��Ϊ�ַ����������õ�������
        sprintf(str,"%d",MotoLocation[0]);
        SetTextValue(8,37,str);
        SetTextValue(9,37,str);
        sprintf(str,"%d",MotoLocation[1]);
        SetTextValue(8,38,str);
        SetTextValue(9,38,str);
        sprintf(str,"%d",MotoLocation[2]);
        SetTextValue(8,39,str);
        SetTextValue(9,39,str);
        sprintf(str,"%d",MotoLocation[3]);
        SetTextValue(8,76,str);
        SetTextValue(9,76,str);
        sprintf(str,"%d",MotoLocation[4]);
        SetTextValue(8,77,str);
        SetTextValue(9,77,str);
        sprintf(str,"%d",MotoLocation[5]);
        SetTextValue(8,78,str);
        SetTextValue(9,78,str);
        delay_ms(3000);
        if (xTaskGetTickCount() - xLastTime1 > pdMS_TO_TICKS(5000)) 
        {
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x15,0,1,NULL,MODH_CmdMutex);//��ѯ1��̽ͷ�¶�
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x16,0,1,NULL,MODH_CmdMutex);//��ѯ2��̽ͷ�¶�
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x17,0,1,NULL,MODH_CmdMutex);//��ѯ3��̽ͷ�¶�
            delay_ms(500);
            sprintf(str,"%.1f",uhwg_RealTemp[0]/10.0);
            SetTextValue(10,9,str);
            printf("1���¿ص�ʵʱ�¶�Ϊ��%d\r\n",uhwg_RealTemp[0]);
            sprintf(str,"%.1f",uhwg_RealTemp[1]/10.0);
            SetTextValue(10,17,str);
            printf("2���¿ص�ʵʱ�¶�Ϊ��%d\r\n",uhwg_RealTemp[1]);
            sprintf(str,"%.1f",uhwg_RealTemp[2]/10.0);
            SetTextValue(10,28,str);
            printf("3���¿ص�ʵʱ�¶�Ϊ��%d\r\n",uhwg_RealTemp[2]);
            xLastTime1 = xTaskGetTickCount();
        } 
  
        if (xTaskGetTickCount() - xLastTime > pdMS_TO_TICKS(20000)) {  // ÿ5��ִ��һ��  
            char pcBuffer[1000];  
            vTaskList(pcBuffer);  // ��������״̬���
            printf("����״̬:\n%s\n", pcBuffer);  
            xLastTime = xTaskGetTickCount();  
        } 
    }
}

void MonitorTasks(void *pvParameters)
{
    u8 str[20];
    UBaseType_t uxOriginalPriority = uxTaskPriorityGet(NULL); // ��ȡ��ǰ��������ȼ�
	while(1)
	{
        if (xSemaphoreTake(MotoMonitBianry,portMAX_DELAY) == pdTRUE)
        {
            printf("����������\r\n");
            vTaskPrioritySet(NULL, moto_TASK_PRIO + 1); // ��ߵ�ǰ��������ȼ�
            //����־������ֵ
            if (MotoTaskFlag == 2)// �����������δ���� 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL && eTaskGetState(moto_Task_Handler) != eDeleted)
                {
                    printf("�������\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// �ȴ�MODH_WriteOrReadParam�����ͷ��ź�����ɺ��֪ͨ
                    vTaskSuspend(moto_Task_Handler);
                    printf("�������\r\n");
                }
            } 
            if (MotoTaskFlag == 1)// �ָ���������ѹ��� 
            {
                MotoTaskFlag =0;
                if (eTaskGetState(moto_Task_Handler) == eSuspended) 
                {
                    printf("����ָ�\r\n");
                    vTaskResume(moto_Task_Handler);
                    printf("�ָ����\r\n");
                }
            }
            if (MotoTaskFlag == 3)// �������� 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL) 
                {
                    printf("��������\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //�ȴ��ͷ��ź������
                    vTaskDelete(moto_Task_Handler);
                    moto_Task_Handler = NULL;
                }

                // ���´�������
                xTaskCreate((TaskFunction_t )moto_task,     
                    (const char*    )"moto_task",   
                    (uint16_t       )moto_STK_SIZE, 
                    (void*          )NULL,
                    (UBaseType_t    )moto_TASK_PRIO,
                    (TaskHandle_t*  )&moto_Task_Handler);
                    printf("�������\r\n");
            }

            if (MotoTaskFlag == 4)// ��ֹ���� 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL) 
                {
                    printf("������ֹ\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// �ȴ��ͷ��ź������
                    vTaskDelete(moto_Task_Handler); // ɾ��ָ������
                    printf("����ֹ����\r\n");
                    moto_Task_Handler = NULL;
                }
            }
            vTaskPrioritySet(NULL, uxOriginalPriority); // �ָ�ԭ���ȼ�
        }
    }
}

void InputOutSlic(void *pvParameters)
{
    u8 str[20];
    while(1)
    {
        if (xSemaphoreTake(InputOutSlicBianry,portMAX_DELAY) == pdTRUE)
        {
            if (InputOutSlicFlag == 1)//�û����·�����Ʒ
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,7,0x6002,0x10,0,NULL,MODH_CmdMutex);  //��������е�������Ʒλ��PR0
                WaitMotoStop(7,LSMotoStatus,2);//�ȴ�������������
                
            }
            if (InputOutSlicFlag == 2)//�û�������Ʒ��ɷ���
            {
                SlicSensorCount = 0;
                MODH_WriteOrReadParam(6,7,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������е�PR1
                WaitMotoStop(7,LSMotoStatus,2);//�ȴ����������������������Ƭ��λ��
                for(int i=0;i<19;i++)
                {
                    vTaskDelay(700/portTICK_RATE_MS);//ȷ����Ƭ�ȶ��˲������ٶ�ȡ
                    //SlicSensor1count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
                    SlicSensor2count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
                    SlicSensor3count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
                    if(i<9)//������������ɨ�裬����ߴ�����ɨ���ھŸ���Ƭ֮�󣬾ͻ��ظ�ɨ��ǰ�ߴ�����ɨ���ģ���˵�9����Ƭ֮��ֻ����ǰ�ߴ������ļ���
                    SlicSensorCount+=SlicSensor2count+SlicSensor3count;
                    else
                    SlicSensorCount+=SlicSensor3count;
                    MODH_WriteOrReadParam(6,7,0x6002,0x12,0,NULL,MODH_CmdMutex);//ÿ���ƶ�һ����Ƭ�ļ��
                    WaitMotoStop(7,LSMotoStatus,2);//�ȴ��������
                    printf("��%d�ε�SlicSensorCount=%d��SlicSensor1count=%d��SlicSensor2count=%d��SlicSensor3count=%d\r\n",i,SlicSensorCount,SlicSensor1count,SlicSensor2count,SlicSensor3count);
                }
                
                sprintf(str,"%d",SlicSensorCount);
                SetTextValue(0,4,str);

                MODH_WriteOrReadParam(6,7,0x6002,0X20,0,NULL,MODH_CmdMutex); //����
                WaitMotoStop(7,LSMotoStatus,2);//�ȴ�������������
                InputOutSlicFlag = 0;
            }

            if (InputOutSlicFlag == 3)//����ֽ���Ƭ�ͳ�
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,8,0x6002,0x10,0,NULL,MODH_CmdMutex);  //���������PR0
                WaitMotoStop(8,LSMotoStatus,2);//�ȴ�������������
            }
            if (InputOutSlicFlag == 4)//����ֻ���
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,8,0x6002,0x11,0,NULL,MODH_CmdMutex);  //��������е�PR1
                WaitMotoStop(8,LSMotoStatus,2);//�ȴ�������������
                MODH_WriteOrReadParam(6,8,0x6002,0X20,0,NULL,MODH_CmdMutex); //����
                WaitMotoStop(8,LSMotoStatus,2);//�ȴ�������������
            }
        }
    }
}

void shiyanlicuheng1(void *pvParameters)//ʵ������1
{
    while(1)
    {
        if (xSemaphoreTake(shiyanlicuheng1Bianry,portMAX_DELAY) == pdTRUE)
        {
            if (shiyanlicuheng1Flag == 1)//�û�����ʵ������1
            {
                shiyanlicuheng1Flag = 0;//ʵ������1
            }
        }
    }
}

void TempControl(void *pvParameters)
{
    u8 str[20],i;
    u16 delaytime=500;
    while(1)
    {
        if (xSemaphoreTake(TempControlBianry,portMAX_DELAY) == pdTRUE)
        {
            if(ucg_SendPidFlag==1)//�û����·���PID��ť
            {
                ucg_SendPidFlag=0;
                for(i=0;i<16;i++)
                {
                    MODH_WriteOrReadParam(6,12,i+1,uhwg_SetTempPID[i],0,NULL,MODH_CmdMutex);//�趨������PID
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
            }
            if (TempControlFlag == 1)//�û�������������
            {
                TempControlFlag = 0;
                if(ucg_SetTempFlag[0]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x11,uhwg_SetTemp[0],0,NULL,MODH_CmdMutex);//�趨1�ż������¶�
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x25,1,0,NULL,MODH_CmdMutex);//����1�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[1]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x12,uhwg_SetTemp[1],0,NULL,MODH_CmdMutex);//�趨1�ż������¶�
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x26,1,0,NULL,MODH_CmdMutex);//����2�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[2]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x13,uhwg_SetTemp[2],0,NULL,MODH_CmdMutex);//�趨1�ż������¶�
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x27,1,0,NULL,MODH_CmdMutex);//����3�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }     
            }
            if (TempControlFlag == 2)//�û�����ֹͣ����
            {
                TempControlFlag = 0;
                if(ucg_SetTempFlag[0]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x25,0,0,NULL,MODH_CmdMutex);//ֹͣ1�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[1]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x26,0,0,NULL,MODH_CmdMutex);//ֹͣ2�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[2]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x27,0,0,NULL,MODH_CmdMutex);//ֹͣ3�ż�����
                    vTaskDelay(delaytime/portTICK_RATE_MS); 
                }           
            }
        }
    }
}

void ValvePump(void *pvParameters) 
{
    while(1)
    {
        if (xSemaphoreTake(ValvePumpBianry,portMAX_DELAY) == pdTRUE)
        {
            if(ucg_ValveRunBtn==1)//���±÷����а�ť
            {
                for(u8 i=0;i<45;i++)
                {
                    if (ucg_ValveSwitch[i]==1)
                    {
                        GetControlValue(2,2*i+2);
                        vTaskDelay(70/portTICK_RATE_MS);
                        printf("ѡ�еķ�IDΪ��%d��ѡ��ķ�����Ϊ��%d\r\n",i,ucg_ValveDir[i]);
                        if(SlaveaddressSwitchTest==0)
                        MODH_WriteOrReadParam(6,9,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);//�䶯����ת����
                        if(SlaveaddressSwitchTest==1)
                        MODH_WriteOrReadParam(6,10,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);
                    }
                }
                ucg_ValveRunBtn=0;
            }
            if(ucg_ValveStopBtn==1)//���±÷�ֹͣ��ť
            {
                for(u8 i=0;i<45;i++)
                {
                    if (ucg_ValveSwitch[i]==1)
                    {
                        if(SlaveaddressSwitchTest==0)
                        vTaskDelay(70/portTICK_RATE_MS);
                        MODH_WriteOrReadParam(6,9,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);//�䶯����ת����
                        if(SlaveaddressSwitchTest==1)
                        MODH_WriteOrReadParam(6,10,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);
                    }
                }
                ucg_ValveStopBtn=0;
            }
        } 
    }    
}
  