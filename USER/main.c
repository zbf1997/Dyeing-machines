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
#include "AT24Cxx.h"
#include "shiyanliucheng.h"

int main(void)
{
     
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);
    Uart1Init(115200);//串口1和串口屏通讯   
    Uart3Init(115200);//串口3输出到串口调试助手
	Usart_FIFO_Init(); //串口2用于modbus通讯
    SetScreen(10);
    delay_ms(5000);                                                                  			                                                                                                                                                                               
	KEY_Init();
    Timer5_Init();
    bsp_InitHardTimer();
    PositionInit();
    InputSlicDetect_Init();
    MotoInit();
	AT24CXX_Init();
    TIM_Cmd(TIM5, DISABLE);//定时器5用于RTOS未运行时的定时,防止频繁的中断导致系统变慢
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    MODH_CmdMutex=xSemaphoreCreateMutex();//创建互斥信号量
    UiCmdBianry=xSemaphoreCreateBinary();//创建二值信号量
    MotoMonitBianry=xSemaphoreCreateBinary();
    InputOutSlicBianry=xSemaphoreCreateBinary();
    TempControlBianry=xSemaphoreCreateBinary();
    shiyanlicuheng1Bianry=xSemaphoreCreateBinary();
    ValvePumpBianry=xSemaphoreCreateBinary();
    
    /*******************************************/ 
    //实验流程信号量
    for (u8 i = 0; i < NUM_VATS; i++) 
    {
        xVatMutex[i] = xSemaphoreCreateMutex();//缸体互斥信号
    }
    TakeGetSampleMutex = xSemaphoreCreateMutex();//取放样互斥信号
    xExpCreateMutex = xSemaphoreCreateMutex();//实验流程创建互斥信号
    xTakeInSampleMutex = xSemaphoreCreateMutex();//取样互斥信号
    shiyanliuchengQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(UserCommand));
    /*******************************************/ 

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
#if 0
    xTaskCreate((TaskFunction_t )MotoStausCheck,     	
                (const char*    )"MotoStausCheck",   	
                (uint16_t       )MotoStausCheck_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )MotoStausCheck_TASK_PRIO,	
                (TaskHandle_t*  )&MotoStausCheckTask_Handler);
#endif
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
/*
    xTaskCreate((TaskFunction_t )shiyanlicuheng1,     	
                (const char*    )"shiyanlicuheng1",   	
                (uint16_t       )shiyanlicuheng1_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )shiyanlicuheng1_TASK_PRIO,	
                (TaskHandle_t*  )&shiyanlicuheng1_Handler);
*/
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

    xTaskCreate((TaskFunction_t )CheckAndStartSYliuchengTask,
                (const char*    )"CASSYCmdTask", 
                (uint16_t       )200, 
                (void*          )NULL, 
                (UBaseType_t    )PRIO_LOWEST, 
                (TaskHandle_t*  )&CheckSYliuchengTask_Handler);

    xTaskCreate((TaskFunction_t )CheckAndStartSYliuchengTask,
                (const char*    )"CASSYCmdTask", 
                (uint16_t       )200, 
                (void*          )NULL, 
                (UBaseType_t    )PRIO_LOWEST, 
                (TaskHandle_t*  )&CheckSYliuchengTask_Handler);
    Him_Init();
    SetScreen(2);
    vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

void Process_message(void *pvParameters)
{
	qsize  size = 0;
    BaseType_t xReturn;
	while(1)
	{
        xReturn = xSemaphoreTake(UiCmdBianry,portMAX_DELAY); /* 获取信号量 */
        if (xReturn == pdTRUE)
        {
            size = queue_find_cmd(cmd_buffer,CMD_MAX_SIZE);
            if(size>0&&cmd_buffer[1]!=0x07)                                              //如果队列不为空且指令不为开机指令
            {                                                                           
                ProcessMessage((PCTRL_MSG)cmd_buffer, size);                             //对串口屏指令进行处理  
            }                                                                           
            else if(size>0&&cmd_buffer[1]==0x07)                                           
            {                                                                           
                // __disable_fault_irq();//如果串口屏指令为0x07开机指令说明串口屏重启了，此时对STM32进行软件重启，再进行一次软硬件的初始化                                                   
                // NVIC_SystemReset();

                Him_Init(); //在freertos下对stm32进行软重启会造成死机，暂时未找到原因,采用重新初始化串口屏的方式，将存储的数据显示
                SetScreen(2); //重新设置串口屏界面
            }
        }
	}
}  

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

//查询状态的任务优先级要比其他任务低，确保不抢占共享资源，因为查询指令使用modbus总线，电机运动也使用modbus总线，如果查询状态的任务优先级高会导致电机运动变慢迟缓
void MotoStausCheck(void *pvParameters)
{
    u8 str[20];
    u8 delaytime=150;
    TickType_t xLastTime, xLastTime1;
    while(1)
    {
        /*
        MODH_WriteOrReadParam(3,1,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询X1的绝对位置
        MODH_WriteOrReadParam(3,2,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询Y1的绝对位置
        MODH_WriteOrReadParam(3,3,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询Z1的绝对位置
        MODH_WriteOrReadParam(3,4,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询X2的绝对位置
        MODH_WriteOrReadParam(3,5,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询Y2的绝对位置
        MODH_WriteOrReadParam(3,6,LSMotoLocation,0,2,NULL,MODH_CmdMutex);//查询Z2的绝对位置
        MODH_WriteOrReadParam(3,1,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询X1的运行状态
        MODH_WriteOrReadParam(3,2,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询Y1的运行状态
        MODH_WriteOrReadParam(3,3,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询Z1的运行状态
        MODH_WriteOrReadParam(3,4,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询X2的运行状态
        MODH_WriteOrReadParam(3,5,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询Y2的运行状态
        MODH_WriteOrReadParam(3,6,LSMotoStatus,0,2,NULL,MODH_CmdMutex);//查询Z2的运行状态
        delay_ms(3000);
        

        // 将MotoLocation数组中的值转换为字符串，并设置到界面上
        sprintf(str,"%d",MotoLocation[0]);
        SetTextValue(6,37,str);
        SetTextValue(7,37,str);
        sprintf(str,"%d",MotoLocation[1]);
        SetTextValue(6,38,str);
        SetTextValue(7,38,str);
        sprintf(str,"%d",MotoLocation[2]);
        SetTextValue(6,39,str);
        SetTextValue(7,39,str);
        sprintf(str,"%d",MotoLocation[3]);
        SetTextValue(6,76,str);
        SetTextValue(7,76,str);
        sprintf(str,"%d",MotoLocation[4]);
        SetTextValue(6,77,str);
        SetTextValue(7,77,str);
        sprintf(str,"%d",MotoLocation[5]);
        SetTextValue(6,78,str);
        SetTextValue(7,78,str);
        */
        
        if (xTaskGetTickCount() - xLastTime > pdMS_TO_TICKS(3000)) 
        {
            /*
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x15,0,1,NULL,MODH_CmdMutex);//查询1号探头温度
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x16,0,1,NULL,MODH_CmdMutex);//查询2号探头温度
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x17,0,1,NULL,MODH_CmdMutex);//查询3号探头温度
            delay_ms(500);
            MODH_WriteOrReadParam(3,12,0x18,0,1,NULL,MODH_CmdMutex);//查询4号探头温度
            delay_ms(500);
            sprintf(str,"%.1f",uhwg_RealTemp[0]/10.0);
            SetTextValue(8,9,str);
            printf("1号温控的实时温度为：%d\r\n",uhwg_RealTemp[0]);
            sprintf(str,"%.1f",uhwg_RealTemp[1]/10.0);
            SetTextValue(8,17,str);
            printf("2号温控的实时温度为：%d\r\n",uhwg_RealTemp[1]);
            sprintf(str,"%.1f",uhwg_RealTemp[2]/10.0);
            SetTextValue(8,28,str);
            printf("3号温控的实时温度为：%d\r\n",uhwg_RealTemp[2]);
            sprintf(str,"%.1f",uhwg_RealTemp[3]/10.0);
            SetTextValue(8,81,str);
            printf("3号温控的实时温度为：%d\r\n",uhwg_RealTemp[3]);
            */
            printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());
            char pcBuffer[1000];  
            vTaskList(pcBuffer);  // 生成任务状态表格
            printf("任务状态:\n%s\r\n", pcBuffer);  
            xLastTime = xTaskGetTickCount();  
        } 
    }
}

void MonitorTasks(void *pvParameters)
{
    u8 str[20];
    UBaseType_t uxOriginalPriority = uxTaskPriorityGet(NULL); // 获取当前任务的优先级
	while(1)
	{
        if (xSemaphoreTake(MotoMonitBianry,portMAX_DELAY) == pdTRUE)
        {
            printf("进入监控任务\r\n");
            vTaskPrioritySet(NULL, moto_TASK_PRIO + 1); // 提高当前任务的优先级
            //检测标志变量的值
            if (MotoTaskFlag == 2)// 挂起任务（如果未挂起） 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL && eTaskGetState(moto_Task_Handler) != eDeleted)
                {
                    printf("进入挂起\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// 等待MODH_WriteOrReadParam函数释放信号量完成后的通知
                    vTaskSuspend(moto_Task_Handler);
                    printf("挂起完成\r\n");
                }
            } 
            if (MotoTaskFlag == 1)// 恢复任务（如果已挂起） 
            {
                MotoTaskFlag =0;
                if (eTaskGetState(moto_Task_Handler) == eSuspended) 
                {
                    printf("进入恢复\r\n");
                    vTaskResume(moto_Task_Handler);
                    printf("恢复完成\r\n");
                }
            }
            if (MotoTaskFlag == 3)// 重启任务 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL) 
                {
                    printf("进入重启\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //等待释放信号量完成
                    vTaskDelete(moto_Task_Handler);
                    moto_Task_Handler = NULL;
                }

                // 重新创建任务
                xTaskCreate((TaskFunction_t )moto_task,     
                    (const char*    )"moto_task",   
                    (uint16_t       )moto_STK_SIZE, 
                    (void*          )NULL,
                    (UBaseType_t    )moto_TASK_PRIO,
                    (TaskHandle_t*  )&moto_Task_Handler);
                    printf("重启完成\r\n");
            }

            if (MotoTaskFlag == 4)// 终止任务 
            {
                MotoTaskFlag =0;
                if (moto_Task_Handler != NULL) 
                {
                    printf("进入终止\r\n");
                    xTaskNotifyGive(moto_Task_Handler);
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// 等待释放信号量完成
                    vTaskDelete(moto_Task_Handler); // 删除指定任务
                    printf("已终止任务\r\n");
                    moto_Task_Handler = NULL;
                }
            }
            vTaskPrioritySet(NULL, uxOriginalPriority); // 恢复原优先级
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
            if (InputOutSlicFlag == 1)//用户按下放入样品
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,7,0x6002,0x10,0,NULL,MODH_CmdMutex);  //输入仓运行到接收样品位置PR0
                WaitMotoStop(7,LSMotoStatus,2);//等待输入仓运行完成
                
            }
            if (InputOutSlicFlag == 2)//用户按下样品完成放入
            {
                SlicSensorCount = 0;
                MODH_WriteOrReadParam(6,7,0x6002,0x11,0,NULL,MODH_CmdMutex);  //输入仓运行到PR1
                WaitMotoStop(7,LSMotoStatus,2);//等待输入仓运行至结束计数玻片的位置
                for(int i=0;i<19;i++)
                {
                    vTaskDelay(700/portTICK_RATE_MS);//确保玻片稳定了不抖动再读取
                    //SlicSensor1count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
                    SlicSensor2count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
                    SlicSensor3count=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
                    if(i<9)//用两个传感器扫描，当后边传感器扫到第九个玻片之后，就会重复扫到前边传感器扫过的，因此第9个玻片之后只计算前边传感器的计数
                    SlicSensorCount+=SlicSensor2count+SlicSensor3count;
                    else
                    SlicSensorCount+=SlicSensor3count;
                    MODH_WriteOrReadParam(6,7,0x6002,0x12,0,NULL,MODH_CmdMutex);//每次移动一个玻片的间隔
                    WaitMotoStop(7,LSMotoStatus,2);//等待运行完成
                    printf("第%d次的SlicSensorCount=%d，SlicSensor1count=%d，SlicSensor2count=%d，SlicSensor3count=%d\r\n",i,SlicSensorCount,SlicSensor1count,SlicSensor2count,SlicSensor3count);
                }
                
                sprintf(str,"%d",SlicSensorCount);
                SetTextValue(0,4,str);

                MODH_WriteOrReadParam(6,7,0x6002,0X20,0,NULL,MODH_CmdMutex); //回零
                WaitMotoStop(7,LSMotoStatus,2);//等待输入仓运行完成
                InputOutSlicFlag = 0;
            }

            if (InputOutSlicFlag == 3)//输出仓将玻片送出
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,8,0x6002,0x10,0,NULL,MODH_CmdMutex);  //输出仓运行PR0
                WaitMotoStop(8,LSMotoStatus,2);//等待输入仓运行完成
            }
            if (InputOutSlicFlag == 4)//输出仓回零
            {
                InputOutSlicFlag = 0;
                MODH_WriteOrReadParam(6,8,0x6002,0x11,0,NULL,MODH_CmdMutex);  //输出仓运行到PR1
                WaitMotoStop(8,LSMotoStatus,2);//等待输出仓运行完成
                MODH_WriteOrReadParam(6,8,0x6002,0X20,0,NULL,MODH_CmdMutex); //回零
                WaitMotoStop(8,LSMotoStatus,2);//等待输出仓运行完成
            }
        }
    }
}

void shiyanlicuheng1(void *pvParameters)//实验流程1
{
    u16 fanyingTime;//试剂反应时间
    while(1)
    {
        if (xSemaphoreTake(shiyanlicuheng1Bianry,portMAX_DELAY) == pdTRUE)
        {
            if(ucg_X3Y3Z3A3RunBtn==1)
            {
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
            if(kaopian[0]==1)
            {
                StainingPodStatus[29]=1;//假设输入仓有样品
                TakeGetSample(1,29,0,0);
                TakeGetSample(2,28,0,0);
                vTaskDelay((kaopian[1]*1000)/portTICK_RATE_MS);
                TakeGetSample(1,28,0,0);
            }
            else 
            {
                StainingPodStatus[29]=1;//假设输入仓有样品
                TakeGetSample(1,29,0,0);
            }
            for(u8 i=0;i<40;i++)
            {
                if (shiyan1Param[i][0]!= 0)
                {
                    if(shiyan1Param[i][1]<=(ActionTime1+ActionTime2+ActionTime3))//反应时间小于等于机械臂将样品放入试剂后的复位时间
                    { 
                        TakeGetSampleNoCloseCap(2,TankAndReagenMapping(shiyan1Param[i][0]),0,0);
                        if(shiyan1Param[i][1]>ActionTime3)
                        {
                            fanyingTime=shiyan1Param[i][1]-ActionTime3;
                            vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                        }
                        TakeGetSampleNoCloseCap(1,TankAndReagenMapping(shiyan1Param[i][0]),1,Z1Shaketime);
                    }
                    else
                    {
                        TakeGetSample(2,TankAndReagenMapping(shiyan1Param[i][0]),0,0);
                        fanyingTime=shiyan1Param[i][1]-(ActionTime1+ActionTime2+ActionTime3);
                        vTaskDelay((fanyingTime*1000)/portTICK_RATE_MS);
                        TakeGetSample(1,TankAndReagenMapping(shiyan1Param[i][0]),1,Z1Shaketime);
                    }   
                }
            }
            TakeGetSample(2,32,0,0);//将样品放到输出仓

            /*X2移动到26号缸再进行复位，防止在某些位置复位时撞到盖臂*/
            MODH_WriteOrReadParam(6,4,0x6201,GET_2BYTE_H(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//设定PR0位置高位
            MODH_WriteOrReadParam(6,4,0x6202,GET_2BYTE_L(uhwg_UncapPosition_Compose[25][0]),0,NULL,MODH_CmdMutex);//设定PR0位置低位
            MODH_WriteOrReadParam(6,4,0x6002,0x10,0,NULL,MODH_CmdMutex);  //立即运行PR0
            WaitMotoStop(4,LSMotoStatus,2);//等待X2轴完成

            SetScreen(11);//流程结束弹出取走样品对话框
            StainingPodStatus[32]=0;//恢复输出仓空闲状态

            if(ucg_X3Y3Z3A3RunBtn==1)//如果启动混匀电机
            {
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
            if(ucg_SendPidFlag==1)//用户按下发送PID按钮
            {
                ucg_SendPidFlag=0;
                for(i=0;i<16;i++)
                {
                    MODH_WriteOrReadParam(6,12,i+1,uhwg_SetTempPID[i],0,NULL,MODH_CmdMutex);//设定加热器PID
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
            }
            if (TempControlFlag == 1)//用户按下启动加热
            {
                TempControlFlag = 0;
                if(ucg_SetTempFlag[0]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x11,uhwg_SetTemp[0],0,NULL,MODH_CmdMutex);//设定1号加热器温度
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x25,1,0,NULL,MODH_CmdMutex);//启动1号加热器
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[1]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x12,uhwg_SetTemp[1],0,NULL,MODH_CmdMutex);//设定1号加热器温度
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x26,1,0,NULL,MODH_CmdMutex);//启动2号加热器
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[2]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x13,uhwg_SetTemp[2],0,NULL,MODH_CmdMutex);//设定1号加热器温度
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                    MODH_WriteOrReadParam(6,12,0x27,1,0,NULL,MODH_CmdMutex);//启动3号加热器
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }     
            }
            if (TempControlFlag == 2)//用户按下停止加热
            {
                TempControlFlag = 0;
                if(ucg_SetTempFlag[0]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x25,0,0,NULL,MODH_CmdMutex);//停止1号加热器
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[1]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x26,0,0,NULL,MODH_CmdMutex);//停止2号加热器
                    vTaskDelay(delaytime/portTICK_RATE_MS);
                }
                if(ucg_SetTempFlag[2]==1)
                {
                    MODH_WriteOrReadParam(6,12,0x27,0,0,NULL,MODH_CmdMutex);//停止3号加热器
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
            if(ucg_ValveRunBtn==1)//按下泵阀运行按钮
            {
                for(u8 i=0;i<45;i++)
                {
                    if (ucg_ValveSwitch[i]==1)
                    {
                        GetControlValue(1,2*(i+1));//获取ucg_ValveDir[i]的值
                        vTaskDelay(70/portTICK_RATE_MS);
                        if(SlaveaddressSwitchTest==0)
                        MODH_WriteOrReadParam(6,9,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);//阀运行
                        if(SlaveaddressSwitchTest==1)
                        MODH_WriteOrReadParam(6,10,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);//泵运行
                    }
                }
                ucg_ValveRunBtn=0;
            }
            if(ucg_ValveStopBtn==1)//按下泵阀停止按钮
            {
                for(u8 i=0;i<45;i++)
                {
                    if (ucg_ValveSwitch[i]==1)
                    {
                        if(SlaveaddressSwitchTest==0)
                        vTaskDelay(70/portTICK_RATE_MS);
                        MODH_WriteOrReadParam(6,9,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);
                        if(SlaveaddressSwitchTest==1)
                        MODH_WriteOrReadParam(6,10,i,ucg_ValveDir[i],0,NULL,MODH_CmdMutex);
                    }
                }
                ucg_ValveStopBtn=0;
            }
        } 
    }    
}
