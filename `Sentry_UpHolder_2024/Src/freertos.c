/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shoot.h"
#include "usart.h"
#include "tim.h"
#include "Vision.h"
#include "Judge.h"
#include "Holder.h"
#include "chassis.h"
#include "can.h"
#include "BMI088driver.h"
//#include "MahonyAHRS.h"
#include "ins_task.h"
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
  osEvent evt;
	osEvent evt0;
  osEvent evt1;
/* USER CODE END Variables */
osThreadId MSGHandle;
osThreadId _ShootHandle;
osThreadId _HolderMotorHandle;
osThreadId _Chassis_MotorHandle;
osThreadId _UIHandle;
osThreadId _JudgeHandle;
osThreadId _LEDHandle;
osThreadId _BMI088Handle;
osMessageQId RDtHMsgHandle;
osMessageQId RDtCMsgHandle;
osMessageQId RDtSMsgHandle;
osMessageQId GBtHMsgHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void TaskMsg(void const * argument);
void Shoot(void const * argument);
void HolderMotor(void const * argument);
void Chassis_Motor(void const * argument);
void UI(void const * argument);
void Judge(void const * argument);
void LED(void const * argument);
void BMI088Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of RDtHMsg */
  osMessageQDef(RDtHMsg, 16, uint32_t);
  RDtHMsgHandle = osMessageCreate(osMessageQ(RDtHMsg), NULL);

  /* definition and creation of RDtCMsg */
  osMessageQDef(RDtCMsg, 16, uint32_t);
  RDtCMsgHandle = osMessageCreate(osMessageQ(RDtCMsg), NULL);

  /* definition and creation of RDtSMsg */
  osMessageQDef(RDtSMsg, 16, uint32_t);
  RDtSMsgHandle = osMessageCreate(osMessageQ(RDtSMsg), NULL);

  /* definition and creation of GBtHMsg */
  osMessageQDef(GBtHMsg, 16, uint32_t);
  GBtHMsgHandle = osMessageCreate(osMessageQ(GBtHMsg), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MSG */
  osThreadDef(MSG, TaskMsg, osPriorityNormal, 0, 256);
  MSGHandle = osThreadCreate(osThread(MSG), NULL);

  /* definition and creation of _Shoot */
  osThreadDef(_Shoot, Shoot, osPriorityRealtime, 0, 512);
  _ShootHandle = osThreadCreate(osThread(_Shoot), NULL);

  /* definition and creation of _HolderMotor */
  osThreadDef(_HolderMotor, HolderMotor, osPriorityRealtime, 0, 512);
  _HolderMotorHandle = osThreadCreate(osThread(_HolderMotor), NULL);

  /* definition and creation of _Chassis_Motor */
  osThreadDef(_Chassis_Motor, Chassis_Motor, osPriorityNormal, 0, 512);
  _Chassis_MotorHandle = osThreadCreate(osThread(_Chassis_Motor), NULL);

  /* definition and creation of _UI */
  osThreadDef(_UI, UI, osPriorityBelowNormal, 0, 128);
  _UIHandle = osThreadCreate(osThread(_UI), NULL);

  /* definition and creation of _Judge */
  osThreadDef(_Judge, Judge, osPriorityHigh, 0, 1024);
  _JudgeHandle = osThreadCreate(osThread(_Judge), NULL);

  /* definition and creation of _LED */
  osThreadDef(_LED, LED, osPriorityNormal, 0, 256);
  _LEDHandle = osThreadCreate(osThread(_LED), NULL);

  /* definition and creation of _BMI088 */
  osThreadDef(_BMI088, BMI088Task, osPriorityNormal, 0, 128);
  _BMI088Handle = osThreadCreate(osThread(_BMI088), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_TaskMsg */
/**
  * @brief  Function implementing the MSG thread.
  * @param  argument: Not used
  * @retval None
  */
float angle_test[3];
float gyro_test[3];
float temprature;
/* USER CODE END Header_TaskMsg */
void TaskMsg(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN TaskMsg */

  /* Infinite loop */
  for(;;)
  {
//		hh1++;
		  REMOTE_DATA_T RDMsg; /* 遥控器消息 */
      Holder_Data_T GBMsg; /* 云台陀螺仪消息 */
		
		  Msg_RemoteDataProcess(&RDMsg); /*< 遥控器消息解算 */
//			imuCalculateEstimatedAttitude();/*新陀螺仪消息解算*/
//			
//			angle_test[0] = roll + 180.0f;
//			angle_test[1] = pitch + 180.0f;
//			angle_test[2] = yaw + 180.0f;
//			gyro_test[0] = BMI088.Gyro[0];//roll
//			gyro_test[1] = BMI088.Gyro[1];//pitch
//			gyro_test[2] = BMI088.Gyro[2];//yaw
//			temprature = BMI088.Temperature;
		  /* 发送消息队列的数据 */
      osMessagePut(RDtHMsgHandle, (uint32_t)&RDMsg, 0);
      osMessagePut(RDtCMsgHandle, (uint32_t)&RDMsg, 0);
      osMessagePut(RDtSMsgHandle, (uint32_t)&RDMsg, 0);
		  osMessagePut(GBtHMsgHandle, (uint32_t)&GBMsg, 0);
      osDelay(5); /*< 延时1个tick */
  }
  /* USER CODE END TaskMsg */
}

/* USER CODE BEGIN Header_Shoot */
/**
* @brief Function implementing the _Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot */
void Shoot(void const * argument)
{
  /* USER CODE BEGIN Shoot */
	  REMOTE_DATA_T *RDMsg;
    Shoot_PIDInit();
  /* Infinite loop */
  for(;;)
  {
//		hh1++;
		evt = osMessageGet(RDtSMsgHandle, 0);
		if (evt.status == osEventMessage)
		{
			RDMsg = (REMOTE_DATA_T *)evt.value.v;
		}
		Shoot_Process(*RDMsg);
		Shoot_CanTransmit();
    osDelay(1);
  }
  /* USER CODE END Shoot */
}

/* USER CODE BEGIN Header_HolderMotor */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
REMOTE_DATA_T *RDMsg_test;
Holder_Data_T *GBMsg_test;
Chassis_Data_T CAMsg_test;
/* USER CODE END Header_HolderMotor */
void HolderMotor(void const * argument)
{
  /* USER CODE BEGIN HolderMotor */
//    REMOTE_DATA_T *RDMsg;
//	  Holder_Data_T *GBMsg;	
		REMOTE_DATA_T *RDMsg;
	  Holder_PidInit();
  /* Infinite loop */
  for(;;)
  {
		Msg_RemoteDataProcess(RDMsg);
		RDMsg_test = RDMsg;
//    evt0 = osMessageGet(RDtHMsgHandle,0);
//		evt1 = osMessageGet(GBtHMsgHandle,0);
//    if(evt0.status == osEventMessage || evt1.status == osEventMessage)
//     {
//        GBMsg = (Holder_Data_T*)evt1.value.v;
//			  RDMsg = (REMOTE_DATA_T*)evt0.value.v;
//			  RDMsg_test = RDMsg;
//			  GBMsg_test = GBMsg;
//     }
			hh1++; 
			Holder_Process(*RDMsg);
			Holder_CanTransmit();
			osDelay(1);
  }
  /* USER CODE END HolderMotor */
}

/* USER CODE BEGIN Header_Chassis_Motor */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
float energy_timely = 0;
/* USER CODE END Header_Chassis_Motor */
void Chassis_Motor(void const * argument)
{
  /* USER CODE BEGIN Chassis_Motor */
//	 Holder_Data_T GBMsg_Chassis;
  /* Infinite loop */
  for(;;)
  {
//		hh1++;
	  osDelay(10);		
  }
  /* USER CODE END Chassis_Motor */
}

/* USER CODE BEGIN Header_UI */
/**
* @brief Function implementing the _UI thread.
* @param argument: Not used
* @retval None
*/
int32_t tar_angle, cur_angle;
float Judge_power;
float num = 100.0f;
/* USER CODE END Header_UI */
void UI(void const * argument)
{
  /* USER CODE BEGIN UI */
	
  /* Infinite loop */
  for(;;)
  {	
//		hh1++;
		  /* JScope观测波形 */
		tar_angle = Holder.Pitch_0x20B.TarAngle;
		cur_angle = Holder.Pitch_0x20B.IMU_Angle;
		
		  /* 蓝牙函数的使用：观测波形 */
			/*COM8*/
//		UART2_SendWave(5, 4, &Holder.Yaw_0x20A.TarAngle, &Holder.Yaw_0x20A.IMU_Angle, &Holder.Yaw_0x20A.TarSpeed, &Holder.Yaw_0x20A.IMU_Speed, &Holder.Yaw_0x20A.Output); //偏航
//		UART2_SendWave(4, 4, &Holder.Pitch_0x20B.TarAngle, &Holder.Pitch_0x20B.IMU_Angle, &Holder.Pitch_0x20B.TarSpeed, &Holder.Pitch_0x20B.IMU_Speed);//俯仰
//		UART2_SendWave(2,4, &Chassis.M3508[4].TarSpeed,&Chassis.M3508[4].Speed);     //底盘
//		UART2_SendWave(4,4, &Chassis.M3508[1].Speed,&Chassis.M3508[2].Speed,&Chassis.M3508[3].Speed,&Chassis.M3508[4].Speed); 
//		UART2_SendWave(2, 4, &Shoot_M2006.TarAngle, &Shoot_M2006.Angle);  //拨盘
		UART2_SendWave(4, 4, &Holder.Yaw_0x20A.TarAngle, &Holder.Yaw_0x20A.IMU_Angle_Lpf, &Holder.Pitch_0x20B.TarAngle, &Holder.Pitch_0x20B.Motor_Angle); //偏航
//		UART2_SendWave(2, 4, &Holder.Yaw_0x20A.TarAngle, &Holder.Yaw_0x20A.IMU_Angle_Lpf); //偏航
//		UART2_SendWave(2, 4, &Judge_power,&num);
//			UART2_SendWave(1, 1, &Remote_Data.S2);
		osDelay(5);
  }//COM8
  /* USER CODE END UI */
}

/* USER CODE BEGIN Header_Judge */
/**
* @brief Function implementing the _Judge thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_Judge */
void Judge(void const * argument)
{
  /* USER CODE BEGIN Judge */
    Judge_InitData();
  /* Infinite loop */
  for(;;)
  {
//		hh1++;
//		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
	  Judge_Proccess();
		Judge_SendData();
    osDelay(10);
  }
  /* USER CODE END Judge */
}

/* USER CODE BEGIN Header_LED */
/**
* @brief Function implementing the _LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED */
void LED(void const * argument)
{
  /* USER CODE BEGIN LED */
  /* Infinite loop */
  for(;;)
  {
//		hh1++;
		Judge_power = JUDGE_f32GetChassisPower();
		Vision_Judge();
		Vision_Get_cmdkeyboard();
		Vision_SendData();
		DownControl();
		DownControl2();
    osDelay(2);
  }
  /* USER CODE END LED */
}

/* USER CODE BEGIN Header_BMI088Task */
/**
* @brief Function implementing the _BMI088 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088Task */
void BMI088Task(void const * argument)
{
  /* USER CODE BEGIN BMI088Task */
	INS_Init();
  /* Infinite loop */
  for(;;)
  {
//		hh1++;
		INS_Task();
    osDelay(1);
  }
  /* USER CODE END BMI088Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */
