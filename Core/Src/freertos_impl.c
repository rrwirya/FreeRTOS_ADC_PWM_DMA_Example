
/**
  **************************************************************************************************
  * @file       : freertos_impl.c
  * @brief      : This file contains all FreeRTOS object initializations and relevant
	*							  tasks that will be running in the background.
  * @author			: 
  **************************************************************************************************
  */
  
  
/* Includes --------------------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"


/* Private includes ------------------------------------------------------------------------------*/
#include "freertos_impl.h"


/* Private typedef -------------------------------------------------------------------------------*/


/* Private define --------------------------------------------------------------------------------*/
/* Set below value to 1 to update TIM3->CCR2 register (that controls the PWM output) through DMA */
#define PWM_DMA_ON											1


/* Private macro ---------------------------------------------------------------------------------*/


/* Exported/Global variables ---------------------------------------------------------------------*/


/* External variables ----------------------------------------------------------------------------*/
	/*--- Used STM32 peripherals ---*/
	extern ADC_HandleTypeDef hadc1;
	extern TIM_HandleTypeDef htim3;
	

/* Private variables -----------------------------------------------------------------------------*/
	/*--- FreeRTOS Timer Handles ---*/
	
	
	/*--- FreeRTOS Task Handles ---*/
	TaskHandle_t h_TaskMCUPB;
	static TaskHandle_t h_TaskADCPWM;
	static TaskHandle_t h_TaskMCULED;
	
	/*--- Private variables for h_TaskADCPWM ---*/
	__IO uint32_t RawAdcValue = 0;				/* Variable that will contain raw value read from ADC */	
	__IO uint32_t RawPWMThresh = 0;				/* Variable that will be linked to PWM DMA */
	
	/*--- Refer to FreeRTOS documentation on uxTaskGetStackHighWaterMark(NULL) ---*/
	UBaseType_t TaskMCUPB_RSS, TaskADCPWM_RSS, TaskMCULED_RSS;

	/*--- DWT Debug Variables ---*/
	__IO unsigned long Time_t1 = 0;
	__IO unsigned long Time_t2 = 0;
	__IO unsigned long Time_tDiff = 0;		/* tDiff[clock_cycles] = t2 - t1. Divide tDiff/CORECLOCK to get time in seconds */
	

/* Private function prototypes -------------------------------------------------------------------*/
static void Task_ProcessPushButton(void *argument);
static void Task_ReadADCControlPWM(void *argument);
static void Task_BlinkMcuLED(void *argument);


/* Private user code -----------------------------------------------------------------------------*/

/**
  **************************************************************************************************
  * FreeRTOS Object Initializations																																 *
  **************************************************************************************************
  */
  
/**
 * @brief	All FreeRTOS Mutexes, Semaphores, Timers, Queues, and Task creations below
 * @note	Use assert_param to ensure that all creations/inits were successful
 */
 
void FRTOS_Init_Mutex(void)
{
	
	
	
}

void FRTOS_Init_SWTimers(void)
{
	
	
	
}

void FRTOS_Init_Queues(void)
{
	
	
	
}

void FRTOS_Init_Tasks(void)
{
	BaseType_t TaskCreationStatus;
	
	/* Task that process push button interrupts to suspend Task2 - ADCPWM */
	TaskCreationStatus = xTaskCreate( Task_ProcessPushButton,
																		"TASK1 - MCU PB",
																		TASK_STACKSIZE_DEFAULT,
																		NULL,
																		TASK_PRIO_MCUPB,
																		&h_TaskMCUPB);
	
	/* Ensure that task creation was successful */
	assert_param(TaskCreationStatus == pdPASS);	
	
	/* Task that records ADC conversions (through DMA) and modifies PWM (through DMA) */
	TaskCreationStatus = xTaskCreate( Task_ReadADCControlPWM,
																		"Task2 - ADCPWM",
																		TASK_STACKSIZE_DEFAULT,
																		NULL,
																		TASK_PRIO_ADCPWM,
																		&h_TaskADCPWM);
	
	/* Ensure that task creation was successful */
	assert_param(TaskCreationStatus == pdPASS);
	
	/* Task that will blink MCU LED periodically */
	TaskCreationStatus = xTaskCreate( Task_BlinkMcuLED,
																		"Task3 - MCULED",
																		TASK_STACKSIZE_MIN,
																		NULL,
																		TASK_PRIO_MCULED,
																		&h_TaskMCULED);
	
	/* Ensure that task creation was successful */
	assert_param(TaskCreationStatus == pdPASS);
}


/**
  **************************************************************************************************
  * FreeRTOS Tasks in ascending priority (and Timer Callbacks)																		 *
  **************************************************************************************************
  */

/**
 * @brief	This task will await GPIO interrupt notification, and suspend/resume Task2
 */
static void Task_ProcessPushButton(void *argument)
{
	/* Variable declarations */
	uint32_t NotificationValue = 0;
	
	while(1)
	{
		/* Block indefinitely until GPIO Interrupt Callback Handler issues a notification
		   to this task */
		NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		/* Calculate remaining stack size for this task upon leaving blocked state */
		TaskMCUPB_RSS = uxTaskGetStackHighWaterMark(NULL);
		
		if(NotificationValue & NOTIF_TASK_MCUPB_RESUME_OTHER_TASK)
		{
			/* Resume task that samples ADC and changes external LED brightness */
			vTaskResume(h_TaskMCULED);
		}
		else if(NotificationValue & NOTIF_TASK_MCUPB_SUSPEND_OTHER_TASK)
		{
			/* Suspend task that samples ADC and changes external LED brightness until another
			   push button interrupt occurs */
			vTaskSuspend(h_TaskMCULED);
		}
		
	}
	
	/* Automatically delete task if code ever reaches here */
	vTaskDelete(NULL);
}

/**
 * @brief	This task reads the ADC conversion value every 100ms, and feeds it into the PWM
 */
static void Task_ReadADCControlPWM(void *argument)
{
	/* Variable declarations */
	const TickType_t DelayFrequency = pdMS_TO_TICKS(50);
	TickType_t LastActiveTime;
	
	/* Initialize ADC with DMA, to automatically read and store ADC conversions into a variable */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&RawAdcValue, 1);
	
#if PWM_DMA_ON
	/* Initialize PWM CHANNEL2 with DMA, to automatically change TIMx->CCR by updating a variable */
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*)&RawPWMThresh, 1);
#else
	/* If DMA is not used, user must update TIMx->CCRy manually to alter duty cycle */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
#endif
	
		/*--- Initialize DWT ---*/
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		/* Start Clock Cycle Counter at 0 */
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		
	vTaskDelay(20);
	
	while(1)
	{
		/* Record last wakeup time and use it to perform blocking delay the next 90ms */
		LastActiveTime = xTaskGetTickCount();
		vTaskDelayUntil(&LastActiveTime, DelayFrequency);
		
		/* Calculate remaining stack size for this task upon leaving blocked state */
		TaskADCPWM_RSS = uxTaskGetStackHighWaterMark(NULL);
		
		/* Perform scaling conversion based on ADC input, and feed value into PWM CCR register */
#if PWM_DMA_ON
		Time_t1 = DWT->CYCCNT;
		
		/* Update CCR register through DMA */
		RawPWMThresh = (uint16_t)((RawAdcValue * MAX_TIM3_PWM_VALUE)/MAX_ADC_12BIT_VALUE);
		
		Time_t2 = DWT->CYCCNT;
		Time_tDiff = Time_t2 - Time_t1;
#else
		Time_t1 = DWT->CYCCNT;
		
		/* Update CCR register manually */
		TIM3->CCR2 = (uint16_t)((RawAdcValue * MAX_TIM3_PWM_VALUE)/MAX_ADC_12BIT_VALUE);
		
		Time_t2 = DWT->CYCCNT;
		Time_tDiff = Time_t2 - Time_t1;
#endif
	}
	
	/* Automatically delete task if code ever reaches here */
	vTaskDelete(NULL);
}


/**
 * @brief	This task toggles onboard microcontroller LED 
 */
static void Task_BlinkMcuLED(void *argument)
{
	/* Variable declarations for performing periodic blocking delays */
	const TickType_t DelayFrequency = pdMS_TO_TICKS(1000);
	TickType_t LastActiveTime;
	
	while(1)
	{		
		/* Record last wakeup time and use it to perform blocking delay the next 1 second */
		LastActiveTime = xTaskGetTickCount();
		vTaskDelayUntil(&LastActiveTime, DelayFrequency);
	
		/* Calculate remaining stack size for this task upon leaving blocked state */
		TaskMCULED_RSS = uxTaskGetStackHighWaterMark(NULL);
		
		/* Toggle onboard MCU LED upon exiting blocked delay */
		HAL_GPIO_TogglePin(NUCLEO_GLED_GPIO_Port, NUCLEO_GLED_Pin);
	}
	
	/* Automatically delete task if code ever reaches here */
	vTaskDelete(NULL);
}

/******************************************* END OF FILE *******************************************/

