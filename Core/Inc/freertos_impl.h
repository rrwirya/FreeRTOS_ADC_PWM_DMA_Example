
/**
  **************************************************************************************************
  * @file           : freertos_impl.h
  * @brief          : Header for freertos_impl.c file.
  *                   <Insert description here>
  * @author         :
  **************************************************************************************************
  */


/* Define to prevent recursive inclusion ---------------------------------------------------------*/
#ifndef __FREERTOS_IMPL_H
#define __FREERTOS_IMPL_H


#ifdef __cplusplus
extern "C" {
#endif


/* Includes --------------------------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"


/* Exported types --------------------------------------------------------------------------------*/


/* Exported variables ----------------------------------------------------------------------------*/
	extern __IO uint32_t RawPWMThresh;
	extern __IO unsigned long Time_tDiff;
	

/* Exported defines ------------------------------------------------------------------------------*/
	/*--- Task Stack Sizes used for xTaskCreate APIs (number should be defined in words or 16-bits) ---*/
	#define TASK_STACKSIZE_DEFAULT						(256 * 4)				
	#define TASK_STACKSIZE_MIN								(64 * 4)

	/*--- Task Priorities ---*/
	#define TASK_PRIO_MCUPB											osPriorityNormal7
	#define TASK_PRIO_ADCPWM										osPriorityNormal5
	#define TASK_PRIO_MCULED										osPriorityNormal2
	
	/*--- Notification Values ---*/
	#define NOTIF_TASK_MCUPB_RESUME_OTHER_TASK		((uint16_t)0x0001)
	#define NOTIF_TASK_MCUPB_SUSPEND_OTHER_TASK		((uint16_t)0x0002)
	
	/*--- Misc. Exported Defines ---*/
	#define MAX_ADC_12BIT_VALUE								((uint16_t)4096)		/* Based on 12-bit ADC resolution */
	#define MAX_TIM3_PWM_VALUE								((uint16_t)1000)		/* Based on prescaler and period */


/* Exported constants ----------------------------------------------------------------------------*/


/* Exported macro --------------------------------------------------------------------------------*/


/* Exported Functions Prototypes -----------------------------------------------------------------*/
void FRTOS_Init_Mutex(void);
void FRTOS_Init_SWTimers(void);
void FRTOS_Init_Queues(void);
void FRTOS_Init_Tasks(void);


#ifdef __cplusplus 
}
#endif



#endif  /* __FREERTOS_IMPL_H */


/******************************************* END OF FILE *******************************************/

