/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include <stdio.h>
#include"bmp180_internals.h"
#include "bmp180.h"
#include "i2c.h"
#include "usart.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
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
  float temperature;  //sensor data
  int32_t pressure;
  float altitude;

  float initialAltitude; //for altitude calculation
  float maxAltitude;

  char txdata[150]; //data holder

  int32_t measurementTime; //for velocity calculation
  int32_t deltaTime;
  float deltaAltitude;
  float velocity;

  uint8_t estApo;  //for time estimation
  uint8_t estLand;

  bool check= false;
//oss setting of bmp180
  bmp180_t bmp180 = {.oversampling_setting = standart};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AcquireData */
osThreadId_t AcquireDataHandle;
const osThreadAttr_t AcquireData_attributes = {
  .name = "AcquireData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Recovery */
osThreadId_t RecoveryHandle;
const osThreadAttr_t Recovery_attributes = {
  .name = "Recovery",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */



	// Wait till initialization is complete
  while (bmp180_init(&hi2c1,&bmp180));
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AcquireData */
  AcquireDataHandle = osThreadNew(StartTask02, NULL, &AcquireData_attributes);

  /* creation of Recovery */
  RecoveryHandle = osThreadNew(StartTask03, NULL, &Recovery_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	//set the mean of first few data to the max
	for(int i=0;i<100;i++)
		{   //combine the values of altitude
		bmp180_get_altitude(&bmp180);
			initialAltitude+= bmp180.altitude;
		}
		    // divide by the number of terms to get the mean
	        initialAltitude/=100;
	        maxAltitude= initialAltitude;

  /* Infinite loop */
  for(;;)
  {
	  // Get all the values
	 	  bmp180_get_all(&bmp180);
	 	  measurementTime = HAL_GetTick();
	 	  altitude = bmp180.altitude;
	 	  temperature = bmp180.temperature;
	 	  pressure = bmp180.pressure;

	 	  //adjust the maximum altitude
	 	  if (maxAltitude < altitude)
	 	  {
	 	  	  maxAltitude = altitude;
	 	  }

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	//set the mean of first few data to the max
	int j=0;
	uint32_t k=1;
  /* Infinite loop */
  for(;;)
  {

	  //transmit the acquired data with UART
	  sprintf(txdata, "%d-) Pressure: %ld Pa\nAltitude: %f m\nTemperature: %f C\n",k, pressure, altitude,temperature);
	  HAL_UART_Transmit(&huart2, txdata, strlen(txdata), 100);

          //calculate velocity with the altitude
	      bmp180_get_altitude(&bmp180);
	  	  deltaTime=HAL_GetTick()-measurementTime;
	  	  deltaAltitude=bmp180.altitude - altitude;
	  	  velocity = deltaAltitude/deltaTime;
	  	  //estimate the time for apogee (works only when out of fuel and friction is neglected)
	  	  estApo=velocity/9.8;
	  	  //estimate the time for landing after apogee (with similar assumptions)
	  	  estLand=sqrt(2*maxAltitude/9.8)-j;
	  if(!check)
	  {
	   sprintf(txdata, "Current velocity: %f m/s\nEstimated time for apogee:%d s\n",velocity,estApo);
	   HAL_UART_Transmit(&huart2, txdata, strlen(txdata), 100);
	  }
	  else if(estLand>=0)
	  {
	   sprintf(txdata, "Current velocity: %f m/s\nEstimated time for landing:%d s\n ",velocity,estLand);
	   HAL_UART_Transmit(&huart2, txdata, strlen(txdata), 100);
		j++;
	  }
	  	  k++;
    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ApogeeDetect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */


  /* Infinite loop */
  for(;;)
  {
	  //checks if the device is falling down
	  if ((maxAltitude - altitude) > 1)
	      { //represents drag parachute trigger
	        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	        check=true;
	      }
	  //checks if the minimum altitude limit has been reached
	  if(check && altitude<=30)
	      {
	        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	      }

    osDelay(1000);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

