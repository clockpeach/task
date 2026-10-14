/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <queue.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include "oled2.h"
#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
typedef struct {
	float roll;
	float pitch;
	float yaw;
}attitude_angel;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static QueueHandle_t xQueue1;
uint8_t receiveData[50];
uint8_t transmitData[50];
uint16_t  rxLen;
volatile uint8_t rx_data_ready = 0;
uint8_t MPU_DATA[14]={0};
uint8_t StartAddress = 0x3B;
double ACCEL_XOUT =0.0,ACCEL_YOUT =0.0,ACCEL_ZOUT =0.0;
double GYRO_XOUT=0.0,GYRO_YOUT=0.0,GYRO_ZOUT=0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
HAL_UART_Transmit (&huart1,(uint8_t *)&ch,1,0xFFFF );
return ch;
}
void ShowHex(uint8_t *buf,uint8_t len)
{
uint8_t i;
printf("hex = ");
for( i = 0; i < len; i++){
printf(" %02X",buf[i]);
}
printf("\r\n");
}
//#串口重定向
void MPU6050_INIT()
{
vTaskDelay(pdMS_TO_TICKS(1000));
uint8_t SendAddress = 0x6b;
uint8_t SendData = 0x00;
HAL_I2C_Mem_Write(&hi2c2,0xD0,SendAddress,1,&SendData,1,0xff);
SendAddress = 0x19;
SendData = 0x07;
HAL_I2C_Mem_Write(&hi2c2,0xD0,SendAddress,1,&SendData,1,0xff);
SendAddress = 0x1A;
SendData = 0x06;
HAL_I2C_Mem_Write(&hi2c2,0xD0,SendAddress,1,&SendData,1,0xff);
SendAddress = 0x1B;
SendData = 0x08;
HAL_I2C_Mem_Write(&hi2c2,0xD0,SendAddress,1,&SendData,1,0xff);
SendAddress = 0x1C;
SendData = 0x00;
HAL_I2C_Mem_Write(&hi2c2,0xD0,SendAddress,1,&SendData,1,0xff);
}
//#mpu6050初始化
uint8_t tx_busy = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart == &huart1 )
  {
		rxLen = Size;         
    rx_data_ready = 1;  
  }
}
//#串口回调中断，并且置标志位，在对应函数中处理数据
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        tx_busy = 0;
    }
}
//#发送空闲回调函数，置标志位，提示可以发送
void oled(void *pvParameters)
{   
	OLED_Init ();
	OLED_DisPlay_On();
	attitude_angel data;

     for(;;)
     {
      if (xQueueReceive(xQueue1, &data, portMAX_DELAY) == pdTRUE)
      {
	    OLED_NewFrame();
      
			OLED_PrintString(1, 5, "IMU Attitude",  &font16x16, OLED_COLOR_NORMAL);
	    OLED_PrintString(1, 19, "roll",  &font16x16, OLED_COLOR_NORMAL);
	    OLED_PrintString(1, 35, "yaw",  &font16x16, OLED_COLOR_NORMAL);
	    OLED_PrintString(1, 50, "pitch",  &font16x16, OLED_COLOR_NORMAL);

	    OLED_PrintFloat(48, 19,  data.roll  * RAD2DEG, 2, &font16x16, OLED_COLOR_NORMAL);
	    OLED_PrintFloat(48, 35, data.yaw   * RAD2DEG, 2, &font16x16, OLED_COLOR_NORMAL);
	    OLED_PrintFloat(48, 50, data.pitch * RAD2DEG, 2, &font16x16, OLED_COLOR_NORMAL);

	    OLED_ShowFrame();
	  }
	 }
}
	

void serial(void *pvParameters)
{
	HAL_UARTEx_ReceiveToIdle_DMA (&huart1,receiveData ,sizeof(receiveData ));
	
    
	
  for(;;)
	{
	if (rx_data_ready)
        {
            rx_data_ready = 0;                         
            memcpy(transmitData,receiveData,rxLen );
            HAL_UART_Transmit_DMA (&huart1,transmitData,rxLen );
					  
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
	
	vTaskDelay(pdMS_TO_TICKS(100));
	}
 

}
	}

void mpu6050(void *pvParameters)
{   
	  MPU6050_INIT() ;
	float roll,pitch,yaw;
  float roll_acc,pitch_acc;
  float alpha = 0.98f;
	float dt = 0.01f;
	
    for(;;)
	{
	HAL_I2C_Mem_Read(&hi2c2,0xD0,StartAddress,I2C_MEMADD_SIZE_8BIT,MPU_DATA,14,50);
float Temp = (MPU_DATA[6]<<8)|MPU_DATA[7];
if(Temp >32768)
Temp-=65536;
Temp = (36.53+ Temp/340);
short int ACCEL_XOUT1 = ((MPU_DATA[0]<<8)|MPU_DATA[1]);ACCEL_XOUT = (double)ACCEL_XOUT1/16384;
short int ACCEL_YOUT1 = ((MPU_DATA[2]<<8)|MPU_DATA[3]);ACCEL_YOUT = (double)ACCEL_YOUT1/16384;
short int ACCEL_ZOUT1 = ((MPU_DATA[4]<<8)|MPU_DATA[5]);ACCEL_ZOUT = (double)ACCEL_ZOUT1/16384;
short int GYRO_XOUT1 = ((MPU_DATA[8]<<8)|MPU_DATA[9]);GYRO_XOUT = (double)GYRO_XOUT1/65.5* DEG2RAD;
short int GYRO_YOUT1 = ((MPU_DATA[10]<<8)|MPU_DATA[11]);GYRO_YOUT = (double)GYRO_YOUT1/65.5* DEG2RAD;
short int GYRO_ZOUT1 = ((MPU_DATA[12]<<8)|MPU_DATA[13]);GYRO_ZOUT = (double)GYRO_ZOUT1/65.5* DEG2RAD;

  roll_acc  = atan2f(ACCEL_YOUT, ACCEL_ZOUT);
  pitch_acc = atan2f(-ACCEL_XOUT, sqrtf(ACCEL_YOUT*ACCEL_YOUT + ACCEL_ZOUT*ACCEL_ZOUT));

    
    roll  = alpha * (roll  + GYRO_XOUT * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + GYRO_YOUT * dt) + (1.0f - alpha) * pitch_acc;
    yaw   = yaw + GYRO_ZOUT * dt;        


        attitude_angel my;
		my.pitch = pitch;
		my.roll = roll;
		my.yaw = yaw;
		
        xQueueSend(xQueue1, &my, 0);
		
vTaskDelay(pdMS_TO_TICKS(10));
	}

//#读mpu6050数据并且进行高位左移八位拼接,同时通过队列将包含数据的队列发送出去
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
xQueue1 = xQueueCreate(10,sizeof(attitude_angel));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

xTaskCreate (serial,"serial",128,NULL,1,NULL);
xTaskCreate (mpu6050,"imu",128,NULL,1,NULL);
xTaskCreate (oled,"oled",128,NULL,1,NULL);

vTaskStartScheduler ();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
