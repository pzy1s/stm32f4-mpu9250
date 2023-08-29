/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU9250Addr 	0xD0// I2C address of the MPU-9250; 0x68 or 0x69 based on the AD0 pin
#define WHO_AM_I        0x75  // Should return 0x71
#define PWR_MGMT_1      0x6B
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define AK8963_ADDRESS 0x0C   //magnetometer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t accel_gyro_data[14];
uint8_t mag_data[6];
int16_t mag_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
int16_t mag_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};

float pitch = 0, roll = 0, yaw = 0;
float p0 = 1.0f, p1 = 0.0f, p2 = 0.0f, p3 = 0.0f, p4 = 0.0f;
float accel_weight = 0.98f;
float gyro_weight = 0.02f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(HALIIC_WriteByteToSlave(MPU9250Addr,reg,data))
		return 1;
	else
		return 0;
}

uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(HALIIC_ReadByteFromSlave(MPU9250Addr,reg,buf))
		return 1;
	else
		return 0;
}
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_WriteMultByteToSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_ReadMultByteFromSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_INT_PIN_CFG      0x37
 
#define MPU9250_DLPF_BW_256         0x00
#define MPU9250_DLPF_BW_188         0x01
#define MPU9250_DLPF_BW_98          0x02
#define MPU9250_DLPF_BW_42          0x03
#define MPU9250_DLPF_BW_20          0x04
#define MPU9250_DLPF_BW_10          0x05
#define MPU9250_DLPF_BW_5           0x06

void MPU9250_Init(void)
{
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80);//Reset
	HAL_Delay(100);
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x00);
	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00);
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18);// Set gyro full scale to 2000 dps
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x18);// Set accelerometer full scale to 16g
	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_20);
	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x09); // Set frequence to 100Hz
	MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02);
}

#define MPU9250_RA_ACCEL_XOUT_H     0x3B
 
#define MPU9250_RA_TEMP_OUT_H       0x41
 
#define MPU9250_RA_GYRO_XOUT_H      0x43
 
#define AK8963_ADDR			0X0C	
#define AK8963_ID			0X48	
#define MAG_WIA				0x00	
#define MAG_XOUT_L			0X03


void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	uint8_t re = MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
		if(re != HAL_OK){
			printf("Wait MPU9250\n");
		}
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	  uint8_t re = MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
		if(re != HAL_OK){
			printf("Wait MPU9250\n");
		}
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

void MPU9250_MagRead(int16_t *magData)
{
    uint8_t buf[6];
    HALIIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);//turn on Bypass Mode
    HAL_Delay(10);
    HALIIC_WriteByteToSlave(AK8963_ADDRESS,0x0A,0x11);
    HAL_Delay(10);
 
    uint8_t re = HALIIC_ReadMultByteFromSlave(AK8963_ADDRESS,MAG_XOUT_L, 6, buf);
    magData[0] = (int16_t)((buf[1] << 8) | buf[0]) ;
    magData[1] = (int16_t)((buf[3] << 8) | buf[2]) ;
    magData[2] = (int16_t)((buf[5] << 8) | buf[4]) ;
		HALIIC_WriteByteToSlave(AK8963_ADDR,0x0A,0X11);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	MPU9250_Init();
	printf("MPU9250_Init Ok\n");
  int16_t acc[3],gyy[3],mag[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU9250_AccRead(acc);
		MPU9250_GyroRead(gyy);
		MPU9250_MagRead(mag);
    printf("a1 %d\n",acc[0]);
		printf("a2 %d\n",acc[1]);
		printf("a3 %d\n",acc[2]);
		printf("g1 %d\n",gyy[0]);
		printf("g2 %d\n",gyy[1]);
		printf("g3 %d\n",gyy[2]);
		printf("m1 %d\n",mag[0]);
		printf("m2 %d\n",mag[1]);
		printf("m3 %d\n",mag[2]);
		//complementary_filter();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
