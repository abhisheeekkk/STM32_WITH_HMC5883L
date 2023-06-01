/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Abhishek Shukla(Github - @abhisheeekkk)
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
 /*
	I was stuck on this for too long, had to connect around 4K resistor to SCL line externally, even if internally it is pulled up.
	wiring is :


  PB8 - SCL (I2C1_SCL)
  PB9 - SDA (I2C1_SDA)
  5V - for HMC5883L
  GND - GND

  ALSO EXTERNALL RESISTOR OF 4k OHM TO SCL AND VCC
  */

/*For logic Analyzer Wiring is :
 *
 * GND TO CH8
 * SDA TO CH1
 * SCL TO CH2
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// variables for receiving DATA

// adresses
// HMC5883l - ADDRESS
//#define HMC5883l_ADDRESS (0x1E << 1)
//// CONTROL REG A
//#define HMC5883l_Enable_A (0x78)
//// CONTROL REG B
//#define HMC5883l_Enable_B (0xA0)
//// MODE REGISTER
//#define HMC5883l_MR (0x00)
//// HMC5883l - MSB / LSB ADDRESSES
//#define HMC5883l_ADD_DATAX_MSB (0x03)
//#define HMC5883l_ADD_DATAX_LSB (0x04)
//#define HMC5883l_ADD_DATAZ_MSB (0x05)
//#define HMC5883l_ADD_DATAZ_LSB (0x06)
//#define HMC5883l_ADD_DATAY_MSB (0x07)
//#define HMC5883l_ADD_DATAY_LSB (0x08)
//// SUM (MSB + LSB) DEFINE
//#define HMC5883l_ADD_DATAX_MSB_MULTI (HMC5883l_ADD_DATAX_MSB | 0x80)
//#define HMC5883l_ADD_DATAY_MSB_MULTI (HMC5883l_ADD_DATAY_MSB | 0x80)
//#define HMC5883l_ADD_DATAZ_MSB_MULTI (HMC5883l_ADD_DATAZ_MSB | 0x80

#define HMC5883L_ADDRESS    0x1E

#define HMC5883L_REG_CONFIGA    0x00
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA       0x03

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Initializes the HMC5883L sensor in continuous mode, By writing 0x00 at 0x02 register.
void HMC5883L_Init(void)
{
    uint8_t config_data = 0x00;// << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1); // Continuous measurement mode, 8-sample average, 15Hz data output rate
//
    HAL_StatusTypeDef st;
    uint8_t kk = 0x1E;
    uint8_t pp = 0x02;

    uint8_t data[2];

    data[0] = 0x02;     // 0x0C in your example
    data[1] = 0x00;    // MSB byte of 16bit data
//    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS<<1, HMC5883L_REG_MODE, I2C_MEMADD_SIZE_8BIT, &config_data, 1, 1000);
//
    st = HAL_I2C_Master_Transmit(&hi2c1, 0x1E<<1, data, 2, HAL_MAX_DELAY); // SET DEVICE MODE - continuous mode


//     HAL_I2C_Master_Transmit(&hi2c1, kk<<1, 0x00, 1, 5000); //SET START RANGE
}

//read the data from sensor
void HMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS<<1, HMC5883L_REG_DATA, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);

    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}

//send the sensor data using serial port.
void USART_SendInt(UART_HandleTypeDef *huart, int16_t value)
{
    char buffer1[8];
    sprintf(buffer1, "%d\r\n", value);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, strlen(buffer1), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t DataX[2];
	uint16_t Xaxis = 0;
	uint8_t DataY[2];
	uint16_t Yaxis = 0;
	uint8_t DataZ[2];
	uint16_t Zaxis = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
//  uint8_t RegSettingA = HMC5883l_Enable_A;
//  uint8_t RegSettingB = HMC5883l_Enable_B;
//  uint8_t RegSettingMR = HMC5883l_MR;

  uint16_t address;
   char uartBuf[10];
  HAL_StatusTypeDef status;
//
//  for (address = 30; address < 255; address++)
//   {
//  uint8_t kk = 0x1E;
//     status = HAL_I2C_IsDeviceReady(&hi2c1,kk<<1, 10, 100);
//     if (status == HAL_OK)
//     {
//       sprintf(uartBuf, "Device found at 0x%02X\r\n", kk);
//       HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
//     }
//   }
//  __HAL_RCC_I2C3_FORCE_RESET();
//
//  HAL_Delay(1000);
//
//  __HAL_RCC_I2C3_RELEASE_RESET();
//
//  MX_I2C3_Init();
  Xaxis = 0;
  Yaxis = 0;
  Zaxis = 0;

//  HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x00 , 1, &RegSettingA , 1, 100);
//  HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x01 , 1, &RegSettingB , 1, 100);
//  HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x02 , 1, &RegSettingMR , 1, 100);



  HMC5883L_Init();

  int16_t x, y, z;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 HMC5883L_ReadData(&x, &y, &z);
	 USART_SendInt(&huart2, x);
	 USART_SendInt(&huart2, y);
	 USART_SendInt(&huart2, z);

	 HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10901E26;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10901E26;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
