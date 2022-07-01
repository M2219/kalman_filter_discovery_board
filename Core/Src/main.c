/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalmanfilter.h"
#include "mathlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 100 // samples
#define dt 0.1 // samples
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
unsigned char buff[64] = {0};
unsigned char xl[4] = {0};
unsigned char yl[4] = {0};
unsigned char zl[4] = {0};
unsigned char vxl[4] = {0};
unsigned char vyl[4] = {0};
unsigned char vzl[4] = {0};

unsigned char uxl[4] = {0};
unsigned char uyl[4] = {0};
unsigned char uzl[4] = {0};

float fx = 0;
float fy = 0;
float fz = 0;
float fvx = 0;
float fvy = 0;
float fvz = 0;

float fux;
float fuy;
float fuz;

//const float dt = 0.1; // time step

// model parameters
const float sig_acc_true[NUM_I][1] = {{0.3}, {0.3}, {0.3}};  // true value of standard deviation of accelerometer noise
const float sig_gps_true[NUM_S][1] = {{3.0}, {3.0}, {3.0}, {0.03}, {0.03}, {0.03}}; // true value of standard deviation of GPS noise

const float  sig_acc[NUM_I][1] = {{0.3}, {0.3}, {0.3}}; // user input of standard deviation of accelerometer noise
const float sig_gps[NUM_S][1] = {{3}, {3}, {3}, {0.03}, {0.03}, {0.03}}; // user input of standard deviation of GPS noise

float Q[NUM_S][NUM_S] = {{(0.25 * powf(dt, 4) * powf(0.3, 2)), 0, 0, 0, 0, 0},
                         {0, (0.25 * powf(dt, 4) * powf(0.3, 2)), 0, 0, 0, 0},
                         {0, 0, (0.25 * powf(dt, 4) * powf(0.3, 2)), 0, 0, 0},
                         {0, 0, 0, (powf(dt, 2) * powf(0.3, 2)), 0, 0},
                         {0, 0, 0, 0, (powf(dt, 2) * powf(0.3, 2)), 0},
                         {0, 0, 0, 0, 0, (powf(dt, 2) * powf(0.3, 2))}}; // process noise covariance matrix

float R[NUM_S][NUM_S] = {{powf(3, 2), 0, 0, 0, 0, 0},
                         {0, powf(3, 2), 0, 0, 0, 0},
                         {0, 0, powf(3, 2), 0, 0, 0},
                         {0, 0, 0, powf(0.03,2), 0, 0},
                         {0, 0, 0, 0, powf(0.03,2), 0},
                         {0, 0, 0, 0, 0, powf(0.03,2)}}; // measurement noise covariance matrix

float F[NUM_S][NUM_S] = {{1, 0, 0, dt, 0, 0},
                         {0, 1, 0, 0, dt, 0},
                         {0, 0, 1, 0, 0, dt},
                         {0, 0, 0, 1, 0, 0},
                         {0, 0, 0, 0, 1, 0},
                         {0, 0, 0, 0, 0, 1}}; // state transition matrix

float B[NUM_S][NUM_I] = {{0.5 * powf(dt, 2), 0, 0},
                         {0, 0.5 * powf(dt, 2), 0},
                         {0, 0, 0.5 * powf(dt, 2)},
                         {dt, 0, 0},
                         {0, dt, 0},
                         {0, 0, dt}}; // control-input matrix

float H[NUM_S][NUM_S] = {{1, 0, 0, 0, 0, 0},
                         {0, 1, 0, 0, 0, 0},
                         {0, 0, 1, 0, 0, 0},
                         {0, 0, 0, 1, 0, 0},
                         {0, 0, 0, 0, 1, 0},
                         {0, 0, 0, 0, 0, 1}};// measurement matrix

// kalman filter initial guess
float x_est[NUM_S][1] = {{2}, {-2}, {0}, {5}, {5.1}, {0.1}};

float P[NUM_S][NUM_S] = {{powf(4, 2), 0, 0, 0, 0, 0},
                         {0, powf(4, 2), 0, 0, 0, 0},
                         {0, 0, powf(4, 2), 0, 0, 0},
                         {0, 0, 0, powf(0.4, 2), 0, 0},
                         {0, 0, 0, 0, powf(0.4, 2), 0},
                         {0, 0, 0, 0, 0, powf(0.4, 2)}};

// temp variable
float x_est_update[NUM_S][1];
float P_update[NUM_S][NUM_S];
float z_measure[NUM_S][1];
float u_p[NUM_I][1];
float x_debug[6];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //restart the interrupt reception mode
{
	HAL_UART_Receive_IT(&huart1, buff, 36);
}

// structure to hold and receive data
// x,y,z, vx, vy, vz = 6 floats = 24B buffer
struct XL_Data
{
float x, y, z, vx, vy, vz;
}tx_data;

// generate random data between float min and float max
// function to convert and send over UART, param XL_Data, UART Handle Typedef

void sendData(struct XL_Data * data, UART_HandleTypeDef * huart)
{
	// uart can only send unsigned int
	// thus we need to convert out struct data
	char buffer[sizeof(data) * 6]; // buffer in size data 24B for four floats
	memcpy(buffer, data, sizeof(data) * 6);
	HAL_UART_Transmit(huart, (uint8_t *)buffer, sizeof(buffer), 500);
}

/* USER CODE END 0 */


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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int X = 1;
  while (1)
  {
    /* USER CODE END WHILE */
    //receive data
    HAL_UART_Receive_IT(&huart1, buff, 36);

    for( int a = 0 ; a < 5 ; a = a + 1 )
    {
        xl[a] = buff[a];
        yl[a] = buff[a + 4];
        zl[a] = buff[a + 8];
        vxl[a] = buff[a + 12];
        vyl[a] = buff[a + 16];
        vzl[a] = buff[a + 20];

        uxl[a] = buff[a + 24];
        uyl[a] = buff[a + 28];
        uzl[a] = buff[a + 32];

    }

    memcpy(&fx, &xl, sizeof(fx));
    memcpy(&fy, &yl, sizeof(fy));
    memcpy(&fz, &zl, sizeof(fz));
    memcpy(&fvx, &vxl, sizeof(fvx));
    memcpy(&fvy, &vyl, sizeof(fvy));
    memcpy(&fvz, &vzl, sizeof(fvz));

    memcpy(&fux, &uxl, sizeof(fux));
    memcpy(&fuy, &uyl, sizeof(fuy));
    memcpy(&fuz, &uzl, sizeof(fuz));


    if(fx != 0.0 && fy != 0.0 && fz != 0.0 && fvx != 0.0 && fvy != 0.0 && fvz != 0.0)
    {
    	z_measure[0][0] = fx;
    	z_measure[1][0] = fy;
    	z_measure[2][0] = fz;
    	z_measure[3][0] = fvx;
    	z_measure[4][0] = fvy;
    	z_measure[5][0] = fvz;

    	if(X == 1)
      	{
           x_debug[0] = fx;
           x_debug[1] = fy;
           x_debug[2] = fz;
           x_debug[3] = fvx;
           x_debug[4] = fvy;
           x_debug[5] = fvz;

	    }
    	u_p[0][0] = fux;
    	u_p[1][0] = fuy;
    	u_p[2][0] = fuz;

        kf(dt, F, B, H, x_est, P, Q, R, u_p, z_measure, x_est_update, P_update);

        memcpy(x_est, x_est_update, sizeof(x_est_update));
        memcpy(P, P_update, sizeof(P_update));

	    //send data
	    tx_data.x = x_est[0][0];
	    tx_data.y = x_est[1][0];
	    tx_data.z = x_est[2][0];
	    tx_data.vx = x_est[3][0];
	    tx_data.vy = x_est[4][0];
	    tx_data.vz = x_est[5][0];

	    // indicate start with S
	    HAL_UART_Transmit(&huart1, (uint8_t *)"S", sizeof("S"), 500);
	    sendData(&tx_data, &huart1);
      	// indicate end of sequence with "Z"
	    HAL_UART_Transmit(&huart1, (uint8_t *)"Z", sizeof("Z"), 500);

	    //HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_13);
        HAL_Delay (20);
        /* USER CODE BEGIN 3 */
        X = X + 1;
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

