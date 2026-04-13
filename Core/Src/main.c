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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include<math.h>

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define SPI_Read_Bit		0x80

#define SPI_Write_Bit		0x7F

#define CS_LOW()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define CS_HIGH()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

#define WHOAMI				0x0F
#define CTRL1				0x10

int16_t az_raw;
int16_t ay_raw;
int16_t ax_raw;

float az_conv;
float ay_conv;
float ax_conv;


float az_bias;
float ay_bias;
float ax_bias;

float az;
float ax;
float ay;


int16_t gx_raw;
int16_t gy_raw;
int16_t gz_raw;



float gx_conv;
float gy_conv;
float gz_conv;


float gx_bias = 0;
float gy_bias = 0;
float gz_bias = 0;

float gx;
float gy;
float gz;

uint32_t last_time = 0;
float angle;
float dt;

int turn_done = 0;


char msg[20] = "Turn performed";


#define OUT_Z_L_A 			0x28

#define CTRL2				0x11

#define CTRL3 				0x12

#define CTRL6				0x15

#define CTRL8				0x17


volatile uint8_t firstEdgeCaptured = 0;

volatile uint8_t lastEdgeCaptured = 0;

volatile uint8_t measurementReady = 0;


uint32_t IC_Value1 = 0;

uint32_t IC_Value2 = 0;

uint32_t time = 0;

float distance = 0;


static uint8_t imu_active = 0;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void SPI_Read_Reg(uint8_t reg, char * name);

void SPI_Write_Reg(uint8_t reg, uint8_t data);

void Read_Accelerometer(int16_t *az_raw, int16_t *ay_raw, int16_t *ax_raw);

void Accelerometer_Conversion(void);

void Accelerometer_Calibration(float *az, float *ay, float *ax);

void Accelerometer_Bias_Offset(void);

void Read_Gyroscope(int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw);

void Gyroscope_Conversion(void);

void Read_Gyroscope_Calibration(float *gx, float *gy, float *gz);

void Gyroscope_Bias_Offset(void);

void Turn_on_Gyroscope();

void Delay_us(uint16_t us);

void IMU_On();

void IMU_Off();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);


  SPI_Read_Reg(WHOAMI, "Whoami");

//  SPI_Write_Reg(CTRL1, 0x09);  // This configures the accelerometer register to work at high performance mode and under  a frequency of 960Hz

  SPI_Write_Reg(CTRL3, 0x44); //BDU + IF_INC - This will apply to both the Accelerometer and the Gyroscope... there will not be a need to re-do it again

//  SPI_Write_Reg(CTRL8, 0x00); // This is setting the full scale(FS) for the accelerometer only
//
//  SPI_Read_Reg(CTRL1, "Accelerometer");
//
//  SPI_Write_Reg(CTRL2, 0x09);  // This configures the Gyroscope register to work at high performance mode and under  a frequency of 960Hz(Is waht actually turn on the sensor)
//
//
//  SPI_Write_Reg(CTRL6, 0x00); //Configuring the LPF and the Full Scale for the gyroscope(to 125dps)


  HAL_Delay(1000);

  last_time = HAL_GetTick();

  IMU_On();

  Gyroscope_Bias_Offset();

  Accelerometer_Bias_Offset();

  IMU_Off();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  Read_Accelerometer(&az_raw, &ay_raw, &ax_raw);

//	  Accelerometer_Conversion();

//	  Read_Gyroscope();

//	  Gyroscope_Conversion();

//	  Accelerometer_Conversion();

//	   I want to trigger the Ultrasonic sensor

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  Delay_us(2);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	  Delay_us(10);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	  if(measurementReady){
		  measurementReady = 0;


		  time = IC_Value2 - IC_Value1; //Each timer tick is in microseconds...and therefore, the time difference is in microseconds

		  distance = time * 0.034 / 2;

	  		 char msg2[20];

	  		 int len = snprintf(msg2, sizeof(msg2), "Distance: %.2f\n\r", distance);

	  		 HAL_UART_Transmit(&huart2, (uint8_t *)msg2, len, HAL_MAX_DELAY);

	  		 HAL_Delay(500);

	  }

	  if(distance < 20){

		  if(!imu_active){
		        IMU_On();
		        last_time = HAL_GetTick();  // reset timing
		        imu_active = 1;
		    }


		  Turn_on_Gyroscope();


		  		 char msg1[20];

		  		 int len = snprintf(msg1, sizeof(msg1), "Angle: %.2f\n\r", angle);

		  		 HAL_UART_Transmit(&huart2, (uint8_t *)msg1, len, HAL_MAX_DELAY);

		  		 HAL_Delay(500);

		  	  if(fabs(angle) >= 90  && !turn_done){
		  		  HAL_UART_Transmit(&huart2, (uint8_t *)msg, 20, HAL_MAX_DELAY);
		  		  angle = 0;
		  		  turn_done = 1;
		  	  }

		  	  if (turn_done) {
		  	      HAL_Delay(2000);   // wait 2 seconds (or any time)

		  	      angle = 0;
		  	      turn_done = 0;

		  	      // 🔥 VERY IMPORTANT: reset timing
		  	      last_time = HAL_GetTick();
		  	  }

		  	  HAL_Delay(5);


	  }

	  else{
		    if(imu_active){
		        IMU_Off();

		        angle = 0;
		        imu_active = 0;

		        last_time = HAL_GetTick();
		    }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SPI_Read_Reg(uint8_t reg, char * name){
	  uint8_t tx_data[2];
	  uint8_t rx_data[2];

	  tx_data[0] = reg | SPI_Read_Bit;
	  tx_data[1] = 0x00;


	  CS_LOW();

	  HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, HAL_MAX_DELAY);

	  CS_HIGH();

	  uint8_t whoami = rx_data[1];

	  char msg[64];

	  int len = snprintf(msg, sizeof(msg), "The %s Register Value : 0x%02X", name, whoami);

	  HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);

}


void SPI_Write_Reg(uint8_t reg, uint8_t data){

	uint8_t tx_data[2];

	tx_data[0] = reg & SPI_Write_Bit;
	tx_data[1] = data;

	CS_LOW();

	HAL_SPI_Transmit(&hspi2, tx_data, 2, HAL_MAX_DELAY);


	CS_HIGH();
}

void Read_Accelerometer(int16_t *az_raw, int16_t *ay_raw, int16_t *ax_raw){
	uint8_t tx[7] = {0};

	uint8_t rx[7] = {0};

	tx[0] = 0x28 | SPI_Read_Bit;

	tx[1] = 0x00;

	CS_LOW();

	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, HAL_MAX_DELAY);

	CS_HIGH();

	 * az_raw = (int16_t)((rx[2] << 8) | rx[1]);

	 * ay_raw = (int16_t)((rx[4] << 8) | rx[3]);

	 * ax_raw = (int16_t)((rx[6] << 8) | rx[5]);


	 //Sensor values are normally read in integers.. but you should always convert them to -
	 //	 - float before you do further operations in order not to loose data

//	 char msg[90];
//
//	 int len = snprintf(msg, sizeof(msg), "\n\r az_raw : %d\n\r  ay_raw : %d\n\r ax_raw : %d\n\r ", *az_raw, *ay_raw, *ax_raw);
//
//	 HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);
}

void Accelerometer_Bias_Offset(void){

	int32_t az_offset = 0;
	int32_t ay_offset = 0;
	int32_t ax_offset = 0;

	int sample = 500;

	for(int i = 0; i < sample; i++){
		Read_Accelerometer(&az_raw, &ay_raw, &ax_raw);

		az_offset += az_raw;
		ay_offset += ay_raw;
		ax_offset += ax_raw;

		HAL_Delay(2);
	}

	az_bias = az_offset / sample;
	ay_bias = ay_offset / sample;
	ax_bias = ax_offset / sample;

}

void Accelerometer_Calibration(float *az, float *ay, float *ax){
	Read_Accelerometer(&az_raw, &ay_raw, &ax_raw);

	*az = az_raw - az_bias + 16384.0f;
	*ay = ay_raw - ay_bias;  // az_bias is float and forces az_raw to be float too... therefore float always dominates!
	*ax = ax_raw - ax_bias;


}

void Accelerometer_Conversion(void){
	Accelerometer_Calibration(&az, &ay, &ax);

	az_conv = az * 0.000061;

	ay_conv = ay * 0.000061;

	ax_conv = ax * 0.000061;


	char msg[90];

	int len = snprintf(msg, sizeof(msg), "\n\r az_conv : %.2f\n\r  ay_conv : %.2f\n\r ax_conv : %.2f\n\r ", az_conv, ay_conv, ax_conv);

	HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);
}

void Read_Gyroscope(int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw){
	uint8_t tx[7] = {0};

	uint8_t rx[7] = {0};

	tx[0] = 0x22 | SPI_Read_Bit;
	tx[1] = 0x00;

	CS_LOW();

	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, HAL_MAX_DELAY);

	CS_HIGH();


	 *gx_raw = (int16_t)(rx[2] << 8 | rx[1]);
	 *gy_raw = (int16_t)(rx[4] << 8 | rx[3]);
	 *gz_raw = (int16_t)(rx[6] << 8 | rx[5]);


}


void Gyroscope_Bias_Offset(void){

	int sample = 500;

	int32_t gx_offset = 0;
	int32_t gy_offset = 0;
	int32_t gz_offset = 0;

	for(int i = 0; i < sample; i++){
		Read_Gyroscope(&gx_raw, &gy_raw, &gz_raw);

		gx_offset += gx_raw;
		gy_offset += gy_raw;
		gz_offset += gz_raw;

		HAL_Delay(2);

	}

	gx_bias = gx_offset / sample;
	gy_bias = gy_offset / sample;
	gz_bias = gz_offset / sample;

	}

void Read_Gyroscope_Calibration(float *gx, float *gy, float *gz){

	Read_Gyroscope(&gx_raw, &gy_raw, &gz_raw);

	*gx = gx_raw - gx_bias;
	*gy = gy_raw - gy_bias;
	*gz = gz_raw - gz_bias;

}

void Gyroscope_Conversion(void){
	Read_Gyroscope_Calibration(&gx, &gy, &gz);

	gx_conv = gx * 0.004375;
	gy_conv = gy * 0.004375;
	gz_conv = gz * 0.004375;

}

void Turn_on_Gyroscope(){

	uint32_t now = HAL_GetTick();
	dt = (now - last_time) / 1000.0f;
	last_time = now;

	Read_Gyroscope_Calibration(&gx, &gy, &gz);

	gz_conv = gz * 0.004375;

	 if (fabs(gz_conv) < 0.5f) {
	        gz_conv = 0;
	    }

	static float gz_filtered = 0;

	gz_filtered = 0.7f * gz_filtered + 0.3f * gz_conv;

	if(!turn_done){  // Limits this function to only perform the intergration if the turn is not done
		angle +=  gz_filtered* dt;

	}

}

void Delay_us(uint16_t us){  // The goodness with this timing function is that it will help us to implement the delays in microseconds
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(__HAL_TIM_GET_COUNTER(&htim6) < us);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */

  if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

	  if(firstEdgeCaptured == 0){
		  IC_Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		  firstEdgeCaptured = 1;

		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
	  }

	  else {
		  IC_Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		  firstEdgeCaptured = 0;

		  measurementReady = 1;

		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

	  }

  }
}

void IMU_On(){
	  SPI_Write_Reg(CTRL1, 0x09); // This configures the accelerometer register to work at high performance mode and under  a frequency of 960Hz
	  SPI_Write_Reg(CTRL8, 0x00); // This is setting the full scale(FS) for the accelerometer only

	  SPI_Write_Reg(CTRL2, 0x09);  // This configures the Gyroscope register to work at high performance mode and under  a frequency of 960Hz(Is waht actually turn on the sensor)
	  SPI_Write_Reg(CTRL6, 0x00); //Configuring the LPF and the Full Scale for the gyroscope(to 125dps)

}


void IMU_Off(){
	  SPI_Write_Reg(CTRL1, 0x00);
	  SPI_Write_Reg(CTRL2, 0x00);
}

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
