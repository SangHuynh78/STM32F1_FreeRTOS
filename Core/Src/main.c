/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c-lcd.h"
#include "DS3231.h"
#include "DHT11.h"
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId SensorTaskHandle;
osThreadId CheckTaskHandle;
osThreadId UartTaskHandle;
osSemaphoreId myBinarySemHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
//void Display_MOI(float Temp);
void UpdateSensorTask(void const * argument);
void ExecuteTask(void const * argument);
void UartTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//UART Related
uint8_t rx_data = 0;

uint8_t indx = 0;
//Frame truyền sensor data lên esp32
uint8_t frame[4]; //Frame truyền data lên esp32

uint8_t pump_status, led_status; //status gửi lên esp32

//Biến cho RTC
char buffer[15];

//TIME mam;
uint8_t  sec,min,hour,day,date,month,year;

//biến cho sky mưa
uint8_t rain = 0; //mặc định là không mưa

 //Biến cho nút bấm PA1
 uint8_t mnt = 0;

 //Biến cho nút bấm PB12
 uint8_t pb12 = 0;
 uint8_t led_for_rain = 0;

 //Biến cho nút bấm PB13
 uint8_t pb13 = 0;

 //Biến lưu độ ẩm đất
 uint8_t moi = 0; //Mặc định là đất ẩm
 char str[20];

 //Hours schedule realated
 // Giờ bật và tắt đèn
 uint8_t light_on_hour = 18;   // Giờ bật đèn (6 PM)
 uint8_t light_off_hour = 6;   // Giờ tắt đèn (6 AM)
 //Giờ bật máy bơm
 uint8_t scheduled_hour = 15;
 uint8_t scheduled_min_start = 30;
 uint8_t scheduled_min_stop = 35;

 //BIẾN CHO DHT11
 float Temperature;
 float Humidity;

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  HAL_TIM_Base_Start(&htim2);
  lcd_init();
  //Set_Time (00, 26, 11, 31, 31, 5, 1976);
  /* USER CODE END 2 */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem */
  osSemaphoreDef(myBinarySem);
  myBinarySemHandle = osSemaphoreCreate(osSemaphore(myBinarySem), 1);

  /* Create the thread(s) */
  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, UpdateSensorTask, osPriorityNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of ButtonTask */
  osThreadDef(CheckTask, ExecuteTask, osPriorityAboveNormal, 0, 128);
  CheckTaskHandle = osThreadCreate(osThread(CheckTask), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, UartTask, osPriorityHigh, 0, 128);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12)
	{
		for(int i = 9999; i>0; i--);
		if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
		{
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
			HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
			pb12 = !pb12;
		}
	}

	if(GPIO_Pin == GPIO_PIN_13)
	{
		for(int i = 9999; i>0; i--);
		if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
		{
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
			HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
			pb13 = !pb13;
		}
	}
}

//Hàm UART CALLBACK ở đây để ngắt mỗi khi có tín hiệu TỪ ESP chuyển trạng thái hay có tín hiệu chuyển relay
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if (rx_data & (1 << 1))
		{
			if (!rain)
				pb13 = 1;	// bật bơm
		}
		else
			pb13 = 0;	// tắt bơm

		if (rx_data & (1 << 0))
			pb12 = 1;	// bật đèn
		else
			if (!rain)
				pb12 = 0;	// tắt đèn
		HAL_UART_Receive_IT(huart, &rx_data, 1);
	}
}
/* USER CODE END 4 */


/* USER CODE BEGIN FOR UPDATESENSORTASK  */
void UpdateSensorTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
	osSemaphoreWait(myBinarySemHandle, osWaitForever);

//	DS3231
	Get_Time(&sec,&min,&hour,&day,&date,&month,&year);
	sprintf (buffer, "%02d:%02d", hour, min);
	lcd_put_cur (0,1);
	lcd_send_string(buffer);
	sprintf (buffer, "%02d-%02d-%02d", date, month, year);
	lcd_put_cur(0, 8);
	lcd_send_string(buffer);

//	DHT11
	DHT11_Read_Data(&Temperature, &Humidity);
	Display_Temp(Temperature);
	Display_Rh(Humidity);

//	Rain Sensor
	rain = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

//	Moisture Sensor
	moi = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

	osSemaphoreRelease(myBinarySemHandle);
    osDelay(10);
  }
  /* USER CODE END FOR UPDATESENSORTASK */
}


/* USER CODE BEGIN FOR EXECUTETASK */
void ExecuteTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonTask */
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreWait(myBinarySemHandle, osWaitForever);
	if (rain)
	{
		led_for_rain = 1;
		pb12 = 0;
		pb13 = 0;
	}
	else
	{
		led_for_rain = 0;
		if (Temperature > 35 || Humidity < 50)
			pb13 = 1;
		if (Temperature < 20 || Humidity > 70)
			pb13 = 0;
		if (hour == scheduled_hour && min >= scheduled_min_start)
			pb13 = 1;
		if (hour == scheduled_hour && min >= scheduled_min_stop)
			pb13 = 0;
	}
	if (pb13)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	if (pb12 || led_for_rain)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	osSemaphoreRelease(myBinarySemHandle);
	osDelay(10);
  }
  /* USER CODE END FOR EXECUTETASK*/
}

/* USER CODE BEGIN FOR UARTTASK*/
void UartTask(void const * argument)
{
  /* USER CODE BEGIN UartTask */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(myBinarySemHandle, osWaitForever);

	  frame[0] = 0x02; // 0x02 là byte nhận dạng chuỗi
	  frame[1] = (uint8_t)Humidity; // Byte chứa giá trị độ ẩm
	  frame[2] = (uint8_t)Temperature; // Byte chứa giá trị nhiệt độ

	  pump_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	  led_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

	  frame[3] = (pump_status << 3) | (led_status << 2) | (moi << 1) | rain; // Byte chứa trạng thái bơm, led, độ ẩm đất, mưa

	  HAL_UART_Transmit(&huart1, frame, sizeof(frame), HAL_MAX_DELAY);

	  osSemaphoreRelease(myBinarySemHandle);
	  osDelay(100);
	  }
  /* USER CODE END FOR UARTTASK*/
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
