/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAMANIO 128
#define SRC_RS485 1
#define SRC_PYTHON 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ---- UART2 (Python) ----
uint8_t frame_buf_uart2[32];
uint8_t idx_u2 = 0;
uint8_t ready_u2 = 0;
uint8_t rx2_byte;

// ---- UART1 (RS485) ----
uint8_t frame_buf_uart1[32];
uint8_t idx_u1 = 0;
uint8_t ready_u1 = 0;
uint8_t rx1_byte;

// ---- Señales ---------
uint32_t senial_senoidal[TAMANIO];
uint32_t senial_triangular[TAMANIO];
uint32_t senial_diente[TAMANIO];
uint32_t senial_cuadrada[TAMANIO];

uint32_t *signal_ptr = NULL;
uint16_t signal_length = TAMANIO;

uint32_t buffer_escalado[TAMANIO];

uint8_t rs485_rx_byte;   // byte temporal para USART1
uint8_t last_frame_source = 0;// 1 = RS485 (USART1) // 2 = Python (USART2)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t crc32_calc(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // --------------------------------------------------------------------
    // Recepción desde PYTHON (USART2)
    // --------------------------------------------------------------------
    if (huart->Instance == USART2)
    {
        frame_buf_uart2[idx_u2++] = rx2_byte;

        if (idx_u2 >= 32)
        {
            ready_u2 = 1;
            idx_u2 = 0;
        }

        HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
        return;
    }

    // --------------------------------------------------------------------
    // Recepción desde RS485 (USART1)
    // --------------------------------------------------------------------
    if (huart->Instance == USART1)
    {
        frame_buf_uart1[idx_u1++] = rx1_byte;

        if (idx_u1 >= 32)
        {
            ready_u1 = 1;
            idx_u1 = 0;
        }

        HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
        return;
    }
}

void crear_senial(void)
{
	uint32_t i;

	for (i = 0 ; i < TAMANIO ; i++)
		senial_senoidal [i] = (float)( sin (2*M_PI*i/TAMANIO) + 1) * (float)(4095 / 2);

	for (i = 0 ; i < TAMANIO/2 ; i++)
		senial_triangular[i] = (uint32_t)(( (float)i / (TAMANIO/2) ) * 4095);
	for (i = TAMANIO/2 ; i < TAMANIO ; i++)
		senial_triangular[i] = (uint32_t)( 4095 - ( (float)(i - TAMANIO/2) / (TAMANIO/2) ) * 4095 );

	for (i = 0 ; i < TAMANIO ; i++)
	    senial_diente[i] = (4095 * i) / TAMANIO;

	for (i = 0 ; i < TAMANIO/2 ; i++)
		senial_cuadrada [i] = (float)(4095);
	for (i = TAMANIO/2 ; i < TAMANIO ; i++)
		senial_cuadrada [i] = (float)(0);

}

uint16_t mapear_amplitud(uint16_t adc_raw)
{
    float A_min = 620.0f;   // 500 mV
    float A_max = 4095.0f;  // 3.3 V

    float A = A_min + (adc_raw / 4095.0f) * (A_max - A_min);

    return (uint16_t)A;
}

void aplicar_amplitud(uint16_t amp)
{
    if (signal_ptr == NULL)
        return;

    for (int i = 0; i < TAMANIO; i++)
    {
        buffer_escalado[i] = (signal_ptr[i] * amp) / 4095;
    }

}

uint16_t leer_pot(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);      // 0..4095
}

void DAC_set_signal(uint8_t tipo)
{
    switch(tipo)
    {
        case 0:
            HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
            signal_ptr = NULL;
            return;

        case 1: signal_ptr = senial_senoidal;  break;
        case 2: signal_ptr = senial_triangular; break;
        case 3: signal_ptr = senial_diente;     break;
        case 4: signal_ptr = senial_cuadrada;   break;
        default:
            return;
    }

    // Aplicar amplitud sobre la nueva señal
    uint16_t pot_raw = leer_pot();
    uint16_t amp = mapear_amplitud(pot_raw);
    aplicar_amplitud(amp);

    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

    HAL_DAC_Start_DMA(&hdac,
                      DAC_CHANNEL_1,
                      buffer_escalado,
                      TAMANIO,
                      DAC_ALIGN_12B_R);
}


void DAC_set_frequency(uint16_t freq_hz)
{
    if (freq_hz < 100)   freq_hz = 100;
    if (freq_hz > 10000) freq_hz = 10000;

    uint32_t tim_clk = 50000000;  // CORRECTO para APB1=HCLK/4

    uint32_t arr = (tim_clk / (freq_hz * TAMANIO)) - 1;

    if (arr > 0xFFFF)
        arr = 0xFFFF;

    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);
}


uint16_t map_adc_to_freq(uint16_t adc)
{
    float f = 100.0f + ((float)adc / 4095.0f) * 9900.0f;

    return (uint16_t)f;  // 100 – 10000 Hz
}

static inline void RS4851_SetTx(void) { HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET); }
static inline void RS4851_SetRx(void) { HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET); }

/* Envío bloqueante por RS485 usando USART1 y control DE */
void RS4851_SendBlocking(uint8_t *buf, uint16_t len)
{
    // 1) activar driver (transmitir)
    RS4851_SetTx();

    // 2) settle time breve (usado HAL_Delay(1) por simplicidad)
    HAL_Delay(1);

    // 3) transmitir por huart1
    HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);

    // 4) esperar TC (Transmission Complete) => último bit enviado
    uint32_t tickstart = HAL_GetTick();
    while (!(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)))
    {
        if ((HAL_GetTick() - tickstart) > 100) break; // timeout por seguridad
    }

    // 5) volver a modo recepción
    RS4851_SetRx();
}

// Responder a quien corresponda
void responder_OK(uint8_t source)
{
    if (source == SRC_PYTHON)
        HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, HAL_MAX_DELAY);

    else
        RS4851_SendBlocking((uint8_t*)"OK\r\n", 4);
}

void responder_ERROR(uint8_t source)
{
    if (source == SRC_PYTHON)
        HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\r\n", 7, HAL_MAX_DELAY);

    else
        RS4851_SendBlocking((uint8_t*)"ERROR\r\n", 7);
}


void procesar_trama(uint8_t *f, uint8_t source)
{
    // 1. Verificar SOF
    if (f[0] != 0xAA || f[1] != 0x55)
    {
        responder_ERROR(source);
        return;
    }

    uint8_t origen  = f[2];
    uint8_t destino = f[3];
    uint8_t size    = f[4];

    // Validar que la trama esté dirigida al Peer (ID = 0x06)
    if (destino != 0x06)
    {
        responder_ERROR(source);
        return;   // ignorar trama
    }

    if (size < 3 || size > 21)
    {
        responder_ERROR(source);
        return;
    }

    uint8_t *payload = &f[5];

    uint32_t offset_crc = 5 + size;
    uint32_t offset_eof = offset_crc + 4;

    if (offset_eof + 1 >= 32)
    {
        responder_ERROR(source);
        return;
    }

    // EOF
    if (f[offset_eof] != 0x55 || f[offset_eof + 1] != 0xAA)
    {
        responder_ERROR(source);
        return;
    }

    // CRC recibido
    uint32_t crc_rx =
        (f[offset_crc]) |
        (f[offset_crc + 1] << 8) |
        (f[offset_crc + 2] << 16) |
        (f[offset_crc + 3] << 24);

    // CRC calculado
    uint8_t data_crc[3 + 21];
    uint32_t len_crc = 3 + size;

    data_crc[0] = origen;
    data_crc[1] = destino;
    data_crc[2] = size;

    for (int i = 0; i < size; i++)
        data_crc[3 + i] = payload[i];

    uint32_t crc_calc = crc32_calc(data_crc, len_crc);

    if (crc_calc != crc_rx)
    {
        responder_ERROR(source);
        return;
    }

    // ---- TRAMA VÁLIDA ----
    responder_OK(source);

    uint16_t adc_raw = (payload[2] << 8) | payload[1];
    adc_raw &= 0x0FFF;

    uint16_t freq = map_adc_to_freq(adc_raw);
    uint8_t tipo  = payload[0];

    DAC_set_frequency(freq);
    DAC_set_signal(tipo);

    // Obtener amplitud actual (ya calculada)
    uint16_t pot_raw = leer_pot();
    uint16_t amp = mapear_amplitud(pot_raw);
    float amp_volts = (amp / 4095.0f) * 3.3f + 0.5f;

    // 🔵 SIEMPRE enviar resultado a PC (USART2)
    char msg[80];
    snprintf(msg, sizeof(msg),
             "TIPO=%u FREQ=%uHz AMP=%uV\r\n",
             tipo, freq, amp_volts);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  crear_senial();
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  RS4851_SetRx();
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // ---- Si llegó una trama por Python ----
	    if (ready_u2)
	    {
	        ready_u2 = 0;
	        procesar_trama(frame_buf_uart2, SRC_PYTHON);
	    }

	    // ---- Si llegó una trama por RS485 ----
	    if (ready_u1)
	    {
	        ready_u1 = 0;
	        procesar_trama(frame_buf_uart1, SRC_RS485);
	    }

	    // ---- Actualizar amplitud continuamente ----
	    uint16_t pot_raw = leer_pot();
	    uint16_t amp = mapear_amplitud(pot_raw);
	    aplicar_amplitud(amp);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 833;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RS485_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RS485_DE_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
