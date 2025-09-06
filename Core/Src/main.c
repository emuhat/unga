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
#include <stdarg.h>
#include "ARGB.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
//uint8_t inbuffer[4];
int actually_light_stuff_up = 1;

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////


/*

  Bytes show up

  write_ptr

  num_bytes_to_read

  recv_mode

*/



const uint8_t PING_TYPE = 0;
const uint8_t PONG_TYPE = 1;
const uint8_t STRING_TYPE = 2;
const uint8_t TOGGLE_LIGHT_TYPE = 3;

#define BUFFER_SIZE 512
#define MAX_PACKET_SIZE 512

// 6 byte header:
// three ident bytes
// 1 byte command
// N byte payload based on command
const uint8_t IDENT_BYTES[] = { 'E', 'P', 'T' };
const size_t IDENT_LEN = sizeof(IDENT_BYTES) / sizeof(IDENT_BYTES[0]);

const uint32_t HEADER_BYTES = 6;
const uint32_t USER_0_POS = 4;
const uint32_t USER_1_POS = 5;
const uint32_t COMMAND_POS = 3;


const int MODE_SCANNING_FOR_HEADER = 0;
const int MODE_RECEIVING_HEADER = 1;
const int MODE_RECEIVING_PAYLOAD = 2;

#define UART_RX_BUFFER_SIZE  40

struct SerialRead
{
	uint8_t* buffer;
	uint32_t buf_size;
    int read_ptr;			// the latest we've processed
    int write_ptr;
};


struct NSPData
{
	struct SerialRead* sr;
    uint8_t tx_buffer[BUFFER_SIZE];
    int packet_start;
    int num_bytes_to_read;	// starting from the packet_start_ptr
    int recv_mode;
};




struct SerialRead from_pi;

struct NSPData nsp_data;

void sr_init(struct SerialRead* sr, uint32_t buf_size) {
	sr->buffer = (uint8_t*)malloc(buf_size);
	sr->buf_size = buf_size;
	sr->read_ptr = 0;
	sr->write_ptr = 0;
}

uint32_t sr_byte_diff(struct SerialRead* sr, int start, int end) {
	return start > end ? end + sr->buf_size - start : end - start;
}

uint32_t sr_add_offset(struct SerialRead* sr, int start, int offset) {
	return (start + offset) % sr->buf_size;
}

uint32_t sr_bytes_available(struct SerialRead* sr) {
	// if the read ptr is ahead of the write ptr, it means the write pointer has wrapped
//	return sr->read_ptr > sr->write_ptr ? sr->write_ptr + sr->buf_size - sr->read_ptr : sr->write_ptr - sr->read_ptr;
	return sr_byte_diff(sr, sr->read_ptr, sr->write_ptr);
}

void nsp_set_for_header_scan(struct NSPData* nsp_data)
{
	nsp_data->recv_mode = MODE_SCANNING_FOR_HEADER;
	nsp_data->num_bytes_to_read = 1; // 1 byte at a time in scan mode
	nsp_data->packet_start = nsp_data->sr->read_ptr;
}

void nsp_set_for_header_receive(struct NSPData* nsp_data)
{
	nsp_data->recv_mode = MODE_RECEIVING_HEADER;
	nsp_data->num_bytes_to_read = HEADER_BYTES;  // Looking for the whole header
	nsp_data->packet_start = nsp_data->sr->read_ptr;
}

void nsp_init(struct NSPData* nsp, struct SerialRead* sr)
{
	nsp->sr = sr;
	nsp_set_for_header_scan(nsp);
}

uint16_t nsp_packet_start(uint8_t* buffer, uint8_t ptype, uint8_t u0, uint8_t u1)
{
	uint16_t write_ptr = 0;

    buffer[write_ptr++] = IDENT_BYTES[0];
    buffer[write_ptr++] = IDENT_BYTES[1];
    buffer[write_ptr++] = IDENT_BYTES[2];
    buffer[write_ptr++] = ptype; // command type

    // user bytes
    buffer[write_ptr++] = u0;
    buffer[write_ptr++] = u1;

    return write_ptr;
}

void nsp_send_packet(uint8_t* buffer, uint16_t len) {
	HAL_UART_Transmit(&huart1, buffer, len, 0xFFFF);
}

void nsp_send_ping_packet(struct NSPData* nsp_data) {
	uint16_t write_ptr = nsp_packet_start(nsp_data->tx_buffer, PING_TYPE, 0, 0);
    nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

void nsp_send_pong_packet(struct NSPData* nsp_data) {
	uint16_t write_ptr = nsp_packet_start(nsp_data->tx_buffer, PONG_TYPE, 0, 0);
    nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

//void nsp_start_read(struct NSPData* nsp_data) {
//	HAL_UART_Receive_DMA(&huart1, nsp_data->rx_buffer, HEADER_BYTES);
//}

void nsp_print(struct NSPData* nsp_data, const char *fmt, ...) {
    char buf[256];  // adjust size to your needs
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0) {
        uint8_t u0 = (n & 0xff);
        uint8_t u1 = (n >> 8);

    	nsp_packet_start(nsp_data->tx_buffer, STRING_TYPE, u0, u1);
    	strcpy((char*)nsp_data->tx_buffer + HEADER_BYTES, buf);
    	nsp_send_packet(nsp_data->tx_buffer, HEADER_BYTES + n);
    }
}

void sr_advance_read_ptr(struct SerialRead* sr, uint32_t count) {

//	nsp_print(&nsp_data, "adv: ba=%d, write is %d, read was %d, advancing by %d", sr_bytes_available(sr), sr->write_ptr, sr->read_ptr, count);
	sr->read_ptr = (sr->read_ptr + count) % sr->buf_size;
}

void nsp_dispatch(struct NSPData* nsp) {
	nsp_print(nsp, "maybe dispatch a packet!!");
	struct SerialRead* sr = nsp->sr;


	int safe_cmd_offset = sr_add_offset(sr, nsp->packet_start, COMMAND_POS);
	int command = sr->buffer[safe_cmd_offset];

	if (command == TOGGLE_LIGHT_TYPE) {
		actually_light_stuff_up = !actually_light_stuff_up;
	}
}

void nsp_process_rx(struct NSPData* nsp)
{
	struct SerialRead* sr = nsp->sr;
	while (1)
	{
		uint32_t bytes_avail = sr_bytes_available(&from_pi);

		// ignore if we don't have the number of required bytes
		if (bytes_avail < nsp->num_bytes_to_read) {
			return;
		}

//		nsp_print(nsp, "BYtesAvail %d, read_pointer = %d", bytes_avail, sr->read_ptr);


		if (nsp->recv_mode == MODE_SCANNING_FOR_HEADER) {

			int good = 0;
			int byte_to_check = sr_byte_diff(sr, nsp->packet_start, sr->read_ptr);	// which byte we're checking

//			nsp_print(nsp, "bc = %d", byte_check);

			if (byte_to_check < IDENT_LEN)
			{
//				nsp_print(nsp, "bc = %d", byte_check);
//				nsp_print(nsp, "exp = %d", IDENT_BYTES[byte_check]);
//				nsp_print(nsp, "got = %d", sr->buffer[sr->read_ptr]);

				if (IDENT_BYTES[byte_to_check] == sr->buffer[sr->read_ptr]) {
					nsp_print(nsp, "awesome");
					good = 1;
				}

				sr_advance_read_ptr(&from_pi, 1);	// move to next byte no matter what
			}

			// huzzah, we reached the end and successfully found all the ident bytes!
			else if (byte_to_check == IDENT_LEN) {
				nsp_print(nsp, "double awesome");
				nsp->recv_mode = MODE_RECEIVING_HEADER;
				nsp->num_bytes_to_read = HEADER_BYTES - IDENT_LEN;	// set up to receive the rest of the header
				good = 1;

				// Note not advancing the read pointer here -- it's already at the right spot
			}

			if (!good) {
				nsp_print(nsp, "not good!!");
				nsp_set_for_header_scan(nsp);
			}
		}

		else if (nsp->recv_mode == MODE_RECEIVING_HEADER) {

			// check here to see if ident matches
			int good = 1;
			for (int i = 0; i < IDENT_LEN; ++i) {
				int safe_off = sr_add_offset(sr, nsp->packet_start, i);
//				nsp_print(nsp, "SafeOff %d", safe_off);
				if (IDENT_BYTES[i] != sr->buffer[safe_off]) {
					good = 0;	// whuh oh, ident doesn't match
					break;
				}
			}

			if (good)
			{
				nsp_dispatch(nsp);

				// skip past the rest of the header
				sr_advance_read_ptr(&from_pi, nsp->num_bytes_to_read);

				// set up to receive the next header
				nsp_set_for_header_receive(nsp);

//				nsp_print(nsp, "set up for header recv at %d", sr->read_ptr);
			}

			// jump to next byte and start scanning
			else {
				sr_advance_read_ptr(&from_pi, nsp->num_bytes_to_read);
				nsp_set_for_header_scan(nsp);
			}
		}
	}
}


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	  HAL_UART_Transmit(&huart1, inbuffer, 4, 0xFFFF);
//	  HAL_UART_Receive_DMA(&huart1, inbuffer, 4);


	if (nsp_data.rx_buffer[0] == IDENT_BYTE_0 &&
			nsp_data.rx_buffer[1] == IDENT_BYTE_1 &&
			nsp_data.rx_buffer[2] == IDENT_BYTE_2 &&
			nsp_data.rx_buffer[3] == TOGGLE_LIGHT_TYPE
			)
	{
		actually_light_stuff_up = !actually_light_stuff_up;
	}


	// Initiate the next read
//	nsp_start_read(&nsp_data);

	// Hered we'd call into the dispatching/read
	//actually_light_stuff_up = !actually_light_stuff_up;
}
*/

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

struct FooBar {
  int x;
  int rgb;
};



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void update(struct FooBar* fb) {
	  fb->x += 1;
	  if (fb->x > 255) {
		  fb->x = 0;

		  fb->rgb = (fb->rgb + 1) % 4;
	  }
}

void draw(int x, struct FooBar* fb) {
  	if(fb->rgb == 3)
  		ARGB_SetWhite(x, fb->x);
  	else
  		ARGB_SetRGB(x, fb->rgb==0?fb->x:0, fb->rgb==1?fb->x:0, fb->rgb==2?fb->x:0);


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t cur_write_ptr)
{
	// size is num bytes that have been written into my buffer; caps out at buffer size
	// force a wrap to zero if we hit the edge of the read buffer
	from_pi.write_ptr = cur_write_ptr == from_pi.buf_size ? 0 : cur_write_ptr;

	// Initiate another read
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, from_pi.buffer, UART_RX_BUFFER_SIZE);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);	// not sure about this...
  ARGB_Init();
  HAL_Delay(500);	// not sure about this...
//
//  uint32_t the_clock = HAL_RCC_GetSysClockFreq();
//
//
  ARGB_Clear(); // Clear stirp

    while (!ARGB_Show());
    HAL_Delay(500);	// not sure about this..




//    nsp_start_read(&nsp_data);

//
//  ARGB_Clear(); // Clear stirp
//  while (!ARGB_Show());


//  ARGB_Clear(); // Clear stirp
//  while (!ARGB_Show());


  int delay = 50;
//  int delay_inc = 50;
//  int delay_dir = 1;


	sr_init(&from_pi, UART_RX_BUFFER_SIZE);


	nsp_init(&nsp_data, &from_pi);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, from_pi.buffer, UART_RX_BUFFER_SIZE);



//  ARGB_FillRGB(0, 128, 0);
//  while (!ARGB_Show());


  struct FooBar yeah0 = {0,0};
  struct FooBar yeah1 = {128,1};
  struct FooBar yeah2 = {200,2};

//  uint8_t buffer[4];
//  buffer[0] = 'A';
//  buffer[1] = 'B';
//  buffer[2] = 'C';
//  buffer[3] = 'D';





//  HAL_UART_Receive_DMA(&huart1, inbuffer, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_ping_tick = HAL_GetTick();
  while (1)
  {
	  delay = 5;
	  HAL_Delay(delay);







	  // Ping every second
	  uint32_t cur_ping_tick = HAL_GetTick();
	  if (cur_ping_tick - last_ping_tick > 1000) {
//		  nsp_send_ping_packet(&nsp_data);


//		  nsp_print(&nsp_data, "abcd %d", counter++);

		  nsp_send_ping_packet(&nsp_data);

		  last_ping_tick = cur_ping_tick;
	  }



//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//	  HAL_Delay(delay);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);




		 nsp_process_rx(&nsp_data);





	  update(&yeah0);
	  update(&yeah1);
	  update(&yeah2);

	  ARGB_Clear();
	  if (actually_light_stuff_up) {
		  draw(0, &yeah0);
		  draw(1, &yeah1);
		  draw(2, &yeah2);
	  }
	  while (!ARGB_Show());

//	  	if(rgb == 3)
//	  		ARGB_SetWhite(0, x);
//	  	else
//	  		ARGB_SetRGB(0, rgb==0?x:0, rgb==1?x:0, rgb==2?x:0);



//	    HAL_Delay(500);	// not sure about this..


	  // This works! Huzzah
	    //HAL_UART_Transmit(&huart1, buffer, 4, 0xFFFF);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENC_A_Pin SENC_B_Pin */
  GPIO_InitStruct.Pin = SENC_A_Pin|SENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SENC_SW_Pin */
  GPIO_InitStruct.Pin = SENC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENC_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// global/static
volatile int encoder_count = 0;
static uint8_t last_state = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SENC_A_Pin || GPIO_Pin == SENC_B_Pin) {
        // Read current 2-bit state
        uint8_t a = HAL_GPIO_ReadPin(SENC_A_GPIO_Port, SENC_A_Pin);
        uint8_t b = HAL_GPIO_ReadPin(SENC_B_GPIO_Port, SENC_B_Pin);
        uint8_t state = (a << 1) | b;

        // Combine last + current into 4-bit index
        uint8_t index = (last_state << 2) | state;

        // Lookup table: +1 for CW, -1 for CCW, 0 for invalid/no move
        static const int8_t table[16] = {
            0, -1, +1,  0,
            +1,  0,  0, -1,
            -1,  0,  0, +1,
            0,  +1, -1,  0
        };

        encoder_count += table[index];
        nsp_print(&nsp_data, "encoder %d", encoder_count);
        last_state = state;
    }
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
