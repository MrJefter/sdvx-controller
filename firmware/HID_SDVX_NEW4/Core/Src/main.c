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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include <string.h>
#include <math.h>
#include "visEffect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	const GPIO_TypeDef* port;
	const uint16_t pin;
} Button_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	JOYSTICK_REPORT_SIZE	5
#define	ENCODER_TIMER_PERIOD	0xFFFF // 65535

#define NUM_BUTTONS				7
#define DEBOUNCE_LOCKOUT_MS		3
#define MAIN_LOOP_DELAY_MS		1

#define ENCODER_SMOOTHING_ALPHA 0.15f

#define DFU_BUTTON_INDEX		6
#define DFU_HOLD_TIME_MS		5000
#define BOOT0_PORT				GPIOA
#define BOOT0_PIN				GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

USBD_JoystickReport_TypeDef joystickReport;

volatile int16_t g_encoder_delta_x = 0;
volatile int16_t g_encoder_delta_y = 0;

static uint16_t last_encoder1_count = 0;
static uint16_t last_encoder2_count = 0;
static int32_t current_axis_x_raw = 0;
static int32_t current_axis_y_raw = 0;

// Make button definitions const
const Button_TypeDef buttons[NUM_BUTTONS] = {
		{GPIOA, BTN1_Pin},
		{GPIOA, BTN2_Pin},
		{GPIOA, BTN3_Pin},
		{GPIOA, BTN4_Pin},
		{GPIOA, FXL_Pin},
		{GPIOA, FXR_Pin},
		{GPIOA, START_Pin}
};

// These are modified by EXTI callback, hence volatile
volatile uint8_t button_stable_state[NUM_BUTTONS] = {0};
volatile uint32_t button_lockout_timer[NUM_BUTTONS] = {0};

static float smoothed_axis_x = 0.0f;
static float smoothed_axis_y = 0.0f;

// These are modified by EXTI callback, hence volatile
static volatile uint32_t dfu_button_press_time = 0;
static volatile uint8_t dfu_button_currently_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void GoToBootloader(void);
static void Read_Encoders(void);
static void Send_HID_Report(void);
static void Process_Buttons(void);
static void Check_DFU_Entry(void);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  visInit();

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }

  // Use defined constant for clarity if desired, though 0xFFFF is common
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  // Read initial counter values *after* setting them
  last_encoder1_count = __HAL_TIM_GET_COUNTER(&htim3);
  last_encoder2_count = __HAL_TIM_GET_COUNTER(&htim4);
  current_axis_x_raw = 0;
  current_axis_y_raw = 0;

  // Initialize report struct
  memset(&joystickReport, 0, sizeof(joystickReport));
  // Redundant if memset is used, but explicit:
  // joystickReport.axis_x = 0;
  // joystickReport.axis_y = 0;
  // joystickReport.buttons = 0;

  // memset(button_stable_state, 0, sizeof(button_stable_state)); // Already initialized statically
  // memset(button_lockout_timer, 0, sizeof(button_lockout_timer)); // Already initialized statically
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	visHandle();
	Read_Encoders();
	Process_Buttons();
	Check_DFU_Entry();
	Send_HID_Report();

	// Consider if HAL_Delay is strictly necessary or if timing can be handled differently
	// For simple loop rate control, it's acceptable.
	HAL_Delay(MAIN_LOOP_DELAY_MS);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = ENCODER_TIMER_PERIOD; // Use define
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; // Check if this matches encoder spec (often rising/falling)
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5; // Adjust filter based on noise/encoder speed
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING; // Check if this matches encoder spec
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5; // Adjust filter based on noise/encoder speed
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = ENCODER_TIMER_PERIOD; // Use define
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; // Check consistency
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5; // Adjust filter
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING; // Check consistency
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5; // Adjust filter
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(BOOT0_PORT, BOOT0_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN4_Pin BTN3_Pin BTN2_Pin FXL_Pin
                           BTN1_Pin FXR_Pin START_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN3_Pin|BTN2_Pin|FXL_Pin
                          |BTN1_Pin|FXR_Pin|START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Debounce handled in software
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Assumes buttons connect pin to GND when pressed
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 (BOOT0 Control) */
  GPIO_InitStruct.Pin = BOOT0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT0_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init - Ensure priorities are appropriate for the application */
  // Using the same priority (0,0) for all might be fine, but consider if some need higher priority
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Process stable button states into HID report format.
  * @retval None
  */
static void Process_Buttons(void)
{
	uint8_t final_button_state = 0;
	// This loop assumes button_stable_state[i] is always 0 or 1.
	for (int i = 0; i < NUM_BUTTONS; i++) {
		final_button_state |= (button_stable_state[i] << i);
	}
	joystickReport.buttons = final_button_state;
}

/**
  * @brief Check if DFU button is held long enough to enter bootloader.
  * @retval None
  */
static void Check_DFU_Entry(void)
{
    // Read volatile variables once
	uint32_t press_time = dfu_button_press_time;
	uint8_t is_pressed = dfu_button_currently_pressed;

	// Check only if the button is currently considered pressed and a start time is logged
	if (is_pressed && press_time != 0) {
		if ((HAL_GetTick() - press_time) >= DFU_HOLD_TIME_MS) {
			GoToBootloader(); // This function does not return
		}
	}
}

/**
 * @brief EXTI Line Detection Callback. Handles button debouncing.
 * @param GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t current_tick = HAL_GetTick();

	// Find which button triggered the interrupt
	for (int i = 0; i < NUM_BUTTONS; i++) {
		if (GPIO_Pin == buttons[i].pin) {
			// Check if lockout period has expired
			if (current_tick >= button_lockout_timer[i]) {
				// Read the *current* raw state (interrupt might be delayed slightly)
                // Assumes PULLUP resistor and button connects to GND when pressed
				uint8_t current_raw_state = (HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin) == GPIO_PIN_RESET);

                // Check if the stable state needs updating
				if (current_raw_state != button_stable_state[i]) {
					button_stable_state[i] = current_raw_state;
					// Start new lockout period
					button_lockout_timer[i] = current_tick + DEBOUNCE_LOCKOUT_MS;

					// Specific logic for the DFU button
					if (i == DFU_BUTTON_INDEX) {
						if (current_raw_state == 1) { // Button pressed
                            // Only record press time if it wasn't already considered pressed
                            // Protect against HAL_GetTick() potentially returning 0 initially
							if (dfu_button_press_time == 0) {
								dfu_button_press_time = current_tick ? current_tick : 1;
							}
							dfu_button_currently_pressed = 1;
						} else { // Button released
							dfu_button_press_time = 0; // Reset timer
							dfu_button_currently_pressed = 0;
						}
					}
				}
			}
			// Found the button, no need to check others
			break;
		}
	}
}

/**
  * @brief Set BOOT0 pin and trigger system reset to enter DFU bootloader.
  * @retval None. This function does not return.
  */
static void GoToBootloader(void) {
	HAL_GPIO_WritePin(BOOT0_PORT, BOOT0_PIN, GPIO_PIN_SET);
	HAL_Delay(250); // Short delay to ensure pin state is stable
	NVIC_SystemReset(); // Trigger reset
}

/**
  * @brief Read encoder values, calculate deltas, apply smoothing, and update report.
  * @retval None
  */
static void Read_Encoders(void)
{
    // Read hardware counters
	uint16_t current_encoder1_count = __HAL_TIM_GET_COUNTER(&htim3);
	uint16_t current_encoder2_count = __HAL_TIM_GET_COUNTER(&htim4);

    // Calculate delta using signed 16-bit arithmetic to handle wrap-around correctly
	int16_t delta1 = (int16_t)(current_encoder1_count - last_encoder1_count);
	int16_t delta2 = (int16_t)(current_encoder2_count - last_encoder2_count);

    // Store current counts for next iteration
	last_encoder1_count = current_encoder1_count;
	last_encoder2_count = current_encoder2_count;

    // Update global delta values (used by visEffect)
	g_encoder_delta_x = delta1;
	g_encoder_delta_y = delta2;

    // Accumulate raw encoder values
	current_axis_x_raw += delta1;
	current_axis_y_raw += delta2;

    // Apply exponential moving average (EMA) smoothing
	smoothed_axis_x = (ENCODER_SMOOTHING_ALPHA * (float)current_axis_x_raw) + ((1.0f - ENCODER_SMOOTHING_ALPHA) * smoothed_axis_x);
	smoothed_axis_y = (ENCODER_SMOOTHING_ALPHA * (float)current_axis_y_raw) + ((1.0f - ENCODER_SMOOTHING_ALPHA) * smoothed_axis_y);

    // Scale and round the smoothed value for the report
    // Adjust scaling factor (10.0f) as needed for desired sensitivity/range
	int32_t scaled_x = (int32_t)roundf(smoothed_axis_x / 10.0f);
	int32_t scaled_y = (int32_t)roundf(smoothed_axis_y / 10.0f);

    // Clamp scaled values to the report's axis range (uint16_t: 0-65535)
	joystickReport.axis_x = (uint16_t)scaled_x;
	joystickReport.axis_y = (uint16_t)scaled_y;
}


/**
  * @brief Send the current joystick HID report over USB.
  * @retval None
  */
static void Send_HID_Report(void)
{
    // Only send if USB device is configured and ready
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joystickReport, JOYSTICK_REPORT_SIZE);
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
  __disable_irq(); // Critical error, halt execution
  while (1) {}
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
   // Consider logging the error before halting
   Error_Handler(); // Halt on assert failure
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
