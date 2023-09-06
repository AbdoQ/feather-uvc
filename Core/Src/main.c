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
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_video.h"
#include "usbh_video_desc_parsing.h"
#include "usbh_video_stream_parsing.h"
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
SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */
extern uint32_t uvc_frame_cnt;
extern uint8_t uvc_parsing_new_frame_ready;
extern uint8_t* uvc_ready_framebuffer_ptr;
extern uint32_t uvc_ready_frame_length;
extern USBH_HandleTypeDef hUsbHostFS;
volatile uint8_t uvc_framebuffer0[UVC_MAX_FRAME_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
extern void USB_HOST_camera_process(void);
uint16_t yuy2_to_rgb565_bt601(uint8_t y, uint8_t u, uint8_t v);
void convert_yuy2_array_to_rgb565_inplace(uint8_t* yuy2_data, int pixel_count);
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
	unsigned int c;
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  // FATFS file initialization
  f_mount(&SDFatFS, SDPath, 1);
  f_open(&SDFile, "video", FA_CREATE_ALWAYS | FA_WRITE);
  f_write(&SDFile, (uint8_t[]){(uint8_t)UVC_TARGET_WIDTH, (uint8_t)UVC_TARGET_HEIGHT}, 2*sizeof(uint8_t), &c);// First 2-bytes: frame width, frame height

  USBH_VIDEO_Target_Format = USBH_VIDEO_YUY2;
  video_stream_init_buffers((uint8_t*)uvc_framebuffer0, (uint8_t*)uvc_framebuffer0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t firstFrameTick;
  uint32_t currentFrameTick;
  uint8_t firstFrameFlag=0;
  uint8_t stopRecording = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    USB_HOST_camera_process();
    if(uvc_parsing_new_frame_ready && stopRecording == 0) {
    	uvc_parsing_new_frame_ready = 0;
    	if(firstFrameFlag == 0) {
    		firstFrameFlag = 1;
    		firstFrameTick = HAL_GetTick();
    		currentFrameTick = firstFrameTick;
    	} else {
    		currentFrameTick = HAL_GetTick() - firstFrameTick;
    		if(currentFrameTick == 10*1000) {
    			// Stop recording after 10 seconds
    			stopRecording = 1;
    			f_close(&SDFile);
    		}
    	}

    	convert_yuy2_array_to_rgb565_inplace(uvc_ready_framebuffer_ptr, UVC_TARGET_WIDTH*UVC_TARGET_HEIGHT);
    	f_write(&SDFile, (uint8_t*)&currentFrameTick, sizeof(currentFrameTick), &c); // Write frame time, in ms relative to first frame
    	f_write(&SDFile, uvc_ready_framebuffer_ptr, 2*UVC_TARGET_WIDTH*UVC_TARGET_HEIGHT, &c);// Write frame data in RGB565 format

		video_stream_ready_update();// Call this to signal processing complete and buffer is available for next frame
    } else {
    	//HAL_Delay(1000);
    	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t yuy2_to_rgb565_bt601(uint8_t y, uint8_t u, uint8_t v) {
    // Convert YUY2 to RGB24 using BT.601
    int r = y + 1.403 * (v - 128);
    int g = y - 0.344 * (u - 128) - 0.714 * (v - 128);
    int b = y + 1.773 * (u - 128);

    // Clamp the values to [0, 255]
    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);

    // Convert RGB24 to RGB565
    uint16_t r5 = r >> 3;
    uint16_t g6 = g >> 2;
    uint16_t b5 = b >> 3;

    return (r5 << 11) | (g6 << 5) | b5;
}

void convert_yuy2_array_to_rgb565_inplace(uint8_t* yuy2_data, int pixel_count) {
    for (int i = 0; i < pixel_count; i += 2) {
        uint8_t y0 = yuy2_data[2 * i];
        uint8_t u = yuy2_data[2 * i + 1];
        uint8_t y1 = yuy2_data[2 * i + 2];
        uint8_t v = yuy2_data[2 * i + 3];

        uint16_t rgb565_0 = yuy2_to_rgb565_bt601(y0, u, v);
        uint16_t rgb565_1 = yuy2_to_rgb565_bt601(y1, u, v);

        // Store RGB565 values in the same YUY2 array in RGB order
        yuy2_data[2 * i] = rgb565_0 >> 8;         // Upper byte of first RGB565 value (RRRRRGGG)
        yuy2_data[2 * i + 1] = rgb565_0 & 0xFF;   // Lower byte of first RGB565 value (GGGBBBBB)
        yuy2_data[2 * i + 2] = rgb565_1 >> 8;     // Upper byte of second RGB565 value (RRRRRGGG)
        yuy2_data[2 * i + 3] = rgb565_1 & 0xFF;   // Lower byte of second RGB565 value (GGGBBBBB)
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
