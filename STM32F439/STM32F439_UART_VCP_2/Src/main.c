/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
// RoSchmi 18.07.2019:
// This application was made for the NUCLEO-STM32F429ZI / F439ZI, may run on other STM32F4 Boars as well
//
// The program demonstrates the use of serial connection through USART and USB_Device Virtual COM Port (VCP, CDC)
//
// Data are sent and received through serial ports (e.g. Teraterm can be used as 2 Terminals)

//
// When data are sent through the CDC virtual COM Port, the function  CDC_Transmit_FS in usbd_cdc_if.c is used
// When data are written through the UART3 (STLink Virtual COM Port) HAL_UART_Transmit in stm32f4xx_hal_uart.c is used
//
// Data sent to the USB CDC Port (CN13) came in through the CDC_Receive_FS callback function in usbd_cdc_if.c
// This function shall not be called by the user code.
// The content of the buffer is copied and transfered to the user program via the VCP_retrieveInputData function in usbd_cdc_if.c
//
// If the CDC Virtual COM Port is not enumerated be the connected PC, try to change the size of stack and heap in CubeMX
// Change stack size and heap size from 400-> 4000, 200->2000
// Furthermore try to set the Checkbox 'Generate peripheral initialization as a pair of 'c/h' files per peripheral
//
// These posts were helpful.
// https://stackoverflow.com/questions/33549084/stm32cubemx-usb-cdc-vcp
//
// STM32 USB CDC (Virtual Com Port) with CubeMX HAL in 6 minutes
// https://www.youtube.com/watch?v=AYICE0gU-Sg
//
// 13-USB_CDC-Library (STM32F429)
// http://mikrocontroller.bplaced.net/wordpress/?page_id=309
//
// STM32 USB CDC
// https://damogranlabs.com/2018/02/stm32-usb-cdc/
//
// Read data from PC to STM32 via USB CDC
// https://community.st.com/s/question/0D50X00009Xkfd1SAB/read-data-from-pc-to-stm32-via-usb-cdc
// Valuable help is also present in the op's bitbucket Project: https://bitbucket.org/rwmao/cdc_onstm32f411rc/src/master/
// (an 'HowTo .pdf and valuable snapshots showing important things in the initialization)
//
// Helpful Video about using interrupts with receive through UART
// https://www.youtube.com/watch?v=VdHt_wJdezM


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"
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

/* USER CODE BEGIN PV */
uint8_t ReadBuffer[100];
uint8_t MessageSent = 0;
uint32_t Len = 0;
uint8_t cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rec_Data[10];
uint8_t rx_index = 0;
uint8_t rx_data;
uint8_t rx_buffer[40];
uint8_t char_buffer[2];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		if (rx_index == 0)
		{
			for (int i = 0; i < 20; i++)
			{
				rx_buffer[i] = 0;
			}
		}

		// Calling CDC_Transmit_FS in usbd_cdc_if

		char_buffer[0] = rx_data;
		if (rx_data !=13 )			// if the charcter received is other than 'enter' ascii 13, save the data in buffer
		{
			rx_buffer[rx_index++] = rx_data;

			// Calling CDC_Transmit_FS in usbd_cdc_if
			CDC_Transmit_FS(char_buffer, 1);
		}
		else
		{
			char_buffer[1] = 0x0A;     // New line

			rx_buffer[rx_index++] = rx_data;
			rx_buffer[rx_index++] = 0x0A;
			rx_index = 0;
			HAL_UART_Transmit(&huart3, rx_buffer, sizeof(rx_buffer), 200);    // transmit the data via uart

			// Calling CDC_Transmit_FS in usbd_cdc_if
			CDC_Transmit_FS(char_buffer, 2);

		}

        HAL_UART_Receive_IT(&huart3, &rx_data,1);   // receive data (one character only)
	}


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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

   __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);  // This will enable UART Receive interrupt

   HAL_UART_Receive_IT(&huart3, &rx_data, 1);

    uint8_t HiMsgUART[] = "Hello UART, CDC is waiting to receive!\r\n";
    uint8_t HiMsgCDC[] = "Hello CDC, UART is waiting to receive!\r\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */


	  if (MessageSent == 0)
	  {
		  // Toggle LEDs to signal that program is running
          for (int i = 0; i < 5; i++)
          {
        	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		  	  HAL_Delay(200);
		  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		  	  HAL_Delay(200);
          }

		  // Calling HAL_UART_Transmit in stm32f4xx_hal_uart.c
		  HAL_UART_Transmit(&huart3,HiMsgUART,sizeof(HiMsgUART),100);// Sending in normal mode

		  // Calling CDC_Transmit_FS in usbd_cdc_if
		  CDC_Transmit_FS(HiMsgCDC, strlen((const char*)HiMsgCDC));

		  MessageSent = 1;
	  }

	  cnt = VCP_retrieveInputData(ReadBuffer, &Len);

	//  HAL_UART_Receive(&huart3, receiveData, 10, 100);

	  if (Len > 0)
	  {
		  HAL_UART_Transmit(&huart3,ReadBuffer,sizeof(ReadBuffer),Len);// Sending in normal mode
		  Len = 0;
	  }

  /* USER CODE END 3 */
  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
