/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "menu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern pFunction JumpToApplication;
extern uint32_t JumpAddress;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern char USBHPath[4];   /* USBH logical drive path */
FATFS myUsbFatFS;

FIL myFile;
FRESULT res;
UINT byteswritten, bytesread;
char rwtext[100];
char msg[100];
unsigned int x;
unsigned char pointer,hexBuffer[0xFF];
bool startPacket,newPacket;
void CopyAppToUserMemory(void);
void debug(char *text);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern ApplicationTypeDef Appli_state;

// fatfs variable

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

#define APP_BLOCK_TRANSFER_SIZE 512
#define SIZE_OF_U32 4


#define FLASH_PAGE_SIZE		2048 						//2 Kbyte per page
#define FLASH_START_ADDR	0x08000000					//Origin
#define FLASH_MAX_SIZE		0x00080000					//Max FLASH size - 512 Kbyte
#define FLASH_END_ADDR		(FLASH_START_ADDR + FLASH_MAX_SIZE)		//FLASH end address
#define FLASH_BOOT_START_ADDR	(FLASH_START_ADDR)				//Bootloader start address
#define FLASH_BOOT_SIZE		0x00020000					//64 Kbyte for bootloader
						//	0x08010000
#define FLASH_USER_START_ADDR	(FLASH_BOOT_START_ADDR + FLASH_BOOT_SIZE)	//User application start address
#define FLASH_USER_SIZE		0x00032000					//200 Kbyte for user application
#define FLASH_MSD_START_ADDR	(FLASH_USER_START_ADDR + FLASH_USER_SIZE)	//USB MSD start address
#define FLASH_MSD_SIZE		0x00032000					//200 Kbyte for USB MASS Storage
#define FLASH_OTHER_START_ADDR	(FLASH_MSD_START_ADDR + FLASH_MSD_SIZE)		//Other free memory start address
#define FLASH_OTHER_SIZE	(FLASH_END_ADDR - FLASH_OTHER_START_ADDR)	//Free memory size

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char msg[100];

uint32_t appSize;
unsigned char appBuffer[512];
unsigned int readBytes;
unsigned int i;
unsigned char programmed = 0;
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
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  programmed=0;
  debug("\r\n\r\n");
  debug("##############################################\r\n");
  debug("#             STM32f4 BootLoader             #\r\n");
  debug("##############################################\r\n");
  debug("\r\n");
  //printf("Bootloader ...");
  /* Check PA0 Input */

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET)
  {
	  debug("Jumping to the application address:");

	  // jump to the application
	  if (((*(__IO uint32_t*)FLASH_USER_START_ADDR) & 0x2FFD0000 ) == 0x20000000)
	  {
		  sprintf(msg,"%d",FLASH_USER_START_ADDR);
		  debug(msg);
			// Jump to user application
			JumpAddress = *(__IO uint32_t*) (FLASH_USER_START_ADDR + 4);

			JumpToApplication = (pFunction) JumpAddress;
			// Initialize user application's Stack Pointer
			__set_MSP(*(__IO uint32_t*) FLASH_USER_START_ADDR);
			JumpToApplication();
		}
	  debug("\r\n");
  }
  else
  {
	  debug("Programming Mode ... \r\n");
	  //HAL_UART_Transmit(&huart2,(unsigned char*)msg,strlen(msg),100);
	  bool mountCheck;

	  mountCheck = false;

	  while(1)
	  {
		  	MX_USB_HOST_Process();

		  	switch(Appli_state)
			{
				case APPLICATION_IDLE:
					break;
				case APPLICATION_START:
					if(f_mount(&myUsbFatFS, (TCHAR const*)USBHPath, 0) != FR_OK)
					{
						/* FatFs Initialization Error */
						//Error_Handler();
						if(mountCheck == false)
						{
							debug("failed to mount ..\r\n");
							mountCheck = true;
						}
					}
					else
					{
						HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
						if(mountCheck == false)
						{
							debug("drive mounted ..\r\n");
							mountCheck = true;
						}
					}

					break;
				case APPLICATION_READY:
					//use here

					if(f_open(&myFile, "BLINK.BIN", FA_READ) == FR_OK)
					{
						debug("file opened ..\r\n");
						appSize = f_size(&myFile);

						debug("file size : ");
						sprintf(msg,"%d",appSize);
						debug(msg);
						debug("\r\n");

						for(i = 0; i < appSize; i++) //Byte-to-byte compare files in MSD_MEM and USER_MEM
						{
							f_read(&myFile, &appBuffer, 1, &readBytes);
							if(*((volatile uint8_t*)(USER_START_ADDRESS + i)) != appBuffer[0])
							{
								//if byte of USER_MEM != byte of MSD_MEM
								break;
							}
						}
						if(i != appSize)//=> was done "break" instruction in for(;;) cycle => new firmware in MSD_FLASH
						{
							debug("copying bin file in to memory ..\r\n");
							CopyAppToUserMemory();
							debug("App installed\r\n");
						}
						f_close(&myFile);
						debug("closing file\r\n");
						programmed=1;
						//FATFS_Status = f_mount(NULL, "0", 1);
						//PeriphDeInit();
						//GoToUserApp();
					}
					else
					{
						debug("error opening file ..\r\n");
					}
				case APPLICATION_DISCONNECT:
					// if the pendrive is in disconnected state
					debug("disconnecting drive ..\r\n");
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,GPIO_PIN_RESET);
					break;

			}

		  	if (programmed != 0)
		  	{
		  		debug("exiting programming mode ..\r\n");
		  		break;
		  	}




	  }



  }
  /*
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET)
  {
	//
	if (((*(__IO uint32_t*)USER_START_ADDRESS) & 0x2FFD0000 ) == 0x20000000)
	{
		// Jump to user application
		JumpAddress = *(__IO uint32_t*) (USER_START_ADDRESS + 4);
		JumpToApplication = (pFunction) JumpAddress;
		// Initialize user application's Stack Pointer
		__set_MSP(*(__IO uint32_t*) USER_START_ADDRESS);
		JumpToApplication();
	}

  }
  else
  {
	FLASH_If_Init();
	Main_Menu ();
	MX_USB_HOST_Process();
  }
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  debug("running wait loop ..\r\n");
  while (1)
  {
	if(programmed == 0)
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);
	else
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin);
	HAL_Delay(500);
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin ORANGE_LED_Pin RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



void CopyAppToUserMemory(void)
{
	uint32_t appTailSize,appBodySize,appAddrPointer,j;
	#define APP_BLOCK_TRANSFER_SIZE 512
	#define SIZE_OF_U32 4
	appTailSize = 1024;

	HAL_FLASH_Unlock();

	debug("function: copy use code ....\r\n app size:");
	sprintf(msg,"%d",appSize);
	debug(msg);
	debug("\r\n");

	f_lseek(&myFile, 0); //Go to the fist position of file
	appTailSize = appSize % APP_BLOCK_TRANSFER_SIZE;
	appBodySize = appSize - appTailSize;
	appAddrPointer = 0;


	debug("appTailSize :");
	sprintf(msg,"%d",appTailSize);
	debug(msg);
	debug("\r\n");

	debug("appBodySize :");
	sprintf(msg,"%d",appBodySize);
	debug(msg);
	debug("\r\n");
	/*
	for(i = 0; i < ((appSize / FLASH_PAGE_SIZE) + 1); i++) //Erase n + 1 pages for new application
	{
		//while (FLASH_GetStatus() != HAL_OK);
		//FLASH_ErasePage(FLASH_USER_START_ADDR + i * FLASH_PAGE_SIZE);
		FLASH_If_Erase(FLASH_USER_START_ADDR + i * FLASH_PAGE_SIZE);
	}
	*/
	debug("erasing flash ..");

	FLASH_Erase_Sector(FLASH_SECTOR_5, FLASH_VOLTAGE_RANGE_3);

	for(i = 0; i < appBodySize; i += APP_BLOCK_TRANSFER_SIZE)
	{
		/*
		 * For example, size of File1 = 1030 bytes
		 * File1 = 2 * 512 bytes + 6 bytes
		 * "body" = 2 * 512, "tail" = 6
		 * Let's write "body" and "tail" to MCU FLASH byte after byte with 512-byte blocks
		 */
		f_read(&myFile, appBuffer, APP_BLOCK_TRANSFER_SIZE, &readBytes); //Read 512 byte from file
		for(j = 0; j < APP_BLOCK_TRANSFER_SIZE; j += SIZE_OF_U32) //write 512 byte to FLASH
		{
			//while(FLASH_GetStatus() != FLASH_COMPLETE);
			FLASH_WaitForLastOperation(100);
			//FLASH_ProgramWord(FLASH_USER_START_ADDR + i + j, *((volatile uint32_t*)(appBuffer + j)));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_USER_START_ADDR + i + j, *((volatile uint32_t*)(appBuffer + j)));
		}
		appAddrPointer += APP_BLOCK_TRANSFER_SIZE; //pointer to current position in FLASH for write
	}
	f_read(&myFile, appBuffer, appTailSize, &readBytes); //Read "tail" that < 512 bytes from file
	while((appTailSize % SIZE_OF_U32) != 0)		//if appTailSize MOD 4 != 0 (seems not possible, but still...)
	{
		appTailSize++;				//increase the tail to a multiple of 4
		appBuffer[appTailSize - 1] = 0xFF;	//and put 0xFF in this tail place
	}
	for(i = 0; i < appTailSize; i += SIZE_OF_U32) //write "tail" to FLASH
	{
		//while(FLASH_GetStatus() != FLASH_COMPLETE);
		FLASH_WaitForLastOperation(100);
	//	FLASH_ProgramWord(FLASH_USER_START_ADDR + appAddrPointer + i, *((volatile uint32_t*)(appBuffer + i)));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_USER_START_ADDR + appAddrPointer + i, *((volatile uint32_t*)(appBuffer + i)));
	}
	FLASH_WaitForLastOperation(100);
	HAL_FLASH_Lock();
}

void debug(char *text)
{
	//HAL_UART_Transmit(&huart3,text,strlen(text),100);
	HAL_UART_Transmit(&huart3,text,strlen(text),100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
