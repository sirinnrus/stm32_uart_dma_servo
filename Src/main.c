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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
typedef uint16_t WORD;
typedef uint8_t BYTE;
#define MIN_SPEED_VAL 390
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

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
unsigned int CRC16_2(unsigned char* buf, int len);
void createMessage(unsigned char *buf, int w_r, int RegNum, int Rval, int val);
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
	//MX_ETH_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();

	unsigned char msgRead[8];
	int i, brute, counter=0, counterError=0;

	uint8_t RxDat[7];
	uint8_t RxDat_w[8];
	uint8_t write_vals[50]={};
	uint16_t value_speed;
	HAL_GPIO_WritePin(rspv_GPIO_Port, rspv_Pin, GPIO_PIN_SET);

	//стоп
	//{ 0x01, 0x06, 0x01, 0x04, 0x00, 0x00 }
	//запуск
	//{ 0x01, 0x06, 0x01, 0x04, 0x00, 0x01 }

	//{ 0x01, 0x03, 0x01, 0x20, 0x00, 0x01 };
	//Чтение регистров Px-x.  Второй байт команда 03
	//{ 0x01, 0x03, 0x01, 0x03, 0x00, 0x01 }
	// 3й байт P№ ; 4й байт -"номер регистра"

	//Запись
	//2й байт команда записи 3й байт P№ ; 4й байт -"номер регистра" ; 5-6 значение
	//{ 0x01, 0x06, 0x01, 0x04, 0x00, 0x03 }

	//регистр Fn1 1281(dec) - 501(hex)
	//{ 0x01, 0x03, 0x05, 0x01, 0x00, 0x01 };

    while (1)
    {
    	HAL_GPIO_TogglePin(GPIOB,LD1_Pin);
    	unsigned int crc;
    	unsigned char crcmsg[8];

    	//Если есть сомнения в формировании отправленного сообщения
    	//    		unsigned char first, second;
    	//    		int i;
    	//
    	//    		unsigned char msg[] = { 0x01, 0x06, 0x01, 0x04, 0x00, 0x01 };
    	//    		size_t msg_size = sizeof(msg);
    	//
    	//    		crc = CRC16_2(msg, msg_size);
    	//
    	//    		for (i=0;i<6;i++){
    	//    			crcmsg[i]=msg[i];
    	//    		}
    	//    		first = (crc >> 0) & 0xff;
    	//    		second = (crc >> 8) & 0xff;
    	//    		crcmsg[6] = first;
    	//    		crcmsg[7] = second;
    	HAL_GPIO_WritePin(GPIOB,LD2_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB,LD3_Pin, GPIO_PIN_RESET);


    	//запуск и остановка
		createMessage(crcmsg, 6, 1, 4 ,1);
		uint8_t* txdata = (uint8_t*)crcmsg;
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
		HAL_Delay(1000);
		createMessage(crcmsg, 6, 1, 4 ,0);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
		HAL_Delay(4000);

		createMessage(crcmsg, 6, 2, 2 ,281); //изменение значения регистра p2-2
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
		HAL_Delay(1000);

		//запуск двигателя для проверки считывания скорости
		createMessage(crcmsg, 6, 1, 4 ,1);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
		HAL_Delay(1000);

		createMessage(crcmsg, 3, 5, 8 ,1); //читаем регистр F8
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat, 7);
		HAL_Delay(1000);

		value_speed = RxDat[3] << 8 | RxDat[4];

		//проверка вращения у меня в f8 при движении двигателя записано 400. Сравнивать со своим значением
		if (value_speed>=MIN_SPEED_VAL){
			for(i=0;i<10;i++){
				HAL_GPIO_TogglePin(GPIOB,LD2_Pin);
				HAL_Delay(500);}
		}
			printf("Hello_");
		HAL_Delay(10000);

		//проверка регистров P
		for (brute=0; brute<=12;brute++){
			if (brute==12){
				printf("%d", brute);
			}
			createMessage(crcmsg, 3, 1, brute ,1);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat, 7);
			HAL_Delay(500);
			//если пришел правильный ответ
			if (RxDat[0]==1 && RxDat[1]==3){
//				HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
//				HAL_Delay(100);
				write_vals[brute]=RxDat[4];
				}
			//если пришел ответ правильный, но сдвинутый на 1 байт
			else if (RxDat[0]==247 && RxDat[0]==1){
				//uint8_t* subRxDat = RxDat + 1;
				uint8_t subRxDat[8];
				for(i = 0; i < 8; i++) {
					subRxDat[i] = RxDat[i+1];
					}
				if (subRxDat[0]==1 && subRxDat[1]==3){
//					HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
//					HAL_Delay(100);
//					HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
					write_vals[brute]=subRxDat[4];
					}
				else {
//					HAL_GPIO_WritePin(GPIOB,LD3_Pin, GPIO_PIN_SET);
//					HAL_Delay(2000);
					write_vals[brute]=-1;
					counterError++;
					}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SON_GPIO_Port, SON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rspv_GPIO_Port, rspv_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SON_Pin */
  GPIO_InitStruct.Pin = SON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : rspv_Pin */
  GPIO_InitStruct.Pin = rspv_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(rspv_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
unsigned int CRC16_2(unsigned char* buf, int len)
{
	unsigned int crc = 0xFFFF;
	for (int pos = 0; pos < len; pos++)
	{
		crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}

	return crc;
}

void createMessage(unsigned char *buf, int w_r, int RegNum, int Rval, int val){
	//buf буфер (массив с размером 8)
	//w_r для записи в задаваемый регистр устанавливаем 6, для чтения -3
	//RegNum старший байт регистра P|F
	//Rval младший байт регистра P|F
	//значение записываемое в указанный регистр в десятичной системе

	//createMessage(crcmsg, 6, 1, 4 ,1); запуск
	//createMessage(crcmsg, 6, 1, 4 ,0); остановка
	//createMessage(crcmsg, 6, 2, 2 ,281); изменение значения регистра p2-2
	int i;
	unsigned int crc;
	unsigned char first, second;
	uint8_t hbyte, lbyte;

	hbyte = val >> 8;
	lbyte = val & 0x00ff;
	unsigned char msg[] = { 0x01, w_r, RegNum, Rval, hbyte, lbyte };
	size_t msg_size = sizeof(msg);
	crc = CRC16_2(msg, msg_size);

	for (i=0;i<6;i++){
		buf[i]=msg[i];
	}
	first = (crc >> 0) & 0xff;
	second = (crc >> 8) & 0xff;
	buf[6] = first;
	buf[7] = second;

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2)
//{
//
//	unsigned int crc;
//	unsigned char first, second;
//	int i;
//	crc=i;
//
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//	unsigned int crc;
//	unsigned char first, second;
//	int i;
//	unsigned char crcmsg[8];
//	createMessage(crcmsg, 6, 1, 4 ,1);
//	uint8_t* txdata = (uint8_t*)crcmsg;
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
//
//	HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
//	HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_UART_TxCpltCallback can be implemented in the user file.
//   */
//}
//void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
//	unsigned char crcmsg[8];
//	createMessage(crcmsg, 3, 1, 4 ,1);
//	uint8_t* txdata = (uint8_t*)crcmsg;
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
//	HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat, 7);
//
//	if (RxDat[4]==0){
//		HAL_GPIO_TogglePin(GPIOB,LD2_Pin);
//		createMessage(crcmsg, 6, 1, 4 ,1);
////		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
////
////		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
//		HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
//	}
//	else if (RxDat[4]==1){
//		HAL_GPIO_TogglePin(GPIOB,LD1_Pin);
////		createMessage(crcmsg, 6, 1, 4 ,0);
////		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txdata, 8);
////		HAL_UART_Receive_DMA(&huart2, (uint8_t*)RxDat_w, 8);
//
//	}
//
//}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_UART_ErrorCallback can be implemented in the user file.
//   */
//}

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
