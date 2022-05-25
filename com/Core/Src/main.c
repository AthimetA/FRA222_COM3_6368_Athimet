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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0b10100000
#define WAIT_ADDR 0x0A
#define OPER_ADDR 0x1A
#define ENDS_ADDR 0x3A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
 // Robot
 typedef struct{
	 // val 0-255
	uint8_t WaitingTimeBuffer;
	uint8_t OperationTimeBuffer ;
	uint8_t EndStationBuffer ;
 	uint8_t StartStation ;
 	uint8_t WaitingTime;
 	uint8_t OperationTime ;
 	uint8_t EndStation ;

 }RobotManagement;

 static RobotManagement Robot;
 // State Machine
 static enum {init,StanBy,ParamSetting,StantionChoosing,EEpromWriteState,EEpromReadState} MCState = init;
 static enum {UserChooseWhatToDo,WaitingTimeEdit,OperationTimeEdit} ParamEditState = UserChooseWhatToDo;
 static enum {UserChooseStation,RobotOperating} StantionChoosingState = UserChooseStation;
 // UART PROTOCAL
 // Buffer
 char TxDataBuffer[64] =
 { 0 };
 // 1 key Only Program
 char RxDataBuffer[2] =
 { -1 };
 int8_t flagUART = 0;
 int16_t inputchar;

 // I2C
 uint8_t eepromWriteFlag = 0;
 uint8_t eepromReadFlag = 0;
 uint8_t eepromDataReadBack[3];
 uint16_t dataLen = 3 ;
 static uint8_t Senddata[3] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void StateMachineManagment();
int16_t UARTRecieveIT();
void EEPROMWriteFcn(uint8_t *Wdata, uint16_t len, uint16_t MemAd);
void EEPROMReadFcn(uint8_t *Rdata, uint16_t len, uint16_t MemAd);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  StateMachineManagment();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void StateMachineManagment()
{
	switch (MCState)
	{
		case init:
			// Header
			sprintf(TxDataBuffer, "\r\n---Program Start---\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			// Main
			// State init
			ParamEditState = UserChooseWhatToDo;
			StantionChoosingState = UserChooseStation;
			// Init Robot
			Robot.WaitingTime = 0;
			Robot.OperationTime = 0;
			Robot.StartStation = 0;
			Robot.EndStation = 0;
			// EEPROM READ
			eepromReadFlag = 1;
			dataLen = 3;
			EEPROMReadFcn(eepromDataReadBack,dataLen,WAIT_ADDR);
			// Wait for Data
			HAL_Delay(100);
			// Wait for Data
			Robot.WaitingTime = eepromDataReadBack[0];
			Robot.OperationTime = eepromDataReadBack[1];
			Robot.StartStation = eepromDataReadBack[2];
			Robot.EndStation = eepromDataReadBack[2];
			Robot.WaitingTimeBuffer = Robot.WaitingTime;
			Robot.OperationTimeBuffer = Robot.OperationTime;
			Robot.EndStationBuffer = Robot.EndStation;
			// End
			flagUART = 0;
			MCState = StanBy;
			break;
		case StanBy:
			// Header
			if(flagUART == 0){
				sprintf(TxDataBuffer, "\r\n---------------------------\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\nPlease Select Mode\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n+Type 1 for Robot Parameter Setting\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n+Type 2 for Choosing Destination Station\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n+Type 3 for EEPROM READ\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n---------------------------\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				flagUART = 1;
			}
			// Main
			HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
			inputchar = UARTRecieveIT();
			if(inputchar!=-1)
			{
				if(inputchar == '1')
				{
					flagUART = 0;
					MCState = ParamSetting;
				}
				else if(inputchar == '2')
				{
					flagUART = 0;
					MCState = StantionChoosing;
				}
				else if(inputchar == '3')
				{
					flagUART = 0;
					MCState = EEpromReadState;
				}
				else if(inputchar == '4')
				{
					flagUART = 0;
					MCState = EEpromWriteState;
				}
				else
				{
					flagUART = 0;
					sprintf(TxDataBuffer, "\r\n---Wrong Command---\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				}
			}
			break;
		case EEpromReadState:
			eepromReadFlag = 1;
			EEPROMReadFcn(eepromDataReadBack,dataLen,WAIT_ADDR);
			flagUART = 0;
			MCState = StanBy;
			break;
		case EEpromWriteState:
			eepromWriteFlag = 1;
			Senddata[0] = Robot.WaitingTime;
			Senddata[1] = Robot.OperationTime;
			Senddata[2] = Robot.EndStation;
			EEPROMWriteFcn(Senddata, dataLen, WAIT_ADDR);
			flagUART = 0;
			MCState = StanBy;
			break;
		case ParamSetting:
			// Sub-state
			switch (ParamEditState)
			{
				case UserChooseWhatToDo:
					// Header
					if(flagUART == 0){
						sprintf(TxDataBuffer, "\r\n---------------------------\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "\r\nPlease Select Parameter to edit\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "\r\nRobot Status WaitingTime:[%d] OperationTime[%d]", Robot.WaitingTimeBuffer, Robot.OperationTimeBuffer);
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, " Start Station:[%d] End Station[%d]\r\n", Robot.StartStation, Robot.EndStation);
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "\r\n+Type 1 for WaitingTime Edit\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type 2 for OperationTime Edit\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type s to save\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type x to cancel\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "\r\n---------------------------\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						flagUART = 1;
					}
					// Main
					HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
					inputchar = UARTRecieveIT();
					if(inputchar!=-1)
					{
						if(inputchar == 'x')
						{
							Robot.WaitingTimeBuffer = Robot.WaitingTime;
							Robot.OperationTimeBuffer = Robot.OperationTime;
							flagUART = 0;
							MCState = StanBy;
						}
						else if(inputchar == '1')
						{
							flagUART = 0;
							ParamEditState = WaitingTimeEdit;
						}
						else if(inputchar == '2')
						{
							flagUART = 0;
							ParamEditState = OperationTimeEdit;
						}
						else if(inputchar == 's')
						{
							Robot.WaitingTime = Robot.WaitingTimeBuffer;
							Robot.OperationTime = Robot.OperationTimeBuffer;
							eepromWriteFlag = 1;
							Senddata[0] = Robot.WaitingTime;
							Senddata[1] = Robot.OperationTime;
							Senddata[2] = Robot.EndStation;
							EEPROMWriteFcn(Senddata, dataLen, WAIT_ADDR);
							flagUART = 0;
							MCState = StanBy;
						}
						else
						{
							flagUART = 0;
							sprintf(TxDataBuffer, "\r\n---Wrong Command---\r\n");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
					}
					break;
				case WaitingTimeEdit:
					// Header
					if(flagUART == 0){
						sprintf(TxDataBuffer, "\r\n---WaitingTime Edit---\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type + for +1 second\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type - for -1 second\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type x to back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						flagUART = 1;
					}
					// Main
					HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
					inputchar = UARTRecieveIT();
					if(inputchar!=-1)
					{
						if(inputchar == 'x')
						{
							flagUART = 0;
							ParamEditState = UserChooseWhatToDo;
						}
						else if(inputchar == '+')
						{
							Robot.WaitingTimeBuffer++;
							sprintf(TxDataBuffer, "Current WaitingTime:[%d]\r\n", Robot.WaitingTimeBuffer);
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
						else if(inputchar == '-')
						{
							Robot.WaitingTimeBuffer--;
							sprintf(TxDataBuffer, "Current WaitingTime:[%d]\r\n", Robot.WaitingTimeBuffer);
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
						else
						{
							flagUART = 0;
							sprintf(TxDataBuffer, "\r\n---Wrong Command---\r\n");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
					}
					break;
				case OperationTimeEdit:
					// Header
					if(flagUART == 0){
						sprintf(TxDataBuffer, "\r\n---OperationTime Edit---\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type + for +1 second\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type - for -1 second\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						sprintf(TxDataBuffer, "+Type x to back\r\n");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						flagUART = 1;
					}
					// Main
					HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
					inputchar = UARTRecieveIT();
					if(inputchar!=-1)
					{
						if(inputchar == 'x')
						{
							flagUART = 0;
							ParamEditState = UserChooseWhatToDo;
						}
						else if(inputchar == '+')
						{
							Robot.OperationTimeBuffer++;
							sprintf(TxDataBuffer, "Current OperationTimeEdit:[%d]\r\n", Robot.OperationTimeBuffer);
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
						else if(inputchar == '-')
						{
							Robot.OperationTimeBuffer--;
							sprintf(TxDataBuffer, "Current OperationTimeEdit:[%d]\r\n", Robot.OperationTimeBuffer);
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
						else
						{
							flagUART = 0;
							sprintf(TxDataBuffer, "\r\n---Wrong Command---\r\n");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
						}
					}
					break;
			}
			break;
		case StantionChoosing:
			// Header
			if(flagUART == 0){
				sprintf(TxDataBuffer, "\r\nPlease Select Destination Station\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\nRobot Status WaitingTime:[%d] OperationTime[%d]", Robot.WaitingTimeBuffer, Robot.OperationTimeBuffer);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, " Start Station:[%d] End Station[%d]\r\n", Robot.StartStation, Robot.EndStation);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "+Type + for +1 Station\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "+Type - for -1 Station\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n+Type x to cancel\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				sprintf(TxDataBuffer, "\r\n+Type g to Start Operating\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				flagUART = 1;
			}
			// Main
			HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
			// Main
			HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 2);
			inputchar = UARTRecieveIT();
			if(inputchar!=-1)
			{
				if(inputchar == 'x')
				{
					Robot.EndStationBuffer = Robot.EndStation;
					flagUART = 0;
					ParamEditState = UserChooseWhatToDo;
				}
				else if(inputchar == '+')
				{
					Robot.EndStationBuffer++;
					Robot.EndStationBuffer %= 16;
					sprintf(TxDataBuffer, "Current End Station:[%d]\r\n", Robot.EndStationBuffer);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				}
				else if(inputchar == '-')
				{
					Robot.EndStationBuffer--;
					Robot.EndStationBuffer %= 16;
					sprintf(TxDataBuffer, "Current End Station:[%d]\r\n", Robot.EndStationBuffer);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				}
				else if(inputchar == 'g')
				{
					Robot.EndStation = Robot.EndStationBuffer;
					flagUART = 0;
					MCState = EEpromWriteState;
				}
				else
				{
					flagUART = 0;
					sprintf(TxDataBuffer, "\r\n---Wrong Command---\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				}
			}
			break;
	}
}

void EEPROMWriteFcn(uint8_t *Wdata, uint16_t len, uint16_t MemAd) {
	if (eepromWriteFlag && hi2c1.State == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_ADDR, MemAd, I2C_MEMADD_SIZE_16BIT,
				Wdata, len);
		eepromWriteFlag = 0;
	}
}
void EEPROMReadFcn(uint8_t *Rdata, uint16_t len, uint16_t MemAd) {
	if (eepromReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, MemAd, I2C_MEMADD_SIZE_16BIT,
				Rdata, len);
		eepromReadFlag = 0;
	}
}

int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
//	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
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
