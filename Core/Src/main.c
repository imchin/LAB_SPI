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
#include <stdio.h>
#include <string.h>
#include <math.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint32_t readadc=0;
uint64_t micros();
uint32_t _micro=0;
uint32_t dataOut=0;
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint32_t OutputPacket;
uint64_t timestamp=0;
char TxDataBuffer[64] ={ 0 };
char RxDataBuffer[64] ={ 0 };
void pim();
uint8_t Posdatapre=0;
uint8_t Posdata=0;
uint8_t state=0;
void pimstatus();
uint8_t Freq=0;
uint8_t Vhigh=0;
uint8_t Vlow=0;
uint8_t Dutycycle=0;
uint8_t stateform=0;
uint8_t slope=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&readadc , 1);
  uint8_t DACConfig = 0b0011;
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_UART_Receive_DMA(&huart2,  (uint8_t*)RxDataBuffer, 32);
  enum{
	  printmenumain=0,waitwaveform,printsubmenusawtooth,printsubmenusin,printsubmenusquare,waitsubmenusquare,waitsubmenusin,waitsubmenusawtooth
  };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	Posdata=huart2.RxXferSize - huart2.hdmarx->Instance->NDTR;
	  	if(Posdatapre!=Posdata){
	  			sprintf(TxDataBuffer, "\r\nReceivedChar:[%c]", RxDataBuffer[Posdatapre] );
	  			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),500);
	  	}

	  		if (micros() - timestamp >= 100){
	  			timestamp = micros();
	  			dataOut=dataOut+1;
	  			dataOut=dataOut%4096;

	  			if (hspi3.State == HAL_SPI_STATE_READY && 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)== GPIO_PIN_SET){
	  				MCP4922SetOutput(DACConfig, dataOut);
	  			}
	  		}


	  	switch(state){
			case printmenumain:
				pim("\r\nPlease Select Wave form:\r\n Mode 0:Sawtooth wave\r\n Mode 1:Sin wave\n\r\n Mode 2:Square wave\n ");
				state=waitwaveform;
			break;
			case waitwaveform:
				if(RxDataBuffer[Posdatapre]=='0' && Posdatapre!=Posdata ){
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='1'&& Posdatapre!=Posdata ){
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='2'&& Posdatapre!=Posdata ){
					state=printsubmenusquare;
				}else if( Posdatapre!=Posdata){
					pim("\r\n\r\nPlease Select Wave form!!!!!!!!!!\r\n\r\n ");
					 state=printmenumain;
				}
			break;
			case printsubmenusquare:
				pimstatus(3);
				pim("\r\n Step 0.1!!\r\n a:increase Freq\r\n s:decrease Freq\r\n d:increase Vhigh\r\n f:decrease Vhigh\r\n g:increase Vlow\r\n h:decrease Vlow\r\n j:increase Duty step 5 percent\r\n k:decrease Duty step 5 percent\r\n b:backmainmenu\r\n");
				state=waitsubmenusquare;
			break;
			case waitsubmenusquare:
				if(RxDataBuffer[Posdatapre]=='a' && Posdatapre!=Posdata ){
					if(Freq>=0 && Freq<100){
						Freq=Freq+1;
						pim("\r\nincrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='s'&& Posdatapre!=Posdata ){
					if(Freq>0 && Freq<=100){
						Freq=Freq-1;
						pim("\r\ndecrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='d'&& Posdatapre!=Posdata ){
					if(Vhigh>=0 && Vhigh<33){
						Vhigh=Vhigh+1;
						pim("\r\nincrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='f'&& Posdatapre!=Posdata ){
					if(Vhigh>0 && Vhigh<=33){
						Vhigh=Vhigh-1;
						pim("\r\ndecrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='g'&& Posdatapre!=Posdata ){
					if(Vlow>=0 && Vlow<33){
						Vlow=Vlow+1;
						pim("\r\nincrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='h'&& Posdatapre!=Posdata ){
					if(Vlow>0 && Vlow<=33){
						Vlow=Vlow-1;
						pim("\r\ndecrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='j'&& Posdatapre!=Posdata ){
					if(Dutycycle>=0 && Dutycycle<100){
						Dutycycle=Dutycycle+5;
						pim("\r\nincrease Dutycycle Done\r\n");
					}else if(Dutycycle<=0){
						pim("\r\nIt lower 0 percent\r\n");
					}else if(Dutycycle>=100){
						pim("\r\nIt over 100 percent\r\n");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='k'&& Posdatapre!=Posdata ){
					if(Dutycycle>0 && Dutycycle<=100){
						Dutycycle=Dutycycle-5;
						pim("\r\ndecrease Dutycycle Done\r\n");
					}else if(Dutycycle<=0){
						pim("It lower 0 percent");
					}else if(Dutycycle>=100){
						pim("It over 100 percent");
					}
					state=printsubmenusquare;
				}else if(RxDataBuffer[Posdatapre]=='b'&& Posdatapre!=Posdata ){
					pim("\r\n\r\nBack main menu\r\n");
					state=printmenumain;
				}else if( Posdatapre!=Posdata){
					pim("\r\n\r\nPlease Select !!!!!!!!!!\r\n\r\n ");
					state=printsubmenusquare;
				}
			break;
			case printsubmenusin:
				pimstatus(2);
				pim("\r\n Step 0.1!!\r\n a:increase Freq\r\n s:decrease Freq\r\n d:increase Vhigh\r\n f:decrease Vhigh\r\n g:increase Vlow\r\n h:decrease Vlow\r\n b:backmainmenu\r\n");
				state=waitsubmenusin;
			break;
			case waitsubmenusin:
				if(RxDataBuffer[Posdatapre]=='a' && Posdatapre!=Posdata ){
					if(Freq>=0 && Freq<100){
						Freq=Freq+1;
						pim("\r\nincrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='s'&& Posdatapre!=Posdata ){
					if(Freq>0 && Freq<=100){
						Freq=Freq-1;
						pim("\r\ndecrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='d'&& Posdatapre!=Posdata ){
					if(Vhigh>=0 && Vhigh<33){
						Vhigh=Vhigh+1;
						pim("\r\nincrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='f'&& Posdatapre!=Posdata ){
					if(Vhigh>0 && Vhigh<=33){
						Vhigh=Vhigh-1;
						pim("\r\ndecrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='g'&& Posdatapre!=Posdata ){
					if(Vlow>=0 && Vlow<33){
						Vlow=Vlow+1;
						pim("\r\nincrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='h'&& Posdatapre!=Posdata ){
					if(Vlow>0 && Vlow<=33){
						Vlow=Vlow-1;
						pim("\r\ndecrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusin;
				}else if(RxDataBuffer[Posdatapre]=='b'&& Posdatapre!=Posdata ){
					pim("\r\n\r\nBack main menu\r\n");
					state=printmenumain;
				}else if( Posdatapre!=Posdata){
					pim("\r\n\r\nPlease Select !!!!!!!!!!\r\n\r\n ");
					state=printsubmenusin;
				}
			break;
			case printsubmenusawtooth:
				pimstatus(1);
				pim("\r\n Step 0.1!!\r\n a:increase Freq\r\n s:decrease Freq\r\n d:increase Vhigh\r\n f:decrease Vhigh\r\n g:increase Vlow\r\n h:decrease Vlow\r\n j:Slope up/Down\r\n b:backmainmenu\r\n");
				state=waitsubmenusawtooth;
			break;
			case waitsubmenusawtooth:
				if(RxDataBuffer[Posdatapre]=='a' && Posdatapre!=Posdata ){
					if(Freq>=0 && Freq<100){
						Freq=Freq+1;
						pim("\r\nincrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='s'&& Posdatapre!=Posdata ){
					if(Freq>0 && Freq<=100){
						Freq=Freq-1;
						pim("\r\ndecrease Freq Done\r\n");
					}else if(Freq<=0){
						pim("\r\nIt lower 0 Hz\r\n");
					}else if(Freq>=100){
						pim("\r\nIt over 10 Hz\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='d'&& Posdatapre!=Posdata ){
					if(Vhigh>=0 && Vhigh<33){
						Vhigh=Vhigh+1;
						pim("\r\nincrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='f'&& Posdatapre!=Posdata ){
					if(Vhigh>0 && Vhigh<=33){
						Vhigh=Vhigh-1;
						pim("\r\ndecrease Vhigh Done\r\n");
					}else if(Vhigh<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vhigh>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='g'&& Posdatapre!=Posdata ){
					if(Vlow>=0 && Vlow<33){
						Vlow=Vlow+1;
						pim("\r\nincrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='h'&& Posdatapre!=Posdata ){
					if(Vlow>0 && Vlow<=33){
						Vlow=Vlow-1;
						pim("\r\ndecrease Vlow Done\r\n");
					}else if(Vlow<=0){
						pim("\r\nIt lower 0 V\r\n");
					}else if(Vlow>=33){
						pim("\r\nIt over 3.3 V\r\n");
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='j'&& Posdatapre!=Posdata ){
					if(slope==0){
						pim("\r\nSlope to up\r\n");
						slope=1;
					}else{
						pim("\r\nSlope to Down\r\n");
						slope=0;
					}
					state=printsubmenusawtooth;
				}else if(RxDataBuffer[Posdatapre]=='b'&& Posdatapre!=Posdata ){
					pim("\r\n\r\nBack main menu\r\n");
					state=printmenumain;
				}else if( Posdatapre!=Posdata){
					pim("\r\n\r\nPlease Select !!!!!!!!!!\r\n\r\n ");
					state=printsubmenusawtooth;
				}
			break;

	  	}














	  	Posdatapre=Posdata;
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

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

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim5)
	{
		_micro += 4294968296;
	}
}
uint64_t micros()
{
	return htim5.Instance->CNT + _micro;
}
void pim(char q[]){

	HAL_UART_Transmit(&huart2, (uint8_t*)q, strlen(q),1000);
}

void pimstatus(int x){
	if(x==3){
		sprintf(TxDataBuffer, "\r\nNow F is:[%.2f]\r\nNow Vhigh is:[%.2f]\r\nNow Vlow is:[%.2f]\r\nNow Dutycycle is:[%d]", (float)Freq*0.1,(float)Vhigh*0.1,(float)Vlow*0.1,(int)Dutycycle);
		pim (TxDataBuffer);
	}else if(x==2){
		sprintf(TxDataBuffer, "\r\nNow F is:[%.2f]\r\nNow Vhigh is:[%.2f]\r\nNow Vlow is:[%.2f]\r\n", (float)Freq*0.1,(float)Vhigh*0.1,(float)Vlow*0.1);
		pim (TxDataBuffer);
	}else if(x==1){
		sprintf(TxDataBuffer, "\r\nNow F is:[%.2f]\r\nNow Vhigh is:[%.2f]\r\nNow Vlow is:[%.2f]\r\nNow slope:%d\r\n", (float)Freq*0.1,(float)Vhigh*0.1,(float)Vlow*0.1,(int)slope);
		pim (TxDataBuffer);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
