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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 2
#define ADC1_INJ_CODE 4095
#define VREF 2499
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF75A8))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF75CA))
#define OC_FILTER 10;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp4;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint32_t ADC12_BUFFER[ADC_BUFFER_SIZE] = {0};
uint32_t ADC2_BUFFER[ADC_BUFFER_SIZE] = {0};

uint32_t IAC_AVG=0;
uint32_t VAC_AVG=0;
uint32_t HVDC_AVG=0;

uint32_t IAC_AVG_2=0;

uint16_t ADC_DRIVERS_MID_VOLTAGE = 0X7FE;
uint16_t IAC_OPAMP_MID_VOLTAGE = 0X418; // quarter VREF when PGA x2
uint16_t OVERCURRENT_THS = 0X799;
uint16_t OVERCURRENT_MID = 0X800;

uint16_t VAC_ADC_VAL = 0x4000;
uint16_t IAC_ADC_VAL = 0x4000;
uint16_t HVDC_ADC_VAL = 0x4000;

uint16_t VAC_ADC_VAL_2 = 0;
uint16_t IAC_ADC_VAL_2 = 0;
uint16_t HVDC_ADC_VAL_2 = 0;


uint16_t INJ_IAC_ADC_VAL = 0;

uint16_t INJ_IAC_ADC_VAL_2 = 0;

uint32_t tmp_jdr1;
uint32_t tmp_jdr2;
uint32_t tmp_jdr3;
uint32_t tmp_jdr4;

uint16_t IAC_REF = 0;
uint16_t VIN_MON = 0;
uint16_t VAC_ADC_VAL_BF_REL = 0;
uint16_t SHUNT_NTC_VAL = 0;
uint16_t INT_TEMP_VAL = 0;

uint16_t IAC_ADC_VAL_BUFF[500] = {0};

uint8_t aqusitionDMA1_1 = 0;
uint8_t *ptr_aqusitionDMA1_1;
uint8_t frequency_factor = 1;

uint8_t injected_adc = 0;

uint16_t ii = 0;

uint16_t SPI_TxBuffer[SPI_TX_BUFFER_SIZE] = {0};

uint16_t SPI_RxBuffer[SPI_RX_BUFFER_SIZE] = {0};

uint16_t *ptr_SPI_TxBuffer;

uint16_t *ptr_SPI_RxBuffer;

uint16_t OC_PROT_ON = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP4_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP6_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
uint16_t GetLSB(uint32_t intValue);
uint16_t GetMSB(uint32_t intValue);
void spi_w16( SPI_TypeDef *SPIx, uint16_t dat );
void SPIx_Transfer2(SPI_TypeDef *SPIx, uint16_t *outp, uint16_t *inp, uint16_t count);
void __attribute__( ( optimize( "O0" ) ) )delay_cycles( uint32_t cyc );
static void CORRECT_OPAMP6_Init(void);

uint16_t Status_Val = 0;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP4_Init();
  MX_DAC1_Init();
  MX_DAC3_Init();
  MX_OPAMP2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  CORRECT_OPAMP6_Init();

  //HAL_OPAMP_Start(&hopamp2);
  //HAL_OPAMP_Start(&hopamp6);
  OPAMP2->CSR |=OPAMP_CSR_OPAMPxEN; //OPAMP2 ENABLE   - IAC CURRENT OPAMP
  OPAMP6->CSR |=OPAMP_CSR_OPAMPxEN; //OPAMP6 ENABLE   - NTC VOLTAGE OPAMP

  HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_DIFFERENTIAL_ENDED);

  //HAL_ADC_Start(&hadc2);
  //HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC12_BUFFER, ADC_BUFFER_SIZE);
  //HAL_TIM_Base_Start(&htim1);

  //DAC3 CHANNEL 1 - COMP1 input  (AC GRID OVERCURRENT LOW ) // 30% max val
  DAC1 ->DHR12R1 = (uint32_t)(OVERCURRENT_MID - OVERCURRENT_THS); //DAC3 CHANNEL 2 SET VALUE
  DAC1->CR |= DAC_CR_EN1; //ENABLE DAC1 CHANNEL 1
  while(!(DAC1->SR & DAC_SR_DAC1RDY)); //WAIT UNTIL DAC1 CHANNEL 1 IS READY
  DAC1 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

  //DAC1 CHANNEL 2
  DAC1 ->DHR12R2 = (uint32_t)ADC_DRIVERS_MID_VOLTAGE; //DAC1 CHANNEL 2 SET VALUE //1.25V - OPAMP ADC DRIVERS MID VOLTAGE
  DAC1->CR |= DAC_CR_EN2; //ENABLE DAC1 CHANNEL 2
  while(!(DAC1->SR & DAC_SR_DAC2RDY)); //WAIT UNTIL DAC1 CHANNEL 2 IS READY
  DAC1 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;


  //DAC3 ->DHR12R1 = (uint32_t)(OVERCURRENT_MID - OVERCURRENT_THS); //DAC3 CHANNEL 2 SET VALUE
  DAC3 ->DHR12R1 = (uint32_t)IAC_OPAMP_MID_VOLTAGE; //DAC1 CHANNEL 2 SET VALUE // - IAC MID VOLTAGE
  DAC3->CR |= DAC_CR_EN1; //ENABLE DAC1 CHANNEL 1
  while(!(DAC3->SR & DAC_SR_DAC1RDY)); //WAIT UNTIL DAC3 IS READY
  DAC3 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

  //DAC3 CHANNEL 2 - COMP4 input (AC GRID OVERCURRENT HIGH)
  DAC3 ->DHR12R2 = (uint32_t)(OVERCURRENT_MID + OVERCURRENT_THS); //DAC3 CHANNEL 2 SET VALUE // 70% max val
  DAC3->CR |= DAC_CR_EN2; //ENABLE DAC3 CHANNEL 2
  while(!(DAC3->SR & DAC_SR_DAC2RDY)); //WAIT UNTIL DAC3 CHANNEL 2 IS READY
  DAC3 ->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;

  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC12_BUFFER, ADC_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC12_BUFFER, ADC_BUFFER_SIZE);

  HAL_COMP_Start(&hcomp1);
  HAL_COMP_Start(&hcomp4);

  //HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out form Comp1/Comp4

   //HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out form Comp1/Comp4

   ptr_aqusitionDMA1_1 = &aqusitionDMA1_1;

   ptr_SPI_TxBuffer =&SPI_TxBuffer[0];

   ptr_SPI_RxBuffer =&SPI_RxBuffer[0];

   // Enable the SPI3 peripheral.
   SPI3->CR1 |=  ( SPI_CR1_SPE );

   //HAL_TIM_OnePulse_Start_IT(&htim1, TIM_CHANNEL_2); // Overcurrent Out form Comp1/Comp4
   HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		  if ((*ptr_aqusitionDMA1_1) == 1)
		  {

				if (frequency_factor == 1) // divide transmit PWM Frequency by 2 - So 50 kHz / 2  = 25 kHz up to day
				{
					*ptr_aqusitionDMA1_1 = 0;
				 //IAC_ADC_VAL = (GetMSB(ADC12_BUFFER[1])+GetMSB(ADC12_BUFFER[0]))/2;
				 //VAC_ADC_VAL = GetLSB(ADC12_BUFFER[0]);
				 //HVDC_ADC_VAL = GetLSB(ADC12_BUFFER[1]);

				 //SPI_TxBuffer[0] = IAC_ADC_VAL;
				 //SPI_TxBuffer[1] = VAC_ADC_VAL;
				 //SPI_TxBuffer[2] = HVDC_ADC_VAL;

				 SPI_TxBuffer[0] = (GetMSB(ADC12_BUFFER[1])+GetMSB(ADC12_BUFFER[0]))/2;
				 SPI_TxBuffer[1] = GetLSB(ADC12_BUFFER[0]);
				 SPI_TxBuffer[2] = GetLSB(ADC12_BUFFER[1]);
				 //delay_us(1);
				 //delay_cycles(1);

				 SPIx_Transfer2(SPI3,ptr_SPI_TxBuffer,ptr_SPI_RxBuffer,SPI_TX_BUFFER_SIZE);

				 //HAL_ADCEx_InjectedStart(&hadc2);
				 //HAL_ADCEx_InjectedStart(&hadc1);

				 //IAC_AVG=((uint32_t)((IAC_ADC_VAL + (GetMSB(ADC12_BUFFER[1])+GetMSB(ADC12_BUFFER[0]))/2)/2));
				 //VAC_AVG=((uint32_t)((VAC_ADC_VAL + GetLSB(ADC12_BUFFER[0]))/2));
				 //HVDC_AVG=((uint32_t)((HVDC_ADC_VAL + GetLSB(ADC12_BUFFER[1]))/2));




				 //IAC_ADC_VAL = (uint16_t)IAC_AVG;
				 //VAC_ADC_VAL = (uint16_t)VAC_AVG;
				 //HVDC_ADC_VAL = (uint16_t)HVDC_AVG;

				 //HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);
				 //HAL_ADCEx_InjectedPollForConversion(&hadc2, 1);

				 //tmp_jdr1 = ADC2 -> JDR1;
				 //tmp_jdr2 = ADC2 -> JDR2;
				 //tmp_jdr3 = ADC2 -> JDR3;
				 //tmp_jdr4 = ADC2 -> JDR4;

				 //INJ_IAC_ADC_VAL = (uint16_t)(tmp_jdr1 + tmp_jdr2 + tmp_jdr3 + tmp_jdr4);

				 //frequency_factor++;
				 frequency_factor = 1;

				 Status_Val = SPI_RxBuffer[0];

				 HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, RESET);
				}

				else if (frequency_factor == 2)
				{
					//delay_cycles(60);
					//HAL_ADCEx_InjectedStart(&hadc2);
				    //HAL_ADCEx_InjectedStart(&hadc1);

					//IAC_ADC_VAL_2 = (GetMSB(ADC12_BUFFER[1])+GetMSB(ADC12_BUFFER[0]))/2;
					//VAC_ADC_VAL_2 = GetLSB(ADC12_BUFFER[0]);
					//HVDC_ADC_VAL_2 = GetLSB(ADC12_BUFFER[1]);

					//VAC_AVG=((uint32_t)((VAC_ADC_VAL + VAC_ADC_VAL_2)/2));
					//HVDC_AVG=((uint32_t)((HVDC_ADC_VAL + HVDC_ADC_VAL_2)/2));

					//VAC_ADC_VAL = (uint16_t)VAC_AVG;
					//HVDC_ADC_VAL = (uint16_t)HVDC_AVG;

					//HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);
					//HAL_ADCEx_InjectedPollForConversion(&hadc2, 1);


					//tmp_jdr1 = ADC2 -> JDR1;
					//tmp_jdr2 = ADC2 -> JDR2;
					//tmp_jdr3 = ADC2 -> JDR3;
					//tmp_jdr4 = ADC2 -> JDR4;

					//INJ_IAC_ADC_VAL_2 = (uint16_t)(tmp_jdr1 + tmp_jdr2 + tmp_jdr3 + tmp_jdr4);

					//IAC_AVG=((uint32_t)(IAC_ADC_VAL + IAC_ADC_VAL_2) / 2);
					//IAC_ADC_VAL = (uint16_t)IAC_AVG;



					frequency_factor = 1;


					  if(Status_Val == 0x15) // GRID INSERTION
					  {
						  if(OC_PROT_ON < 2000)
							  {
								  OC_PROT_ON++;
							  }
						  if(OC_PROT_ON == 2000)
							  {



								  OC_PROT_ON = 2001;
							  }
						  /* Enable the TIM Capture/Compare 2 interrupt */
						  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
						  /* Set or reset the CCxE Bit */
						  TIM1->CCER |= (uint32_t)(TIM_CCx_ENABLE << (TIM_CHANNEL_2 & 0x1FU)); /* 0x1FU = 31 bits max shift */
						  /* Enable the main output */
						  TIM1->BDTR |= (TIM_BDTR_MOE);

						  //HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, SET);
					  }
					  HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, RESET);
				}

				//*ptr_aqusitionDMA1_1 = 0;

		}


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_EXT_IT11;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGINTERL_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_INVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**OPAMP2 GPIO Configuration
  PA6   ------> OPAMP2_VOUT
  PB0   ------> OPAMP2_VINP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO2;
  OPAMP_InitStruct.InputInverting = LL_OPAMP_INPUT_INVERT_CONNECT_NO;
  LL_OPAMP_Init(OPAMP2, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP2, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
  LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_2_OR_MINUS_1);
  LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP6_Init(void)
{

  /* USER CODE BEGIN OPAMP6_Init 0 */

  /* USER CODE END OPAMP6_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**OPAMP6 GPIO Configuration
  PB11   ------> OPAMP6_VOUT
  PB13   ------> OPAMP6_VINP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP6_Init 1 */

  /* USER CODE END OPAMP6_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_FOLLOWER;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO2;
  LL_OPAMP_Init(OPAMP6, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP6, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP6, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
  LL_OPAMP_SetTrimmingMode(OPAMP6, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP6_Init 2 */
  //OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_DAC; !!!!!!!!!!!!!
  /* USER CODE END OPAMP6_Init 2 */

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
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 5;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS1_GPIO_Port, STATUS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS1_Pin */
  GPIO_InitStruct.Pin = STATUS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(Status_Val != 0x15) // NO GRID INSERTION
  {

  }
  else if (Status_Val == 0x15) // GRID INSERTION
  {

	  }
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIMEx_BreakCallback could be implemented in the user file
   */
}

uint16_t GetLSB(uint32_t intValue)
{
	return (intValue & 0x0000FFFF);
}

uint16_t GetMSB(uint32_t intValue)
{
    return ((intValue & 0xFFFF0000)) >> 16;
}

void spi_w16( SPI_TypeDef *SPIx, uint16_t dat ) {
  // Wait for TXE 'transmit buffer empty' bit to be set.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  // Send the bytes.
  *( uint16_t* )&( SPIx->DR ) = dat;
}

void SPIx_Transfer2(SPI_TypeDef *SPIx, uint16_t *outp, uint16_t *inp, uint16_t count) {
    while(count--) {
        while(!(SPIx->SR & SPI_SR_TXE))
            ;
        *(volatile uint16_t *)&SPIx->DR = *outp++;
        while(!(SPIx->SR & SPI_SR_RXNE))
            ;
        *inp++ = *(volatile uint16_t *)&SPIx->DR;
    }
}

static void CORRECT_OPAMP6_Init(void)
{

  /* USER CODE BEGIN OPAMP6_Init 0 */

  /* USER CODE END OPAMP6_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**OPAMP6 GPIO Configuration
  PB11   ------> OPAMP6_VOUT
  PB13   ------> OPAMP6_VINP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP6_Init 1 */

  /* USER CODE END OPAMP6_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_FOLLOWER;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_DAC;
  LL_OPAMP_Init(OPAMP6, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP6, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP6, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
  LL_OPAMP_SetTrimmingMode(OPAMP6, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP6_Init 2 */
  //OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_DAC; !!!!!!!!!!!!!
  /* USER CODE END OPAMP6_Init 2 */

}

//Microsecond delay
void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}

void __attribute__( ( optimize( "O0" ) ) )
delay_cycles( uint32_t cyc ) {
  for ( uint32_t d_i = 0; d_i < cyc; ++d_i ) { asm( "NOP" ); }
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
