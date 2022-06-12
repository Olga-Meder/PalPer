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
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_qspi.h"
#include "../../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_audio.h"
#include "string.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUFFER_SIZE        	1
#define WRITE_READ_ADDR     ((uint32_t)0x00100000)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)

uint8_t qspi_aTxBuffer[BUFFER_SIZE];
uint8_t qspi_aRxBuffer[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//1.open -> sektor 16
//2.trade -> sektor 17
//3.cymbal -> sektor 18
//4.kick2 -> sektor 19
//5.wood -> sektor 20
//6.snare -> sektor 21

//7.clap -> sektor 22
//8.tom -> sektor 23
//9.hihat -> sektor 24
//10.kick -> sektor 25
//11.hihatModular -> sektor 26
//12.kick2 -> sektor 27

//13.cowbell -> sektor 28
//14.egg -> sektor 29
//15.cabasa -> sektor 30
//16.congaLov -> sektor 31
//17.clave -> sektor 32
//18.congaHigh -> sektor 33

void AudioPlay_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  Error_Handler();
}

/* Size (in bytes) of each audio file */
#define AUDIO_FILE_SIZE      (uint32_t)(44144)

/* Address of the first audio in FLASH memory*/
#define AUDIO_START_ADDRESS  WRITE_READ_ADDR

#define AUDIODATA_SIZE                      2

/* Remainig number of audio samples to play */
static int32_t RemainingAudioSamplesNb;

/* Address (in Flash memory) of the first audio sample to play */
static uint16_t *pAudioSample;

static uint32_t RozmiarSekcji = 65536;

void AudioPlay_TransferComplete_CallBack()
{
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

LCD_HandleTypeDef hlcd;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint32_t PomiarADC;
volatile uint8_t adc;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	PomiarADC = HAL_ADC_GetValue(&hadc1); // Pobranie zmierzonej wartosci
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LCD_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 50);
	return len;
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SAI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_GLASS_Init();
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  BSP_LCD_GLASS_ScrollSentence((uint8_t *)"      WITAMY W PALPER", 1, SCROLL_SPEED_HIGH);

  static QSPI_Info pQSPI_Info;
  uint8_t status;
  status = BSP_QSPI_Init();
  if (status == QSPI_OK) {
	  pQSPI_Info.FlashSize          = (uint32_t)0x00;
	  pQSPI_Info.EraseSectorSize    = (uint32_t)0x00;
	  pQSPI_Info.EraseSectorsNumber = (uint32_t)0x00;
	  pQSPI_Info.ProgPageSize       = (uint32_t)0x00;
	  pQSPI_Info.ProgPagesNumber    = (uint32_t)0x00;
  }

  if(BSP_AUDIO_OUT_Init(2,  // Słuchawki
                        80, // %głośności
                        44100) != 0)  // częstotliwość
  {
		  Error_Handler();
  }

  BSP_AUDIO_OUT_RegisterCallbacks(AudioPlay_Error_CallBack,
              	  	  	  	  	  NULL,
                                  AudioPlay_TransferComplete_CallBack);

  if(BSP_AUDIO_OUT_SetVolume(80) != 0)
  {
	  Error_Handler();
  }

  int flag = 1;
  int flag2 = 1;
  uint32_t count = 0;
  HAL_ADC_Start(&hadc1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //przyciski
/*
	  if(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port,JOY_CENTER_Pin) && flag){
		  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * count));
		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_Delay(500);
		  BSP_AUDIO_OUT_Stop(2);
		  count++;
		  flag = 0;
		  count = count % 18;
	  }
	  if(!(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port,JOY_CENTER_Pin))){
		  flag = 1;
	  }
*/
	  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
		  PomiarADC = HAL_ADC_GetValue(&hadc1);
//		  char buffor[8];
//		  sprintf(buffor,"%i",PomiarADC);
//		  BSP_LCD_GLASS_Clear();
//		  BSP_LCD_GLASS_DisplayString(buffor);
//		  HAL_Delay(200);

//		  HAL_ADC_Start(&hadc1);

	  //////////////////////////////////////////////////////////////////////////////////////

	  if(PomiarADC == 7 || PomiarADC == 15 || PomiarADC == 31)
	  {
		  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+17)));  //congahigh
		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_Delay(150);
		  BSP_AUDIO_OUT_Stop(2);
	  }
	  else if(PomiarADC == 63)
	  {
		  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+12)));
		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_Delay(150);
		  BSP_AUDIO_OUT_Stop(2);
	  }
	  else if(PomiarADC == 192)
	  {
		  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+15)));  //congalow
		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_Delay(150);
		  BSP_AUDIO_OUT_Stop(2);
	  }
	  else if(PomiarADC == 224 || PomiarADC == 240)
	  {
		  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+4)));
		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_Delay(150);
		  BSP_AUDIO_OUT_Stop(2);
	  }
	  HAL_ADC_Start(&hadc1);


	  }
	  //////////////////////////////////////////////////////////////////////////////////////
		  /*
	  }

		  if(PomiarADC >  5 && PomiarADC <= 85)
		  {
			  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
			  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+17)));  //congahigh
			  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
			  HAL_Delay(150);
			  BSP_AUDIO_OUT_Stop(2);

		  }
		  if(PomiarADC > 85 && PomiarADC <= 170)
		  {
		  	  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  	  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+15))); //congalow
		  	  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  	  HAL_Delay(150);
		  	  BSP_AUDIO_OUT_Stop(2);

		  }
		  else if(PomiarADC > 170)
		  {
		  	  RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
		  	  pAudioSample = (uint16_t *) (WRITE_READ_ADDR+(RozmiarSekcji * (count+4))); //hihat
		  	  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  	  HAL_Delay(150);
		  	  BSP_AUDIO_OUT_Stop(2);

		  }
//		  BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
		  HAL_ADC_Start(&hadc1);
	  }
	  */
//      if(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port,JOY_CENTER_Pin) && flag2){
//          count++;
//          flag2 = 0;
//          count = count % 9;
//      }
//      if(!(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port,JOY_CENTER_Pin)))
//          flag2 = 1;
//
//
//      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
//          PomiarADC = HAL_ADC_GetValue(&hadc1);
//
//          if(PomiarADC < 5) {
//              flag = 1;
//          }
//          else if(PomiarADC > 5 && PomiarADC < 80 ) {
//              RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
//              pAudioSample = (uint16_t ) (WRITE_READ_ADDR+(RozmiarSekcji*((2*count)+1)));
//              BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
//              HAL_Delay(300);
//              BSP_AUDIO_OUT_Stop(2);
//              flag = 1;
//
//          }
//          else if(PomiarADC > 80) {
//              RemainingAudioSamplesNb = (uint32_t)(AUDIO_FILE_SIZE / 2);
//              pAudioSample = (uint16_t) (WRITE_READ_ADDR+(RozmiarSekcji*(2*count)));
//              BSP_AUDIO_OUT_Play(pAudioSample, RemainingAudioSamplesNb);
//              HAL_Delay(300);
//              BSP_AUDIO_OUT_Stop(2);
//              flag = 1;
//          }
//
//          HAL_ADC_Start(&hadc1);
//      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */
  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */
  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_31;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_3;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_2;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_4;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV32;
  hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */
  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */
  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */
  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */
  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */
  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */
  /* USER CODE END SAI1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIGITIZER_OUT_X1_GPIO_Port, DIGITIZER_OUT_X1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIGITIZER_IN_X2_GPIO_Port, DIGITIZER_IN_X2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AUDIO_RST_Pin */
  GPIO_InitStruct.Pin = AUDIO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_CENTER_Pin DIGITIZER_IN_Y2_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin|DIGITIZER_IN_Y2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIGITIZER_OUT_X1_Pin */
  GPIO_InitStruct.Pin = DIGITIZER_OUT_X1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIGITIZER_OUT_X1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIGITIZER_IN_X2_Pin */
  GPIO_InitStruct.Pin = DIGITIZER_IN_X2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIGITIZER_IN_X2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_R_Pin */
  GPIO_InitStruct.Pin = LD_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_G_Pin */
  GPIO_InitStruct.Pin = LD_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

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
