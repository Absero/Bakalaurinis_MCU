/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define KANALU_SKAICIUS 10
#define DAC_SIGNAL_STEPS 100
#define CHANGE_SINE_F(f) TIM7->ARR = ( 1.0 / f * HAL_RCC_GetHCLKFreq() / (TIM7->PSC+1) / DAC_SIGNAL_STEPS) - 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//volatile int16_t mAdcData[KANALU_SKAICIUS]={0};
struct {
	uint8_t adcDone :1;
	uint8_t usbDRDY :1;
	uint8_t start :1;
	uint8_t rem :5;
} mVeliavos;

struct {
	uint8_t *data;
	uint32_t len;
} mUSB_data;

volatile struct {
	int16_t adcData[KANALU_SKAICIUS];
	int16_t temperatura;
} mPaketasSiuntimui;

int16_t TEMP;
float Temperature = 0;
uint8_t Presence = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void generateSine(uint16_t *array, uint32_t len, uint16_t amplitude);
void ledsToggle();
void delay(uint16_t time);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
void measTemp();

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	mVeliavos.adcDone = 1;
}

void CDC_ReceiveCallback(uint8_t *buf, uint32_t len) {
	mUSB_data.data = buf;
	mUSB_data.len = len;
	mVeliavos.usbDRDY = 1;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint16_t dacSignal[DAC_SIGNAL_STEPS] = { 0 };
	generateSine(dacSignal, DAC_SIGNAL_STEPS, 2000);

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
	MX_ADC2_Init();
	MX_USB_DEVICE_Init();
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_DAC_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);

	CHANGE_SINE_F(50);

	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) dacSignal, DAC_SIGNAL_STEPS, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim7);	// Timer for sine wave

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) mPaketasSiuntimui.adcData, KANALU_SKAICIUS);
	HAL_TIM_Base_Start(&htim3); // Timer for ADC

	HAL_TIM_Base_Start(&htim6); // Timer for OneWire protocol delays

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (mVeliavos.usbDRDY) {
			switch (*mUSB_data.data) {
			case 0:
			case 1:
				mVeliavos.start = *mUSB_data.data;
				break;
			case 2:
				// Change ADC timer period
				if (mUSB_data.len == 3)
					TIM3->ARR = *(uint16_t*) (mUSB_data.data + 1) - 1 < 299 ? 299 : *(uint16_t*) (mUSB_data.data + 1) - 1;
				break;
			case 3:
				// Change sine wave frequency
				CHANGE_SINE_F(*(uint16_t* ) (mUSB_data.data + 1));
				break;
			case 4:
				// Change sine wave amplitude
				generateSine(dacSignal, DAC_SIGNAL_STEPS, *(uint16_t*) (mUSB_data.data + 1));
			default:
				break;
			}
			mVeliavos.usbDRDY = 0;
		}

		if (mVeliavos.adcDone) {
			mVeliavos.adcDone = 0;
			ledsToggle();
			measTemp();
			mPaketasSiuntimui.temperatura = TEMP;

			if (mVeliavos.start)
				CDC_Transmit_FS((uint8_t*) mPaketasSiuntimui.adcData, KANALU_SKAICIUS * 2 + 2);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void generateSine(uint16_t *array, uint32_t len, uint16_t amplitude) {
	amplitude /= 2;
	//	Generate sine wave for DAC
	for (uint8_t i = 0; i < len; i++) {
		*(uint16_t*) (array + i) = amplitude + amplitude * sin(2 * M_PI * i / len);
	}
}

void ledsToggle() {
	HAL_GPIO_TogglePin(led0_GPIO_Port, led0_Pin);
	HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
}

void delay(uint16_t time) {
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6)) < time)
		;
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_Start(void) {
	uint8_t Response = 0;
	Set_Pin_Output(temp_GPIO_Port, temp_Pin);   // set the pin as output
	HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, 0);  // pull the pin low
	delay(480);   // delay according to datasheet

	Set_Pin_Input(temp_GPIO_Port, temp_Pin);    // set the pin as input
	delay(80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin(temp_GPIO_Port, temp_Pin)))
		Response = 1;    // if the pin is low i.e the presence pulse is detected
	else
		Response = -1;

	delay(400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write(uint8_t data) {
	Set_Pin_Output(temp_GPIO_Port, temp_Pin);  // set as output
//	delay(1);

	for (int i = 0; i < 8; i++) {

		if ((data & (1 << i)) != 0)  // if the bit is high
				{
			// write 1

			Set_Pin_Output(temp_GPIO_Port, temp_Pin);  // set as output
			HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, 0);  // pull the pin LOW
			delay(1);  // wait for 1 us

			Set_Pin_Input(temp_GPIO_Port, temp_Pin);  // set as input
//			HAL_GPIO_WritePin (temp_GPIO_Port, temp_Pin, 1);
			delay(60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(temp_GPIO_Port, temp_Pin);
			HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, 0);  // pull the pin LOW
			delay(60);  // wait for 60 us

			Set_Pin_Input(temp_GPIO_Port, temp_Pin);
		}
	}
}

uint8_t DS18B20_Read(void) {
	uint8_t value = 0;
	uint8_t nuskaityta = 0;

	Set_Pin_Input(temp_GPIO_Port, temp_Pin);

	for (int i = 0; i < 8; i++) {
		Set_Pin_Output(temp_GPIO_Port, temp_Pin);   // set as output
		HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, 0);  // pull the data pin LOW
		delay(2);  // wait for > 1us

		Set_Pin_Input(temp_GPIO_Port, temp_Pin);  // set as input
		delay(8);

		nuskaityta = HAL_GPIO_ReadPin(temp_GPIO_Port, temp_Pin);

		value |= nuskaityta << i;  // read = 1

		delay(60);  // wait for 60 us
	}
	return value;
}

void measTemp() {

	uint8_t Temp_byte1, Temp_byte2;

	Presence = DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0x44);  // convert t

	do {
		HAL_Delay(1);
		Set_Pin_Output(temp_GPIO_Port, temp_Pin);   // set as output

		HAL_GPIO_WritePin(temp_GPIO_Port, temp_Pin, 0);  // pull the data pin LOW
		delay(2);  // wait for > 1us

		Set_Pin_Input(temp_GPIO_Port, temp_Pin);  // set as input
		delay(10);
	} while (!HAL_GPIO_ReadPin(temp_GPIO_Port, temp_Pin));

	Presence = DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0xBE);  // Read Scratch-pad
	HAL_Delay(1);

	Temp_byte1 = DS18B20_Read();
	Temp_byte2 = DS18B20_Read();
	TEMP = (Temp_byte2 << 8) | Temp_byte1;
	Temperature = (float) TEMP / 16;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
