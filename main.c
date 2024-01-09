/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
int32_t tpr = 0; // время открытой двери
int8_t Tzd = 0; // температура заданная
int8_t Tdt = 0; // температура с датчика
int8_t Tpr = 0; // температура за 10 секунд до проверки
float adc_Tdt = 0; // значение температуры с датчика на момент считывания
float adc_Tzd = 0; // значение заданной температуры на момент считывания
float adc_Tdt1 = 0; // значение температуры с датчика на момент считывания
float adc_Tzd1= 0; // значение заданной температуры на момент считывания
float adc_Tdt_rez = 0; // значение температуры с датчика после преобразования
float adc_Tzd_rez = 0; // значение заданной температуры после преобразования
float middle = 0; // выходное значение с фильтра для заданной температуры
float middle1 = 0; // выходное значение с фильтра для температуры с датчика
int32_t t = 0; // время работы установки
int8_t Kompr = 0; // состояние работы компрессора (0 - не работает, 1 - работает)
int8_t error_state = 0; // состояние ошибки (0 - ошибка отсутствует, 1 - ошибка присутствует)
int8_t door_state = 0; // состояние двери (0 - дверь открыта, 1 - дверь закрыта)
int8_t power_state = 0; // состояние установки (0 - установка выключена, 1 - установка включена)
int8_t prov = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float  med_Tzd(float f) // медианный фильтр для значений с потенциометра для заданной температуры
{
	  static float buffer[3] = {0,0,0};
	  if (buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0) {
	    for (int i = 0; i < 3; i++) {
	      buffer[i] = f;
	    }
	  }

	  buffer[0] = buffer[1];
	  buffer[1] = buffer[2];
	  buffer[2] = f;

	  float a = buffer[0];
	  float b = buffer[1];
	  float c = buffer[2];
	  if ((a <= b) && (a <= c))
	  {
		  middle = (b <= c) ? b : c;
      }
	  else
	  {
	    if ((b <= a) && (b <= c))
	    {
	    	middle = (a <= c) ? a : c;
	    }
	    else
	    {
	    	middle = (a <= b) ? a : b;
	    }
	  }

	  return middle;
}

float med_Tdt(float f1) // медианный фильтр для значений с потенциометра для температуры с датчика
{
	  static float buffer1[3] = {0,0,0};
	  if (buffer1[0] == 0 && buffer1[1] == 0 && buffer1[2] == 0) {
	    for (int i1 = 0; i1 < 3; i1++) {
	      buffer1[i1] = f1;
	    }
	  }

	  buffer1[0] = buffer1[1];
	  buffer1[1] = buffer1[2];
	  buffer1[2] = f1;

	  float a1 = buffer1[0];
	  float b1 = buffer1[1];
	  float c1 = buffer1[2];
	  if ((a1 <= b1) && (a1 <= c1))
	  {
		  middle1 = (b1 <= c1) ? b1 : c1;
      }
	  else
	  {
	    if ((b1 <= a1) && (b1 <= c1))
	    {
	    	middle1 = (a1 <= c1) ? a1 : c1;
	    }
	    else
	    {
	    	middle1 = (a1 <= b1) ? a1 : b1;
	    }
	  }

	  return middle1;
}

void doorTest(int tpr) { // Проверка состояния открытой двери
	if (tpr > 15) {
	  	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	  	Kompr = 0;
	  	HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	}
}

void tempTest(int8_t Tdt, int8_t Tpr) { // Проверка работоспособности датчика температуры
	if (Tdt < -7 || Tdt > 45) {
		error_state = 1;
	}
	if (abs(Tdt - Tpr) > 20 && t % 10 == 0 && t > 10) {
		error_state = 1;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);

  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //power_state = HAL_GPIO_ReadPin(Power_GPIO_Port, Power_Pin);
	  //door_state = HAL_GPIO_ReadPin(Door_GPIO_Port, Door_Pin);

	  if (power_state)
	  {
		  if (error_state)
		  {
			  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_RESET);
		  }
		  else if (tpr >= 3)
		  {
			  doorTest(tpr);
		  }
		  else if (tpr < 3)
		  {
			  HAL_ADC_Start_IT(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, 1);
			  adc_Tdt = HAL_ADC_GetValue(&hadc1);
			  adc_Tdt1 = adc_Tdt / 4095 * 60 - 10;
			  adc_Tdt_rez = med_Tdt(adc_Tdt1);
			  Tdt = (int8_t) adc_Tdt_rez;
			  HAL_ADC_Stop_IT(&hadc1);

			  HAL_ADC_Start_IT(&hadc2);
			  HAL_ADC_PollForConversion(&hadc2, 1);
			  adc_Tzd = HAL_ADC_GetValue(&hadc2);
			  adc_Tzd1 = adc_Tzd / 4095 * 60 - 10;
			  adc_Tzd_rez = med_Tzd(adc_Tzd1);
			  Tzd = (int8_t) adc_Tzd_rez;
			  HAL_ADC_Stop_IT(&hadc2);
			  //prov = abs(Tdt - Tzd);
			  if (abs(Tdt - Tzd) <= 2)
			  {
				  Kompr = 0;
				  HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
				  tempTest(Tdt, Tpr);
			  }
			  else if (abs(Tdt - Tzd) > 2)
			  {
				  Kompr = 1;
				  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
				  if (Tdt - Tzd < 0)
				  {
					  HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_RESET);
				  }
				  else
				  {
					  HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_SET);
				  }
				  tempTest(Tdt, Tpr);
			  }
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Kompr_GPIO_Port, Kompr_Pin, GPIO_PIN_RESET);
	  }



	  //if (adc_reg_value > 3200)
	  //{
	   //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	   //pos = 2;
	  //}
	  //else
	  //{
	  //	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	  //	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  //	pos = 1;
	  //}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
