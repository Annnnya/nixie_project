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
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//clock variables and functions



RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;


GPIO_TypeDef* HOUR1 = GPIOE;
uint16_t hour_pins1[] = {GPIO_PIN_7,GPIO_PIN_8 , GPIO_PIN_9, GPIO_PIN_10};

GPIO_TypeDef* HOUR2 = GPIOE;
uint16_t hour_pins2[] = {GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

GPIO_TypeDef* MIN1 = GPIOA;
uint16_t min_pins1[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

GPIO_TypeDef* MIN2 = GPIOD;
uint16_t min_pins2[] = {GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};



void write_lamp(GPIO_TypeDef* H1_port, uint16_t* pins, int n){
  int zeros[4]={0};
  for(int i=0;n>0;i++)
      {
      zeros[i]=n%2;
      n=n/2;
     }
  for (int i=0; i<4; ++i){
  HAL_GPIO_WritePin(H1_port, pins[i], zeros[i]);}
}

void set_time(uint8_t hrs, uint8_t mins){
  sTime.Hours = hrs;
  sTime.Minutes = mins;
  sTime.Seconds = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

void get_and_write_time(){
//  RTC_TimeTypeDef sTime;
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  uint8_t hours = sTime.Minutes;
  uint8_t mins = sTime.Seconds;
  write_lamp(HOUR1, hour_pins1, hours/10);
  write_lamp(HOUR2, hour_pins2, hours%10);
  write_lamp(MIN1, min_pins1, mins/10);
  write_lamp(MIN2, min_pins2, mins%10);
}

void reset_lamps(){
  write_lamp(HOUR1, hour_pins1, 0);
  write_lamp(HOUR2, hour_pins2, 0);
  write_lamp(MIN1, min_pins1, 0);
  write_lamp(MIN2, min_pins2, 0);
}

void standart_test(){
	for (int i =0; i<10; ++i){
		write_lamp(HOUR1, hour_pins1, i);
	  write_lamp(HOUR2, hour_pins2, i);
	  write_lamp(MIN1, min_pins1, i);
	  write_lamp(MIN2, min_pins2, i);
	  HAL_Delay(1000);
	  }
	write_lamp(HOUR1, hour_pins1, 1);
		  write_lamp(HOUR2, hour_pins2, 2);
		  write_lamp(MIN1, min_pins1, 3);
		  write_lamp(MIN2, min_pins2, 4);
		  HAL_Delay(2000);
}

//global pin variables

GPIO_TypeDef* port = GPIOC;
int CLK_PIN = CLK_Pin;
int DIO_PIN = DIO_Pin;
int STB_PIN = STB_Pin;
GPIO_TypeDef* DHT22_PORT = GPIOA;
int DHT22_PIN = DHT22_Pin;



//multi-use helper functions

GPIO_InitTypeDef GPIO_InitStruct = {0};

void chmode_out(GPIO_TypeDef* GPIO_PORT, int PIN){
  GPIO_InitStruct.Pin = PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
}

void chmode_in(GPIO_TypeDef* GPIO_PORT, int PIN){
  GPIO_InitStruct.Pin = PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);
}

void delay_us (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}



//functions for TM1638


const uint8_t SegmCodes[18] =
{
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F, // 9
  0x77, // A
  0x7c, // b
  0x39, // C
  0x5E, // d
  0x79, // E
  0x71,  // F
  //!!!! not hex
  0x38,// L - 10
  0x76 // H - 11
};

void shift_out(GPIO_TypeDef* port, int CLK_PIN,
    int DIO_PIN, bool dir, uint8_t command){
    for (int i = 0; i < 8; i++)
    {
        bool output = false;
        if (dir)
        {
            output = command & 0b10000000;
            command = command << 1;
        }
        else
        {
            output = command & 0b00000001;
            command = command >> 1;
        }
        HAL_GPIO_WritePin(port, DIO_PIN, output);
        HAL_GPIO_WritePin(port, CLK_PIN, 1);
        delay_us(10);
        HAL_GPIO_WritePin(port, CLK_PIN, 0);
        delay_us(10);
    }
  }


  void send_command(uint8_t bt){
    HAL_GPIO_WritePin(port, STB_PIN, 0);
    shift_out(port, CLK_PIN,  DIO_PIN, false, bt);
    HAL_GPIO_WritePin(port, STB_PIN, 1);
  }
  void send_args(uint8_t bt){
    shift_out(port, CLK_PIN,  DIO_PIN, false, bt);
  }

  void reset_TM(){
    send_command(0x40);
    HAL_GPIO_WritePin(port, STB_PIN, 0);
    send_args(0xc0);
    for (uint8_t i = 0; i < 16; i++){
         send_args(0x00);
       }
    HAL_GPIO_WritePin(port, STB_PIN, 1);
  }

  void print_num(int n){
      send_command(0x40);
      HAL_GPIO_WritePin(GPIOD, STB_Pin, 0);
      send_args(0xc0);
      for (int i =0; i<8; ++i){
      send_args(SegmCodes[n%10]);
      n=n/10;
      send_args(0x00);}
      HAL_GPIO_WritePin(GPIOD, STB_Pin, 1);
    }

  void print_temp(int temp){
    send_command(0x40);
    HAL_GPIO_WritePin(GPIOD, STB_Pin, 0);
    send_args(0xc0);
    send_args(SegmCodes[temp/10]);
    send_args(0x00);
    send_args(SegmCodes[temp%10]);
    send_args(0x00);
    send_args(99);
    send_args(0x00);
    send_args(SegmCodes[0x0C]);
    HAL_GPIO_WritePin(GPIOD, STB_Pin, 1);
  }

  void print_hum(int hum){
      send_command(0x40);
      HAL_GPIO_WritePin(GPIOD, STB_Pin, 0);
      send_args(0xca);
      send_args(SegmCodes[hum/10]);
      send_args(0x00);
      send_args(SegmCodes[hum%10]);
      send_args(0x00);
      send_args(SegmCodes[0x11]);
      HAL_GPIO_WritePin(GPIOD, STB_Pin, 1);
    }




//functions for DHT22


  void DHT22_Start (void)
  {
//  DHT22_PORT->MODER |= GPIO_MODER_MODER3_0;
  chmode_out(DHT22_PORT, DHT22_PIN);
    HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);
    delay_us (1200);
    HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);
    delay_us (20);
//    DHT22_PORT->MODER &= ~(GPIO_MODER_MODER3);
    chmode_in(DHT22_PORT, DHT22_PIN);
  }

  uint8_t DHT22_Check_Response (void)
  {
//    DHT22_PORT->MODER &= ~(GPIO_MODER_MODER3);
  chmode_in(DHT22_PORT, DHT22_PIN);
    uint8_t Response = 0;
    delay_us (40);
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
    {
      delay_us (80);
      if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
      else Response = -1;
    }

    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));
    return Response;
  }

  uint8_t DHT22_Read (void)
  {
    uint8_t i,j;
    for (j=0;j<8;j++)
    {
      while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));
      delay_us (40);
      if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
      {
        i&= ~(1<<(7-j));
      }
      else i|= (1<<(7-j));
      while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));
    }

    return i;
  }

  void TH_read_write(){
      DHT22_Start();
        uint8_t response = DHT22_Check_Response();
        if (response){
          float temp_Celsius = 0;
          float Humidity = 0;
          uint8_t hum1 = DHT22_Read();
          uint8_t hum2 = DHT22_Read();
          uint8_t tempC1 = DHT22_Read();
          uint8_t tempC2 = DHT22_Read();
          uint8_t SUM = DHT22_Read();
          uint8_t CHECK = hum1 + hum2 + tempC1 + tempC2;
          reset_TM();
          if (CHECK == SUM){
           if (tempC1>127){
             temp_Celsius = (float)tempC2/10*(-1);
           }
           else{
             temp_Celsius = (float)((tempC1<<8)|tempC2)/10;
           }
           Humidity = (float) ((hum1<<8)|hum2)/10;
           print_temp((int)temp_Celsius);
           print_hum((int)Humidity);
          }
          print_temp((int)temp_Celsius);
          print_hum((int)Humidity);
        }
        else{
          print_temp(0);
          print_hum(0);}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  standart_test();
//  reset();

//  HAL_TIM_Base_Start_IT(&htim3);

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    RTC_TimeTypeDef sTime;
    sTime.Hours = 1;
    sTime.Minutes = 23;
    sTime.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    get_and_write_time();
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start(&htim1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    send_command(0x8a);
    reset_TM();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
//{
////	static RTC_TimeTypeDef sTime;
////		HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
////		get_and_write_time();
//	reset_lamps();
//	print_num(__HAL_TIM_GET_COUNTER(&htim3));
//	HAL_Delay(1000);
//	print_temp(25);
//
////	print_num(__HAL_TIM_GET_COUNTER(&htim3));
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {
	  get_and_write_time();
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
