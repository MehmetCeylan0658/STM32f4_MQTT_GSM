/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "MQTTPacket.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void mqtt_init(void);
void mqtt(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char SMSCTRLZ   = 0x1A; // use to terminate tcp data incase of buffer mode
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	//mqtt_init(); 
	mqtt();
	//HAL_UART_Transmit(&huart1,(uint8_t *)"AT\r\n",4,50);
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_14);
		HAL_Delay(200);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
This function is responsible for opening a tcp socket in tranparent mode
*/
void mqtt_init(void)
{
char mx[60];	
sprintf(mx,"AT\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+CPIN?\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+CREG?\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIFGCNT=0\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QICSGP=1,\"CMNET\"\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIMUX=0\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIMODE=1\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QITCFG=3,2,512,1\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIDNSIP=1\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIREGAPP\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIACT\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QILOCIP\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(1000);
sprintf(mx,"AT+QIOPEN=\"TCP\",\"iot.eclipse.org\",\"1883\"\r\n");	
HAL_UART_Transmit(&huart1,(uint8_t *)&mx,strlen(mx),50);
for(int i=0; i<sizeof(mx);i++){mx[i]=0;}	
HAL_Delay(10000);
}
 void mqtt(void)
 { 

//char tx[40];
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
//int rc = 0;
unsigned char buf[200];
MQTTString topicString = MQTTString_initializer;
char* payload = "yourpayload";
int payloadlen = strlen(payload);
int buflen = sizeof(buf);

data.clientID.cstring = "me";
data.keepAliveInterval = 20;
data.cleansession = 1;
int len = MQTTSerialize_connect(buf, buflen, &data); /* 1 */

topicString.cstring = "yourtopic";
len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, topicString, payload, payloadlen); /* 2 */

len += MQTTSerialize_disconnect(buf + len, buflen - len); /* 3 */
	 mqtt_init();
//sprintf(tx,"AT+QISEND\r\n");	 
//HAL_UART_Transmit(&huart1,(uint8_t *)&tx,strlen(tx),50);
//HAL_Delay(5000);
HAL_UART_Transmit(&huart1,(uint8_t *)&buf,len,50);
/*
rc = Socket_new("127.0.0.1", 1883, &mysock);
rc = write(mysock, buf, len);
rc = close(mysock);*/
 }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
