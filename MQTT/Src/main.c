/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "MQTTPacket.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
HAL_SD_CardInfoTypeDef SDCardInfo; 
/* Private variables ---------------------------------------------------------*/
FRESULT res;
#define RX_TIMEOUT          ((uint32_t)0xFFFFFFFF)
char* UploadFile = "upload.txt";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void main_menu(void);
void mqtt_init(void);
int read_file(char* filename, char* Buff);
void mqtt_pub(void);
void mqtt_connect(void);
int fsize(char* file);
void gprs_initialize(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char buffr[500];
unsigned char SMSCTRLZ   = 0x1A;
char mx[60];char vx[50];
uint8_t receive,rx=0,cnt=0;
unsigned int z=0;
bool UPLOAD_FILE=false,LOG_DATA=false;
char data[200];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
   res = f_mount(&SDFatFS, SDPath, 1);
   if(res!=FR_OK){HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"SD Card Not Mounted\r\n",21);}
   else
    {
      HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"SD Card Mounted\r\n",17);
      main_menu();//Displays Run time Status on Serial Port
    }
   	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(UPLOAD_FILE == true)
    {
			memset(buffr,'\0',sizeof(buffr));
		  int leng = read_file(UploadFile,buffr);
			/*Introduced As Guard Time or UART transmission fucks Up(Spent 5 hrs on this)
			NOT Necessary here but was a PINTA when executing two UART_transmit command successively.
			*/
			HAL_Delay(300);
      mqtt_pub();
      UPLOAD_FILE=false;
    }
		HAL_Delay(50);
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

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
int read_file(char* filename, char* Buff)
{
  int j=0;
  while(z<fsize(UploadFile))
  {
		f_open(&SDFile,UploadFile,FA_READ);
    char mydata[15]="\0";
    memset(mydata,0,sizeof(mydata));    
    f_lseek(&SDFile, z);
    f_gets(mydata,sizeof(mydata),&SDFile);
    z +=strlen(mydata);
    j += sprintf( Buff + j, mydata);
		f_close(&SDFile);
  } 
  return z; 
}
void gprs_initialize(void)
{
  char AT [9][30] = {
       "AT+QIFGCNT=0\r\n","AT+QICSGP=1,\"CMNET\"\r\n","AT+QIMUX=0\r\n","AT+QIMODE=1\r\n"
       ,"AT+QITCFG=3,2,512,1\r\n","AT+QIDNSIP=1\r\n","AT+QIREGAPP\r\n","AT+QIACT\r\n","AT+QILOCIP\r\n"};
  if(cnt<=8)
  {
    char mydata[100]="\0";
    sprintf(mydata,"%s",AT[cnt]);
    HAL_UART_Transmit_DMA(&huart1,(uint8_t*)&mydata,strlen(mydata));
    ++cnt;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_DMA(&huart1,(uint8_t *)&receive,sizeof(receive));
  data[rx]=receive;
   if((strstr(data,"SMS Ready")!=NULL)&&(strstr(data,"+CIEV:")!=NULL))
   {
      HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"AT\r\n",4);
      memset(data,'\0',sizeof(data));rx=0;  
   }
   else if(data[rx-3]=='O'&&data[rx-2]=='K'&&data[rx-1]=='\r'&&data[rx]=='\n')
   {
    gprs_initialize();
    memset(data,'\0',sizeof(data));rx=0;
   }

   else if(cnt==9)
   {
    memset(data,'\0',sizeof(data));rx=0;
    ++cnt;
    HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"AT+QIOPEN=\"TCP\",\"DOMAIN_NAME\",\"PORT\"\r\n",45);
   }
    //For Connecting to the HTTP Server
  //else if(data[rx-6]=='C'&&data[rx-5]=='O'&&data[rx-4]=='N'&&data[rx-3]=='N'&&data[rx-2]=='E'&&data[rx-1]=='C'&&data[rx]=='T')//Furtehr Add OR condition for continuous data logging
  else if((strstr(data,"CONNECT\r\n")!=NULL)||cnt==11)
	{
			if(cnt==10)
			{
				mqtt_connect();
				++cnt;
			}
			else if(cnt==11)
			{
				/*TODO: Verify after mqtt_connect does it return CONNECT or just some encoded text*/
				UPLOAD_FILE=true;
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_14);
				++cnt;
			}
      memset(data,'\0',sizeof(data));rx=0;
     }
	  else{rx++;}
}
void mqtt_connect(void)
{
MQTTPacket_connectData datas = MQTTPacket_connectData_initializer;
unsigned char buf[200];
MQTTString topicString = MQTTString_initializer;
int buflen = sizeof(buf);

datas.username.cstring = "*****";//Port to your requirement
datas.password.cstring = "********";//Port to your requirement
datas.clientID.cstring = "*********";//Port to your requirement
datas.keepAliveInterval = 120;
datas.cleansession = 1;
int len = MQTTSerialize_connect(buf, buflen, &datas); 

HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,len);
}

 void mqtt_pub()
 {
unsigned char buf[600];	 
MQTTString topicString = MQTTString_initializer; 
int payloadlen = strlen(buffr);
int buflen = sizeof(buf);
topicString.cstring = "rtc";
int len = MQTTSerialize_publish(buf, buflen , 0, 0, 0, 0, topicString, buffr, payloadlen);
HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,len);
}


void main_menu(void)
{
  char tx[200];
  sprintf(tx,"\r\n================= Main Menu ==================\r\n\n");
    HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&tx,strlen(tx));
    HAL_Delay(100);
    memset(tx,'\0',sizeof(tx));
    sprintf(tx,"  Sync Data       ------------------------------ 0\r\n\n");
    HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&tx,strlen(tx));
    HAL_Delay(100);
    memset(tx,'\0',sizeof(tx));
    sprintf(tx,"  Run application ----------------------------- 1\r\n\n");
    HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&tx,strlen(tx));
    HAL_Delay(100);
    memset(tx,'\0',sizeof(tx));
    HAL_UART_Receive(&huart1, &cnt, 1, RX_TIMEOUT);
    if(cnt=='0')
    {
      HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"RESET the EVAL-BOARD\r\n",22);
      HAL_UART_Receive_DMA(&huart1,(uint8_t *)&receive,sizeof(receive));
      cnt=0;
    }
    else if(cnt=='1')
    {
      LOG_DATA=true;
      cnt=0;
    }
}
int fsize(char* file)
{
    int len =0;
    f_open(&SDFile,file, FA_READ);
    len =  (int)f_size(&SDFile);
    f_close(&SDFile);
    return len;
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
