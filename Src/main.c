
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define STEER_TIME  600     //ms
#define RELAY_TIME  250

#define STEER_M45   600     //800
#define STEER_0     803     //1020
#define STEER_P45   1008    //1245

#define TRIGGER_ON  1940
#define TRIGGER_OFF 1100

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    double latitude;
    double longitude;
    float height;
}GPSMsgType;

GPSMsgType gpsMsg;

FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects
char fileName[20]="default.txt";
uint32_t byteswritten;

uint16_t seqid = 1;
uint8_t aRxBuffer = 0, bRxBuffer = 0;
uint8_t rxFlag = 0;
uint8_t rmcBuffer[100],printBuffer[100];
char gpsmsgBuf[100];
uint8_t length = 0;
uint8_t decodeFlag=0;
uint8_t moveFlag=0;

#define startAddr 0X08010000
FLASH_EraseInitTypeDef fl_erase;
uint32_t PageError = 0;
uint16_t fileNum=0;
const uint16_t magicNum = 0x1016;

uint16_t steerPulse = 500;
TIM_OC_InitTypeDef sConfigOC4,sConfigOC3;

uint32_t    uwIC1Value1 = 0;
uint32_t    uwIC1Value2 = 0;
uint32_t    uwDiffCapture = 0;

uint8_t sysMode=0x00;
uint64_t sysTick = 0;
uint64_t lstGpsTick = 0;
uint64_t lstLedTick = 0;
uint8_t gpsStatus=0x00;     // bit0: have  bit2: valid

uint8_t keyState = 1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t GPS_MsgRecv(uint8_t ch);
uint8_t GPS_Decode(char* str,uint8_t len);
uint8_t GetComma(char* str,uint8_t num);
double GetDoubleNumber(char *s);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_RTC_Init();
  MX_FATFS_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
    HAL_UART_Receive_IT(&huart2, &bRxBuffer, 1);
    
    fileNum = *(__IO uint32_t*)(startAddr);
    if(fileNum != magicNum)   fileNum = 0;
	else fileNum = *(__IO uint32_t*)(startAddr+4);
    fileNum++;
  
	HAL_FLASH_Unlock();
	fl_erase.TypeErase=FLASH_TYPEERASE_SECTORS;
	fl_erase.Sector=FLASH_SECTOR_4;
	fl_erase.NbSectors=1;
	fl_erase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
	
	HAL_FLASHEx_Erase(&fl_erase, &PageError);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, startAddr, (uint32_t)magicNum);
    HAL_FLASH_Program(TYPEPROGRAM_WORD, startAddr+4, (uint32_t)fileNum);
	
	HAL_FLASH_Lock();
	
	fileNum = *(__IO uint32_t*)(startAddr+4);
    
    printf("\r\n***File name: LOG%04d.txt***",fileNum);
    
    printf("\r\n Mount: ");
    retSD = f_mount(&fs, "0:", 1);
    if(retSD)   printf("error: %d",retSD);
    else        printf("success!");
    
    printf("\r\n Open : ");
    sprintf(fileName,"LOG%04d.txt",fileNum);
    retSD = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE);
    if(retSD)   printf("error: %d",retSD);
    else        printf("success!");

    sConfigOC4.OCMode = TIM_OCMODE_PWM1;
    sConfigOC4.Pulse = STEER_M45;
    sConfigOC4.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC4.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  
    sConfigOC3.OCMode = TIM_OCMODE_PWM1;
    sConfigOC3.Pulse = 500;
    sConfigOC3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC3.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
      
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // GPS msg decode
        if(decodeFlag)
        {
            //printf("\r\n%s",printBuffer);
            GPS_Decode((char*)printBuffer,decodeFlag);
            decodeFlag=0;
            lstGpsTick = HAL_GetTick();
            if(!(gpsStatus&0x01))
            {
                printf("\r\n GPS onboard!");
                gpsStatus |= 0x01;
            }
        }
        if(gpsStatus&0x01)
        {
            if(HAL_GetTick()-lstGpsTick>2500)
            {
                printf("\r\n GPS lost!");
                gpsStatus &= ~0x01;
            }
        }
        
        // LED mode
        if(gpsStatus&0x04)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        }
        else if(gpsStatus&0x01)
        {
            if(gpsStatus&0x02)
            {
                if(HAL_GetTick()-lstLedTick>1000)
                {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                    lstLedTick = HAL_GetTick();
                }
            }
            else
            {
                if(HAL_GetTick()-lstLedTick>500)
                {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                    lstLedTick = HAL_GetTick();
                }
            }
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
  
        // Output judge
        switch(sysMode)
        {
            case 0x00:
                sConfigOC4.Pulse = STEER_M45;
                HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                sysMode = 0x01;
                sysTick = HAL_GetTick();
                break;
            
            case 0x01:
                if(HAL_GetTick()-sysTick>STEER_TIME)
                {
                    sysMode = 0x02;
                    moveFlag=2;
                }
                break;
            
            case 0x02:
                if(moveFlag==1)    sysMode = 0x10;
                break;
            
            case 0x10:
                sConfigOC4.Pulse = STEER_M45;
                HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                sysTick = HAL_GetTick();
                sysMode = 0x11;
                break;
            
            case 0x11:
                if(HAL_GetTick()-sysTick>STEER_TIME)   sysMode = 0x12;
                break;
            
            case 0x12:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
                memset(gpsmsgBuf,0,100);
                sprintf(gpsmsgBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.2f",seqid++,
                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
                retSD = f_write(&fil, gpsmsgBuf, sizeof(gpsmsgBuf), (void *)&byteswritten);
                retSD = f_sync(&fil);
                printf("%s",gpsmsgBuf);
                sysTick = HAL_GetTick();
                sysMode = 0x13;
                break;
            
            case 0x13:
                if(HAL_GetTick()-sysTick>RELAY_TIME)
                {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                    sysMode = 0x20;
                }
                break;
            
            case 0x20:
                sConfigOC4.Pulse = STEER_0;
                HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                sysTick = HAL_GetTick();
                sysMode = 0x21;
                break;
            
            case 0x21:
                if(HAL_GetTick()-sysTick>STEER_TIME)   sysMode = 0x22;
                break;
            
            case 0x22:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
                memset(gpsmsgBuf,0,100);
                sprintf(gpsmsgBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.2f",seqid++,
                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
                retSD = f_write(&fil, gpsmsgBuf, sizeof(gpsmsgBuf), (void *)&byteswritten);
                retSD = f_sync(&fil);
                printf("%s",gpsmsgBuf);
                sysMode = 0x23;
                sysTick = HAL_GetTick();
                break;
            
            case 0x23:
                if(HAL_GetTick()-sysTick>RELAY_TIME)
                {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                    sysMode = 0x30;
                }
                break;
            
            case 0x30:
                sConfigOC4.Pulse = STEER_P45;
                HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
                sysTick = HAL_GetTick();
                sysMode = 0x31;
                break;
            
            case 0x31:
                if(HAL_GetTick()-sysTick>STEER_TIME)   sysMode = 0x32;
                break;
            
            case 0x32:
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
                memset(gpsmsgBuf,0,100);
                sprintf(gpsmsgBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.2f",seqid++,
                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
                retSD = f_write(&fil, gpsmsgBuf, sizeof(gpsmsgBuf), (void *)&byteswritten);
                retSD = f_sync(&fil);
                printf("%s",gpsmsgBuf);
                sysMode = 0x33;
                sysTick = HAL_GetTick();
                break;
            
            case 0x33:
                if(HAL_GetTick()-sysTick>RELAY_TIME)
                {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
                    sysMode = 0x00;
                }
                break;
        }
        
        // Input PWM
        if((uwDiffCapture>(TRIGGER_ON-50)&&uwDiffCapture<(TRIGGER_ON+50))&&(moveFlag==0))     //1940
        {
            moveFlag=1;

        }
        if((uwDiffCapture>(TRIGGER_OFF-50)&&uwDiffCapture<(TRIGGER_OFF+50))&&(moveFlag==2))     //1100
        {
            moveFlag=0;
        }

        // Key
        if(keyState!=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
        {
            keyState = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);
            
            if(keyState == GPIO_PIN_SET)
            {
                moveFlag=1;
                printf("\r\n Key reset.");
            }
            else if(keyState == GPIO_PIN_RESET)
            {
                printf("\r\n Key pressed.");
            }    
        }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
    
    retSD = f_close(&fil);
    if(retSD)   printf(" Close error: %d\r\n",retSD);
    else        printf(" Close success!\r\n");
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        //HAL_UART_Transmit(&huart1, &aRxBuffer, 1, 1); // debug: test gps message
        
        length = GPS_MsgRecv(aRxBuffer);
        if(length)
        {
            memset(printBuffer,0,80);
            memcpy(printBuffer,rmcBuffer,length);
            decodeFlag = length;
        }
        HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
    }
    
    if(huart->Instance == USART1)
    {
        HAL_UART_Transmit(&huart1, &bRxBuffer, 1, 1);
        HAL_UART_Receive_IT(&huart1, &bRxBuffer, 1);
    }
}

uint8_t GPS_MsgRecv(uint8_t ch)
{
    static uint8_t stage = 0x00;
    static uint8_t cnt = 0;
    uint8_t ret = 0;
    
    switch(stage)
    {
        case 0x00:
            if(ch=='$')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x01;
            }
            break;
            
        case 0x01:
            if(ch=='G')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x02;
            }
            else
            {
                stage = 0x00;
                cnt = 0;
            }
            break;
            
        case 0x02:
            if(ch=='P'||ch=='N')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x03;
            }
            else
            {
                stage = 0x00;
                cnt = 0;
            }
            break;
            
        case 0x03:
            //if(ch=='R'||ch=='G')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x04;
            }
            //else
            //{
            //    stage = 0x00;
            //    cnt = 0;
            //}
            break;
            
        case 0x04:
            //if(ch=='M'||ch=='G')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x05;
            }
            //else
            //{
            //    stage = 0x00;
            //    cnt = 0;
            //}
            break;
            
        case 0x05:
            //if(ch=='C'||ch=='A')
            {
                rmcBuffer[cnt++]=ch;
                stage = 0x06;
            }
            //else
            //{
            //    stage = 0x00;
            //    cnt = 0;
            //}
            break;
            
        case 0x06:
            rmcBuffer[cnt++]=ch;
            if(ch=='*') stage = 0x07;
            break;
        
        case 0x07:
            rmcBuffer[cnt++]=ch;
            stage = 0x08;
            break;
        
        case 0x08:
            rmcBuffer[cnt++]=ch;
            ret = cnt;
            cnt = 0;
            stage = 0x00;
            break;
    }
    return ret;
}


uint8_t GPS_Decode(char* buf,uint8_t len)
{
    uint8_t tmp,status;
    
    if(!strncmp("$GNRMC",buf,6))
    {
        status = buf[GetComma(buf,2)];

        gpsMsg.hour = (buf[7] - '0') * 10 + (buf[8] - '0');
        gpsMsg.minute = (buf[9] - '0') * 10 + (buf[10] - '0');
        gpsMsg.second = (buf[11] - '0') * 10 + (buf[12] - '0');

        tmp = GetComma(buf,9);
        gpsMsg.day = (buf[tmp + 0] - '0') * 10 + (buf[tmp + 1] - '0');
        gpsMsg.month = (buf[tmp + 2] - '0') * 10 + (buf[tmp + 3] - '0');
        gpsMsg.year = (buf[tmp + 4] - '0') * 10 + (buf[tmp + 5] - '0') + 2000;

        if(gpsMsg.year>2000&&gpsMsg.year<2099)
        {
            gpsStatus |= 0x02;
        }
        else gpsStatus &= ~0x02;
        
        if(status=='A')
        {           
            gpsMsg.latitude = GetDoubleNumber(&buf[GetComma(buf,3)])/100;
            gpsMsg.longitude = GetDoubleNumber(&buf[GetComma(buf,5)])/100;
            
            if(!(gpsStatus&0x04))
            {
                printf("\r\n GPS data available!");
                gpsStatus |= 0x04;
            }
        }
        else
        {
            gpsMsg.latitude = 0;
            gpsMsg.longitude = 0;
            if(gpsStatus&0x04)
            {
                printf("\r\n GPS data invalid!");
                gpsStatus &= ~0x04;
            }
        }
    }
    
    else if(!strncmp("$GNGGA",buf,6))
    {
        gpsMsg.height=GetDoubleNumber(&buf[GetComma(buf,9)]);
    }

    return 0;
}

uint8_t GetComma(char* str,uint8_t num)
{
    uint8_t cnt=0,tmp=0,comma=0;
    while(comma!=num)
    {
        tmp=str[cnt++];
        if(tmp=='\0') return 0;
        if(tmp==',') comma++;
    }
    return cnt;
}

double GetDoubleNumber(char *s)
{
    char buf[128];
    int i;
    double rev;
    i=GetComma(s,1);
    strncpy(buf,s,i);
    buf[i]=0;
    rev=atof(buf);

    return rev;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_SET)
        {
            uwIC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
        }
        else
        {
            uwIC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            if(uwIC1Value2 > uwIC1Value1)
                uwDiffCapture = uwIC1Value2 - uwIC1Value1; 
            else
                uwDiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -uwIC1Value1 + 1) + uwIC1Value2; 
        }   
    }
//             printf("\r\n%d",uwDiffCapture);  // debug: test input pwm duty
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
