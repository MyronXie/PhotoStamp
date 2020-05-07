/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @Version        : 1.4(200507)
  * @Author         : Myron Xie
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
#include "gps.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* ========== FUNC SELECT ========== */
//#define     FUNC_CAM                // Enbale camera trigger function
#define     FUNC_GPS                // Enable GPS recording function

//#define     TRIG_MODE_FALLING
#define     TRIG_MODE_PWM
/* ========== FUNC SELECT ========== */


/* System */
uint8_t     errorCode = 0x00;

/* GPS */
GPSMsgType  gpsMsg;
uint8_t     gpsStatus   = 0x00;     // bit0: onboard  bit1: received  bit2: valid
uint8_t     gpsAvlFlag  = 0;        // Whether the msg is decoded
uint64_t    gpsTick     = 0;

/* File */
FATFS       fs;                     // Work area (file system object) for logical drive
FIL         fil;                    // file objects
char        fileName[16]="default0.txt"; // Max Length:12 (255 not support yet)
char        fileBuf[60];
uint16_t    seqID       = 0;
uint32_t    fileBytes   = 0;        // No use, only a param of f_write
uint8_t     fileOpened  = 0;

/* USART */
uint8_t     recvByte    = 0;
uint8_t     msgBuf[100];

/* PWM */
volatile static uint32_t uwDiffCapture = 0;     // Positive pulse width of PWM
uint8_t     trigMode = 0;
uint64_t    trigTick = 0;
#define     TRIG_PWM_MIN    1600
#define     TRIG_PWM_MAX    1900
#define     TRIG_GUARD  500     //ms

/* Camera */
#ifdef FUNC_CAM
#define CAM_OFF     0           // Camera running status
#define CAM_ON      1
#define CAM_TRIGGER 2000        // (us) Threshold of positive width of input PWM
#define CAM_TIMEOUT 3000        // (ms) Timeout for camera

typedef struct
{
    uint8_t         status;
    GPIO_TypeDef*   output_base;
    uint16_t        output_pin;
    GPIO_TypeDef*   input_base;
    uint16_t        input_pin;
    uint16_t        time;
    uint16_t        delay;
    uint64_t        tick;
}CamType;

// For debug, just comment the camera(s) you don't need, and change CAMNUM
#define CAMNUM 5
CamType camera[CAMNUM] = {
    {CAM_OFF,CAM1_OUT_GPIO,CAM1_OUT_PIN,CAM1_IN_GPIO,CAM1_IN_PIN,0,0,0},
    {CAM_OFF,CAM2_OUT_GPIO,CAM2_OUT_PIN,CAM2_IN_GPIO,CAM2_IN_PIN,0,0,0},
    {CAM_OFF,CAM3_OUT_GPIO,CAM3_OUT_PIN,CAM3_IN_GPIO,CAM3_IN_PIN,0,0,0},
    {CAM_OFF,CAM4_OUT_GPIO,CAM4_OUT_PIN,CAM4_IN_GPIO,CAM4_IN_PIN,0,0,0},
    {CAM_OFF,CAM5_OUT_GPIO,CAM5_OUT_PIN,CAM5_IN_GPIO,CAM5_IN_PIN,0,0,0},
};

uint8_t     camMode = 0x00;
uint64_t    camTick = 0;
uint8_t     camCnt  = 0;
uint16_t    maxTime = 0;
#endif

/* ========== LED ========== */
#define LED_ON()    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET)
#define LED_TOG()   HAL_GPIO_TogglePin(LED_GPIO, LED_PIN)

#define LED_ALWAYS_OFF      0x00
#define LED_ALWAYS_ON       0x10
#define LED_TWINKLE_LONG    0x20
#define LED_TWINKLE_SHORT   0x21
#define LED_TWINKLE_MID     0x22
#define LED_FLASH_DIM       0x30
#define LED_FLASH_DIM_WAIT  0x31
uint8_t     ledMode = 0x00;
uint64_t    ledTick = 0;

/* ========== KEY ========== */
uint8_t keyState    = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
    MX_USART2_UART_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
    //Enable USART1 & USART2
    USART_Recv_Enable();
    
    printf("\r\n***** PhotoStamp v1.4 *****\r\n");
    
    // File Process
    printf("\r\n[FILE] Mount ");
    retSD = f_mount(&fs, "0:", 1);
    if(retSD)
    {
        printf("error: %d\r\n",retSD);
        errorCode = retSD;
    }
    else    printf("success!\r\n");
        
    // Start PWM monitoring
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    ledMode = LED_TWINKLE_LONG;

    while(1)
    {
        // Fatal error process: Turn off the LED & Exit the program
        if(errorCode)
        {
            printf("  FATAL ERROR: %d",errorCode);
            LED_OFF();
            return 0;
        }
        
        // Create Files when date and time are available
        if(!fileOpened)
        {
            if(gpsStatus&0x01)
            {
                sprintf(fileName,"%02d%02d%02d%02d.txt",gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute);
                printf("\r\n[FILE] Open File: %s",fileName);
                retSD = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE);
                if(retSD)
                {
                    printf(" error: %d\r\n",retSD);
                    errorCode = retSD;
                }
                else
                {   
                    printf(" success!\r\n");
                    fileOpened = 1;
                }
            }
        }
        
        #ifdef FUNC_CAM
        /* ========== Camera state machine ========== */
        switch(camMode)
        {
            // Waiting for PWM input singal 
            case 0x00:
                //if(HAL_GetTick()%2001==0)//<--- Modify the value here to change trigger frequency
                if(uwDiffCapture>CAM_TRIGGER-25 && uwDiffCapture<CAM_TRIGGER+25)
                {
                    camMode = 0x01;
                    camTick = HAL_GetTick();
                }
                break;
            
            // Camera control signal [Output]
            case 0x01:
                camCnt = 0;
                for(int i=0; i<CAMNUM; i++)
                {
                    if(camera[i].status == CAM_ON)
                    {
                        camCnt++;
                        continue;
                    }
                    else if((HAL_GetTick()-camTick) >= camera[i].delay)
                    {
                        camera[i].status = CAM_ON;
                        camera[i].tick = HAL_GetTick();
                        HAL_GPIO_WritePin(camera[i].output_base, camera[i].output_pin, GPIO_PIN_SET);
                    }
                }
                if(camCnt>=CAMNUM)
                {
                    //printf("OUTPUT:%4d,%4d,%4d,%4d,%4d\r\n",(int)camera[0].tick%1000,(int)camera[1].tick%1000,(int)camera[2].tick%1000,(int)camera[3].tick%1000,(int)camera[4].tick%1000);
                    //printf("%04d:",HAL_GetTick()%1000);
                    camMode = 0x02;
                }
                break;
            
            // Waiting for camera response [input]
            case 0x02:
                camCnt = 0;
                for(int i=0; i<CAMNUM; i++)
                {
                    if(camera[i].status == CAM_OFF)
                    {
                        camCnt++;
                        continue;
                    }
                    else if(HAL_GPIO_ReadPin(camera[i].input_base, camera[i].input_pin)==GPIO_PIN_SET)
                    {
                        camera[i].status = CAM_OFF;
                        camera[i].time = HAL_GetTick() - camera[i].tick;
                        HAL_GPIO_WritePin(camera[i].output_base, camera[i].output_pin, GPIO_PIN_RESET);
                    }
                    else if((HAL_GetTick()-camera[i].tick) >= CAM_TIMEOUT)
                    {
                        camera[i].status = CAM_OFF;
                        camera[i].time = CAM_TIMEOUT;
                        HAL_GPIO_WritePin(camera[i].output_base, camera[i].output_pin, GPIO_PIN_RESET);
                    }
                }

                if(camCnt>=CAMNUM)
                {
                    //printf(" INPUT:%4d,%4d,%4d,%4d,%4d\r\n",(int)camera[0].tick%1000,(int)camera[1].tick%1000,(int)camera[2].tick%1000,(int)camera[3].tick%1000,(int)camera[4].tick%1000);
                    camMode = 0x03;
                }
                break;
            
            // Calculate camera delay time
            case 0x03:
                memset(fileBuf,0,30);
                sprintf(fileBuf,"\r\n%4d,%4d,%4d,%4d,%4d",camera[0].time,camera[1].time,camera[2].time,camera[3].time,camera[4].time);
                retSD = f_write(&fil, fileBuf, sizeof(fileBuf), (void *)&fileBytes);
                retSD = f_sync(&fil);
                printf("%s",fileBuf);
                maxTime = 0;
                for(int i=0; i<CAMNUM; i++)
                {
                    printf("%3d,",camera[i].time);
                    if(camera[i].time > maxTime)    maxTime = camera[i].time;
                }
                printf("\r\n");
                for(int i=0; i<CAMNUM; i++)
                {
                    camera[i].delay = maxTime - camera[i].time;
                }
                //printf(" DELAY:%4d,%4d,%4d,%4d,%4d\r\n",camera[0].delay,camera[1].delay,camera[2].delay,camera[3].delay,camera[4].delay);
                camMode = 0x04;
                break;

            // Waiting for PWM signal disapper to avoid trigger continously
            case 0x04:
                if(uwDiffCapture<CAM_TRIGGER-25 || uwDiffCapture>CAM_TRIGGER+25)
                {
                    camMode = 0x05;
                    camTick = HAL_GetTick();
                }
                break;
            
            case 0x05:
                if(HAL_GetTick()-camTick>=100)
                {
                    camMode = 0x00;
                }
                break;
                
            default:
                camMode = 0x00;
                break;        
        }
        
        #endif /* FUNC_CAM */
        
        #ifdef FUNC_GPS
        /* ========== GPS ========== */
        
        if(GPS_Buffer_Available())
        {
            recvByte = GPS_Buffer_NextByte();
            gpsAvlFlag = GPS_MsgRecv(recvByte);
        }

        if(gpsAvlFlag)
        {
            memcpy(msgBuf,recvBuf,gpsAvlFlag);
            *(msgBuf+gpsAvlFlag)='\0';       // Make buffer more readable
            gpsStatus = GPS_Decode((char*)msgBuf,&gpsMsg,gpsAvlFlag);
            gpsAvlFlag = 0;
        }
        
        if(gpsStatus&0x10)
        {
            gpsTick = HAL_GetTick();
            if(ledMode != LED_FLASH_DIM_WAIT) ledMode = LED_ALWAYS_ON;
        }
        else if(gpsStatus&0x01)
        {
            ledMode = LED_TWINKLE_MID;
        }
        else
        {
            if(HAL_GetTick()-gpsTick>3000)  // GPS Lost
            {
                ledMode = LED_TWINKLE_LONG;
            }
        }

        #endif /* FUNC_GPS */

        /* ========== File Recorder ========== */
        switch(trigMode)
        {
            case 0x00:
                if(HAL_GetTick()-trigTick > TRIG_GUARD)
                {
                    #ifdef TRIG_MODE_PWM
                    if((uwDiffCapture>(TRIG_PWM_MIN-50))&&(uwDiffCapture<(TRIG_PWM_MAX+50)))    trigMode = 0x01;
                    #endif
                }
                if(trigMode!=0x01)  break;
            
            case 0x01:
                if(fileOpened)
                {
                    ledMode = LED_FLASH_DIM;
                    memset(fileBuf,0,sizeof(fileBuf));
                    sprintf(fileBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.1f",++seqID,
                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
                    retSD = f_write(&fil, fileBuf, sizeof(fileBuf), (void *)&fileBytes);
                    retSD = f_sync(&fil);
                    printf("%s",fileBuf);
                }
                trigTick = HAL_GetTick();
                trigMode = 0x02;
                break;
            
            case 0x02:
                #ifdef TRIG_MODE_PWM
                if(uwDiffCapture<(TRIG_PWM_MIN-50))  trigMode = 0x00;
                #endif
                #ifdef TRIG_MODE_FALLING
                trigMode = 0x00;
                #endif
                break;

            default:
                trigMode = 0x00;
                break;

        }

        /* ========== LED ========== */
        
        if(HAL_GetTick()%1000==0)   printf("\r\n[SYS] %d: <LED>%x, <GPS>%d \r\n",HAL_GetTick()/1000,ledMode,(int)gpsTick);
        
        switch(ledMode)
        {            
            case LED_ALWAYS_OFF:
                LED_OFF();
                break;
            
            case LED_ALWAYS_ON:
                LED_ON();
                break;
            
            case LED_TWINKLE_LONG:
                if(HAL_GetTick()-ledTick>1000)
                {
                    LED_TOG();
                    ledTick = HAL_GetTick();
                }
                break;
                
            case LED_TWINKLE_SHORT:
                if(HAL_GetTick()-ledTick>100)
                {
                    LED_TOG();
                    ledTick = HAL_GetTick();
                }
                break;
                
            case LED_TWINKLE_MID:
                if(HAL_GetTick()-ledTick>500)
                {
                    LED_TOG();
                    ledTick = HAL_GetTick();
                }
                break;
            
            case LED_FLASH_DIM:
                LED_OFF();
                ledTick = HAL_GetTick();
                ledMode = LED_FLASH_DIM_WAIT;
                break;
            
            case LED_FLASH_DIM_WAIT:
                if(HAL_GetTick()-ledTick>300)
                {
                    ledMode = LED_ALWAYS_ON;
                }
                break;
                
            default:
                ledMode = LED_ALWAYS_OFF;
                break;
        }

        /* ========== KEY ========== */
        if(keyState != HAL_GPIO_ReadPin(KEY_GPIO,KEY_PIN))
        {
            keyState = HAL_GPIO_ReadPin(KEY_GPIO,KEY_PIN);
            
            if(keyState == GPIO_PIN_SET)
            {
                //printf("\r\n Key reset.");
            }
            else
            {
                //printf("\r\n Key pressed.");
            }    
        }
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if(fileOpened)
    {
        retSD = f_close(&fil);
        if(retSD)   printf(" Close error: %d\r\n",retSD);
        else        printf(" Close success!\r\n");
    }
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    volatile static uint32_t uwIC1Value1 = 0, uwIC1Value2 = 0;
    static uint8_t tim_flag = 0;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if(HAL_GPIO_ReadPin(TRIG_GPIO,TRIG_PIN) == GPIO_PIN_SET)
        {
            uwIC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            tim_flag = 0x01;
        }
        else
        {
            uwIC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            if(tim_flag==0x01)  tim_flag = 0x02;
            else tim_flag = 0x00;
        }
        if(tim_flag==0x02)
        {
            if(uwIC1Value2 > uwIC1Value1)
                uwDiffCapture = uwIC1Value2 - uwIC1Value1; 
            else
                uwDiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) - uwIC1Value1 + 1) + uwIC1Value2;

            //printf("%d,%d,%d\r\n",uwIC1Value1,uwIC1Value2,uwDiffCapture);  // [debug] test input pwm duty
        }
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == CTRL_PIN)
    {
        #ifdef TRIG_FALLING
        if(trigMode = 0x00) trigMode = 0x01;
        #endif
//        if(!gpsMode)
//        {
//            gpsMode = 0x01;
//            ledMode = LED_FLASH_DIM;
//            if(HAL_GetTick()-fileTick>500)   //file save guard time
//            {
//                if(fileOpened)
//                {
//                    memset(fileBuf,0,100);
//                    sprintf(fileBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.2f",seqID++,
//                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
//                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
//                    retSD = f_write(&fil, fileBuf, sizeof(fileBuf), (void *)&fileBytes);
//                    retSD = f_sync(&fil);
//                    printf("%s",fileBuf);
//                }
//            }
//            fileTick = HAL_GetTick();
//            gpsMode = 0x00;
//        }
//        else
//        {
//            printf("Too quick!");
//        }
    }

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

/****END OF FILE****/
