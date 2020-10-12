/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @version        : 2.0-beta3(201009)
  * @author         : Myron Xie
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
#include "utils.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* System */
uint8_t     sysMode     = 0x00;     // System running status
uint8_t     sysModeLst  = 0xFF;     // System running status
uint8_t     errorCode   = 0x00;     // If errorCode>0, shut down system immediately

/* GPS */
GPSMsgType  gpsMsg;                 // Contains time and location msg
uint8_t     gpsStatus   = 0x00;     // bit0: onboard,  bit1: received,  bit2: valid
uint8_t     gpsAvlFlag  = 0;        // Whether the msg is decoded
uint64_t    gpsTick     = 0;

/* File */
FATFS       fs;                     // Work area (file system object) for logical drive
FIL         fil;                    // file objects
char        fileName[16]="default0.txt"; // Max Length:12 (255 not support yet)
char        fileBuf[70];
uint16_t    seqID       = 0;
uint32_t    fileBytes   = 0;        // No use, only a param of f_write
uint8_t     fileOpened  = 0;

/* USART */
uint8_t     recvByte    = 0;
uint8_t     msgBuf[100];

/* PWM */
volatile static uint32_t uwDiffCapture = 0;     // Positive pulse width of PWM
volatile static uint32_t uwTest = 0;
uint8_t     trigFlag = 0;
uint64_t    trigTick = 0;

/* Camera */
#ifdef  FUNC_CAM

// For debug, just comment the camera(s) you don't need, and change CAMNUM
#define CAMNUM 5
CamType camera[CAMNUM] = {
    {CAM1_OUT_GPIO,CAM1_OUT_PIN,CAM1_IN_GPIO,CAM1_IN_PIN,CAM1_LED_GPIO,CAM1_LED_PIN,0,0,0,0},
    {CAM2_OUT_GPIO,CAM2_OUT_PIN,CAM2_IN_GPIO,CAM2_IN_PIN,CAM2_LED_GPIO,CAM2_LED_PIN,0,0,0,0},
    {CAM3_OUT_GPIO,CAM3_OUT_PIN,CAM3_IN_GPIO,CAM3_IN_PIN,CAM3_LED_GPIO,CAM3_LED_PIN,0,0,0,0},
    {CAM4_OUT_GPIO,CAM4_OUT_PIN,CAM4_IN_GPIO,CAM4_IN_PIN,CAM4_LED_GPIO,CAM4_LED_PIN,0,0,0,0},
    {CAM5_OUT_GPIO,CAM5_OUT_PIN,CAM5_IN_GPIO,CAM5_IN_PIN,CAM5_LED_GPIO,CAM5_LED_PIN,0,0,0,0},
};

uint64_t    camTick     = 0;
uint8_t     camCnt      = 0;
#endif

/* LED */
uint8_t     ledMode = 0x00;
uint64_t    ledTick = 0;

/* KEY */
uint8_t     keyState    = 1;

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
  
    // Enable USART1 & USART2
    USART_Recv_Enable();
    
    printf("\r\n***** PhotoStamp v2.0 *****\r\n");
    
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

    for(int i=0; i<CAMNUM; i++)
    {
        HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_SET);
    }

    camTick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    ledMode = LED_TWINKLE_LONG;

    #ifdef FUNC_CAM
    sysMode = 0x00;
    #else
    sysMode = 0x20;
    #endif

    while(1)
    {      
        // Fatal error process: Turn off the LED & Exit the program
        if(errorCode)
        {
            printf("  FATAL ERROR: %d",errorCode);
            LED_OFF();
            HAL_GPIO_WritePin(EXT_LED_GPIO, EXT_LED_PIN, GPIO_PIN_RESET);
            return 0;
        }
        
        // Create Files when date and time are available
        if(!fileOpened)
        {
            if(gpsStatus&0x01)
            {
                sprintf(fileName,"%02d%02d%02d%02d.txt",gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute); // MMddhhmm.txt
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
        
        /* ========== System state machine ========== */
        
        if(sysModeLst!=sysMode)
        {
            sysModeLst=sysMode;
            printf("[Debug] Mode: 0x%02x\r\n",sysMode);
        }
        
//        if(HAL_GetTick()%2==0)
//        {
//            for(int i=0;i<5;i++)
//            {
//                if(HAL_GPIO_ReadPin(camera[i].input_base, camera[i].input_pin)==GPIO_PIN_RESET)
//                printf("CAM_IN:%d,0;\r\n",i);
//            }
//        }
        
        switch(sysMode)
        {
            #ifdef FUNC_CAM
            // Camera Startup Process
            case 0x00:
                if(HAL_GetTick()-camTick >= CAM_START_TIME_A)
                {
                    for(int i=0; i<CAMNUM; i++)
                    {
                        HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_RESET);
                    }

                    camTick = HAL_GetTick();
                    sysMode = 0x01;
                }
                break;

            case 0x01:
                if(HAL_GetTick()-camTick >= CAM_TRIGGED_TIME)
                {
                    for(int i=0; i<CAMNUM; i++)
                    {
                        HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_SET);
                    }

                    camTick = HAL_GetTick();
                    sysMode = 0x10;
                }
                break;

            case 0x10:
                if(HAL_GetTick()-camTick >= CAM_START_TIME_B-CAM_START_TIME_C)
                {
                    camTick = HAL_GetTick();
                    camCnt = 0;
                    sysMode = 0x11;
                }
                break;

            case 0x11:
                if(HAL_GetTick()-camTick >= CAM_START_TIME_C)
                {
                    HAL_GPIO_WritePin(camera[camCnt].output_gpio, camera[camCnt].output_pin, GPIO_PIN_RESET);
                    camTick = HAL_GetTick();
                    sysMode = 0x12;
                }
                break;

            case 0x12:
                if(HAL_GetTick()-camTick >= CAM_TRIGGED_TIME)
                {
                    HAL_GPIO_WritePin(camera[camCnt].output_gpio, camera[camCnt].output_pin, GPIO_PIN_SET);
                    camCnt++;
                    camTick = HAL_GetTick();
                    sysMode = 0x11;
                }
                else
                {
                    if(!(camera[camCnt].fb))
                    {
                        if(HAL_GPIO_ReadPin(camera[camCnt].input_gpio, camera[camCnt].input_pin)==GPIO_PIN_RESET)
                        {
                            camera[camCnt].fb = 1;
                            camera[camCnt].led = 1;
                            camera[camCnt].led_tick = HAL_GetTick();
                        }
                    }
                }
                if(camCnt >= CAMNUM)
                {
                    printf("[Init] Feedback: %1d, %1d, %1d, %1d, %1d", camera[0].fb, camera[1].fb, camera[2].fb, camera[3].fb, camera[4].fb);
                    for(int i=0;i<CAMNUM;i++)
                    {
                        camera[i].fb = 0;
                    }
                    camTick = HAL_GetTick();
                    sysMode = 0x13;
                }
                break;
            
            case 0x13:
                if(HAL_GetTick()-camTick >= CAM_START_TIME_C)
                {
                    camTick = HAL_GetTick();
                    sysMode = 0x20;
                }
                break;

            #endif /* FUNC_CAM initial */

            // Waiting for PWM input singal 
            case 0x20:
                if(HAL_GetTick()-camTick > TRIG_GUARD_TIME)
                {
                    #ifdef TRIG_MODE_PWM
                    //if(ISTRIGGED(uwTest))
                    if(ISTRIGGED(uwDiffCapture))
                    #endif
                    #ifdef TRIG_MODE_FALLING
                    if(trigFlag == 0x01)
                    #endif
                    {
                        //printf("PWN IN:%d\r\n",uwTest);
                        //printf("IN:%d\r\n",uwDiffCapture);
                        camTick = HAL_GetTick();
                        #ifdef FUNC_CAM
                        sysMode = 0x21;
                        #else
                        sysMode = 0x30;
                        #endif
                    }
                }
                if(sysMode!=0x21)   break;

            // Camera control signal [Output]
            case 0x21:
                camCnt = 0;
                for(int i=0; i<CAMNUM; i++)
                {
                    camera[i].status = CAM_ON;
                    HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_RESET);
                }
                camTick = HAL_GetTick();
                sysMode = 0x22;
                break;
            
            // Waiting for camera response [input]
            case 0x22:
                camCnt = 0;
                for(int i=0; i<CAMNUM; i++)
                {
                    if(camera[i].status == CAM_OFF)
                    {
                        camCnt++;
                        continue;
                    }
                    else if(HAL_GPIO_ReadPin(camera[i].input_gpio, camera[i].input_pin)==GPIO_PIN_RESET)
                    {
                        camera[i].status = CAM_OFF;
                        camera[i].fb = 1;
                        camera[i].led = 1;
                        camera[i].led_tick = HAL_GetTick();
                        HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_SET);
                    }
                    else if((HAL_GetTick()-camTick) >= CAM_TIMEOUT)
                    {
                        camera[i].status = CAM_OFF;
                        camera[i].led = 0;
                        camera[i].fb = 0;       // timeout
                        HAL_GPIO_WritePin(camera[i].output_gpio, camera[i].output_pin, GPIO_PIN_SET);
                    }
                }

                if(camCnt >= CAMNUM)    // all camera off
                {
                    sysMode = 0x30;
                }
                break;
            
            // Calculate camera delay time
            case 0x30:
                if(fileOpened)
                {   
                    uint8_t size_char = 0;
                    ledMode = LED_FLASH_DIM;
                    memset(fileBuf,0,sizeof(fileBuf));
                    sprintf(fileBuf,
                    #ifdef FUNC_CAM
                    "%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.1f,%1d,%1d,%1d,%1d,%1d\r\n"
                    #else
                    "%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.1f\r\n",
                    #endif
                    ,++seqID
                    ,gpsMsg.year, gpsMsg.month, gpsMsg.day, gpsMsg.hour, gpsMsg.minute, gpsMsg.second
                    ,gpsMsg.latitude, gpsMsg.longitude, gpsMsg.height
                    #ifdef FUNC_CAM
                    ,camera[0].fb, camera[1].fb, camera[2].fb, camera[3].fb, camera[4].fb
                    #endif
                    );
                    
                    while(fileBuf[size_char]!='\0'&&size_char<=70)  size_char++;
                    retSD = f_write(&fil, fileBuf, size_char, (void *)&fileBytes);
                    retSD = f_sync(&fil);
                    printf("\r\n[MSG] %s",fileBuf);
                }
                else
                {
                    //File not opened yet
                    printf("Feedback: %1d,%1d,%1d,%1d,%1d\r\n",camera[0].fb, camera[1].fb, camera[2].fb, camera[3].fb, camera[4].fb);
                }

                for(int i=0; i<CAMNUM; i++)
                {
                    camera[i].fb = 0;
                }

                camTick = HAL_GetTick();
                sysMode = 0x31;
                break;

            // Waiting for PWM signal disapper to avoid trigger continously
            case 0x31:
                #ifdef TRIG_MODE_PWM
                //if(!ISTRIGGED(uwTest))
                if(!ISTRIGGED(uwDiffCapture))
                {
                    //printf("  PWM OUT:%d\r\n",uwTest);
                    //printf("OUT:%d",uwDiffCapture);
                    sysMode = 0x20;
                }
                #endif
                #ifdef TRIG_MODE_FALLING
                trigFlag = 0x00;
                sysMode = 0x20;
                #endif
                break;

            default:
                // sysMode = 0x20;
                break;        
        }

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


        // /* ========== File Recorder ========== */
        // switch(trigFlag)
        // {
        //     case 0x00:
        //         if(HAL_GetTick()-trigTick > TRIG_GUARD_TIME)
        //         {
        //             #ifdef TRIG_MODE_PWM
        //             if((uwDiffCapture>(TRIG_PWM_MIN-50))&&(uwDiffCapture<(TRIG_PWM_MAX+50)))    trigFlag = 0x01;
        //             #endif
        //         }
        //         if(trigFlag!=0x01)  break;
            
        //     case 0x01:
        //         if(fileOpened)
        //         {
        //             ledMode = LED_FLASH_DIM;
        //             memset(fileBuf,0,sizeof(fileBuf));
        //             sprintf(fileBuf,"\r\n%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.1f",++seqID,
        //             gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
        //             gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
        //             retSD = f_write(&fil, fileBuf, sizeof(fileBuf), (void *)&fileBytes);
        //             retSD = f_sync(&fil);
        //             printf("%s",fileBuf);
        //         }
        //         trigTick = HAL_GetTick();
        //         trigFlag = 0x02;
        //         break;
            
        //     case 0x02:
        //         #ifdef TRIG_MODE_PWM
        //         if(uwDiffCapture<(TRIG_PWM_MIN-50))  trigFlag = 0x00;
        //         #endif
        //         #ifdef TRIG_MODE_FALLING
        //         trigFlag = 0x00;
        //         #endif
        //         break;

        //     default:
        //         trigFlag = 0x00;
        //         break;

        // }

        /* ========== LED ========== */
        
        if(HAL_GetTick()%1000==0)
            printf("\r\n[SYS] %d: <LED>%x, <GPS>%d, <File>%d \r\n",HAL_GetTick()/1000,ledMode,(int)gpsTick,fileOpened);
        
        switch(ledMode)
        {            
            case LED_ALWAYS_OFF:
                LED_OFF();
                HAL_GPIO_WritePin(EXT_LED_GPIO, EXT_LED_PIN, GPIO_PIN_RESET);
                break;
            
            case LED_ALWAYS_ON:
                LED_ON();
                HAL_GPIO_WritePin(EXT_LED_GPIO, EXT_LED_PIN, GPIO_PIN_SET);
                break;
            
            case LED_TWINKLE_LONG:
                if(HAL_GetTick()-ledTick>1000)
                {
                    LED_TOG();
                    HAL_GPIO_TogglePin(EXT_LED_GPIO, EXT_LED_PIN);
                    ledTick = HAL_GetTick();
                }
                break;
                
            case LED_TWINKLE_SHORT:
                if(HAL_GetTick()-ledTick>100)
                {
                    LED_TOG();
                    HAL_GPIO_TogglePin(EXT_LED_GPIO, EXT_LED_PIN);
                    ledTick = HAL_GetTick();
                }
                break;
                
            case LED_TWINKLE_MID:
                if(HAL_GetTick()-ledTick>500)
                {
                    LED_TOG();
                    HAL_GPIO_TogglePin(EXT_LED_GPIO, EXT_LED_PIN);
                    ledTick = HAL_GetTick();
                }
                break;
            
            case LED_FLASH_DIM:
                LED_OFF();
                HAL_GPIO_WritePin(EXT_LED_GPIO, EXT_LED_PIN, GPIO_PIN_RESET);
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
        
        // LED for camera
        #ifdef FUNC_CAM
        for(int i = 0; i < CAMNUM; i++)
        {
            if(camera[i].led)
            {
                if(HAL_GetTick()-camera[i].led_tick > LED_DURATION)
                {
                    camera[i].led = 0;
                }
                else
                {
                    HAL_GPIO_WritePin(camera[i].led_gpio, camera[i].led_pin, GPIO_PIN_RESET);
                }  
            }
            else
            {
                HAL_GPIO_WritePin(camera[i].led_gpio, camera[i].led_pin, GPIO_PIN_SET);
            }
        }
        #endif /* FUNC_CAM */
        
        

        /* ========== KEY ========== */
        if(keyState != HAL_GPIO_ReadPin(KEY_GPIO,KEY_PIN))
        {
            keyState = HAL_GPIO_ReadPin(KEY_GPIO,KEY_PIN);
            
            if(keyState == GPIO_PIN_RESET)
            {
                uwTest = 1500;
                printf("  Key reset.\r\n");
            }
            else
            {
                uwTest = 1800;
                printf("  Key pressed.\r\n");
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
        if(trigFlag = 0x00) trigFlag = 0x01;
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
