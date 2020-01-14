/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @Version        : 1.0(191112)
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

/* ===== FUNC SELECT ===== */
//#define FUNC_CAM
#define FUNC_GPS

#define INPUT_CTRL_TEST

uint8_t errorCode = 0x00;

/* ========== GPS ========== */
GPSMsgType  gpsMsg;
uint8_t     gpsStatus   = 0x00;     // bit0: onboard  bit1: received  bit2: valid
uint8_t     gpsAvlFlag  = 0;        // Whether the msg is decoded
uint8_t     gpsMode     = 0x00;
uint64_t    gpsTick     = 0;

/* ========== File ========== */
FATFS       fs;                     // Work area (file system object) for logical drive
FIL         fil;                    // file objects
char        fileName[20]="default.txt";
uint16_t    seqid = 1;
uint32_t byteswritten;
uint8_t fileOpened = 0;

/* ========== FLASH ========== */
#define startAddr 0X08010000
const uint16_t MAGICNUM = 0x1016;
uint32_t PageError = 0;
uint16_t fileNum = 0;

/* ========== USART ========== */
uint8_t     aRxBuffer = 0, bRxBuffer = 0;
uint8_t     rxFlag = 0;
uint8_t     msgBuffer[80];
char        writeBuf[80];

uint8_t     testFlag = 0;

/* ========== PWM ========== */
volatile static uint32_t uwDiffCapture = 0;     // Positive pulse width of PWM


/* ========== KEY ========== */
uint8_t keyState    = 1;



/* ========== Camera ========== */
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
CamType camera[CAMNUM] ={
                            {CAM_OFF,GPIOE,GPIO_PIN_0,GPIOE,GPIO_PIN_1,0,0,0},
                            {CAM_OFF,GPIOE,GPIO_PIN_6,GPIOE,GPIO_PIN_2,0,0,0},
                            {CAM_OFF,GPIOE,GPIO_PIN_5,GPIOE,GPIO_PIN_3,0,0,0},
                            {CAM_OFF,GPIOE,GPIO_PIN_4,GPIOB,GPIO_PIN_3,0,0,0},
                            {CAM_OFF,GPIOD,GPIO_PIN_7,GPIOD,GPIO_PIN_6,0,0,0},
                        };

uint8_t     camMode = 0x00;
uint64_t    camTick = 0;
uint8_t     camCnt  = 0;
uint16_t    maxTime = 0;


/* ========== LED ========== */
#define LED_ON()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#define LED_OFF()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
#define LED_TOG()   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
#define LED_ALWAYS_OFF      0x00
#define LED_ALWAYS_ON       0x10
#define LED_TWINKLE_LONG    0x20
#define LED_TWINKLE_SHORT   0x21
#define LED_TWINKLE_MID     0x22
#define LED_FLASH_DIM       0x30
#define LED_FLASH_DIM_WAIT  0x31
uint8_t     ledMode = 0x00 ;
uint64_t    ledTick = 0;

#ifdef INPUT_CTRL_TEST
#define INPUT_CTRL_GPIO    GPIOA
#define INPUT_CTRL_PIN     GPIO_PIN_15
#else
#define INPUT_CTRL_GPIO    GPIOC
#define INPUT_CTRL_PIN     GPIO_PIN_3
#endif

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
    MX_RTC_Init();
    MX_FATFS_Init();
    MX_TIM1_Init();
//  MX_TIM3_Init();
//  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
    //Enable USART1 & USART2
    HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
    HAL_UART_Receive_IT(&huart2, &bRxBuffer, 1);
    
    //Read File Number from FLASH
//    fileNum = *(__IO uint32_t*)(startAddr);
//    if(fileNum == MAGICNUM)
//        fileNum = *(__IO uint32_t*)(startAddr+4);
//    else fileNum = 0;   // Dummy magic word, clear cnt for file number
//    fileNum++;
//  
//    HAL_FLASH_Unlock();
//    FLASH_EraseInitTypeDef  fl_erase;
//    fl_erase.TypeErase      = FLASH_TYPEERASE_SECTORS;
//    fl_erase.Sector         = FLASH_SECTOR_4;
//    fl_erase.NbSectors      = 1;
//    fl_erase.VoltageRange   = FLASH_VOLTAGE_RANGE_3;
//    
//    HAL_FLASHEx_Erase(&fl_erase, &PageError);
//    HAL_FLASH_Program(TYPEPROGRAM_WORD, startAddr, (uint32_t)MAGICNUM);
//    HAL_FLASH_Program(TYPEPROGRAM_WORD, startAddr+4, (uint32_t)fileNum);
//    HAL_FLASH_Lock();
//    
//    fileNum = *(__IO uint32_t*)(startAddr+4);
    
    printf("\r\n***** PhotoStamp v1.0(111912) ******\r\n");
    
    // File Process
    printf("\r\n  Mount ");
    retSD = f_mount(&fs, "0:", 1);
    if(retSD)
    {
        printf("error: %d\r\n",retSD);
        errorCode = retSD;
    }
    else    printf("success!\n\n");
    
//    printf("\r\n Open : ");
//    sprintf(fileName,"LOG%04d.txt",fileNum);
//    retSD = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE);
//    if(retSD)   printf("error: %d",retSD);
//    else        printf("success!");
    
    // Start PWM monitoring
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    ledMode = LED_TWINKLE_SHORT;

    while(1)
    {
        if(errorCode)
        {
            printf("  FATAL ERROR: %d",errorCode);
            LED_OFF();
            return 0;
        }
        
        if(!fileOpened)
        {
            if(gpsStatus&0x01)
            {
                sprintf(fileName,"%02d%02d%02d%02d.txt",gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute);
                printf("\r\n  Open File: %s",fileName);
                retSD = f_open(&fil, fileName, FA_CREATE_ALWAYS | FA_WRITE);
                if(retSD)
                {
                    printf(" error: %d\r\n",retSD);
                    errorCode = retSD;
                }
                else
                {   
                    printf(" success!\r\n");
                    ledMode = LED_TWINKLE_LONG;
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
                memset(writeBuf,0,30);
                sprintf(writeBuf,"\r\n%4d,%4d,%4d,%4d,%4d",camera[0].time,camera[1].time,camera[2].time,camera[3].time,camera[4].time);
                retSD = f_write(&fil, writeBuf, sizeof(writeBuf), (void *)&byteswritten);
                retSD = f_sync(&fil);
                printf("%s",writeBuf);
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
        if(gpsAvlFlag)
        {
            memset(msgBuffer,0,80);
            memcpy(msgBuffer,recvBuffer,gpsAvlFlag);
            //printf("%s",msgBuffer);
            gpsStatus = GPS_Decode((char*)msgBuffer,&gpsMsg,gpsAvlFlag);
            gpsAvlFlag = 0;
        }
        
        // Write gps msg
        switch(gpsMode)
        {
            // Waiting
            case 0x00:
                if(gpsStatus&0x10)
                {
                    gpsTick = HAL_GetTick();
                    if(ledMode != LED_FLASH_DIM_WAIT) ledMode = LED_ALWAYS_ON;
                }
                else
                {
                    if(HAL_GetTick()-gpsTick>3000)  // GPS Lost
                    {
                        ledMode = LED_TWINKLE_MID;
                    }
                }
            
                if(HAL_GPIO_ReadPin(INPUT_CTRL_GPIO, INPUT_CTRL_PIN)==GPIO_PIN_RESET)
                {
                    gpsMode = 0x01;
                    ledMode = LED_FLASH_DIM;
                }
                
                if(testFlag == 1)
                {
                    gpsMode = 0x01;
                    ledMode = LED_FLASH_DIM;
                }
                break;
            
            // Writing
            case 0x01:
                if(fileOpened)
                {
                    memset(writeBuf,0,100);
                    sprintf(writeBuf,"%d,%04d%02d%02d,%02d%02d%02d,%.7f,%.7f,%.2f\r\n",seqid++,
                    gpsMsg.year,gpsMsg.month,gpsMsg.day,gpsMsg.hour,gpsMsg.minute,gpsMsg.second,
                    gpsMsg.latitude,gpsMsg.longitude,gpsMsg.height);
                    retSD = f_write(&fil, writeBuf, sizeof(writeBuf), (void *)&byteswritten);
                    retSD = f_sync(&fil);
                    printf("%s",writeBuf);
                }
                gpsMode = 0x02;
                break;
            
            case 0x02:
                if(HAL_GPIO_ReadPin(INPUT_CTRL_GPIO, INPUT_CTRL_PIN)==GPIO_PIN_SET)
                {
                    gpsMode = 0x00;
                }
                if(testFlag == 1)
                {
                    gpsMode = 0x00;
                    testFlag = 0;
                }
                break;
            
            default:
                gpsMode = 0x00;
                break;
        }

        #endif /* FUNC_GPS */

        /* ========== LED ========== */
        
        if(HAL_GetTick()%1000==0)   printf("\r\nLED_MODE: %x, %d, %d\r\n",ledMode,(int)gpsTick,HAL_GetTick());
        
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
                if(HAL_GetTick()-ledTick>250)
                {
                    ledMode = LED_ALWAYS_ON;
                }
                break;
                
            default:
                ledMode = LED_ALWAYS_OFF;
                break;
        }


        /* ========== KEY ========== */
        if(keyState!=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
        {
            keyState = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);
            
            if(keyState == GPIO_PIN_SET)
            {
                //printf("\r\n Key reset.");
            }
            else if(keyState == GPIO_PIN_RESET)
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        //HAL_UART_Transmit(&huart1, &aRxBuffer, 1, 1); // [DEBUG] test gps recv message
        
        gpsAvlFlag = GPS_MsgRecv(aRxBuffer);
//        if(gpsAvlFlag)
//        {
//            memset(msgBuffer,0,80);
//            memcpy(msgBuffer,recvBuffer,gpsAvlFlag);
//        }
        HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
    }
    
    if(huart->Instance == USART1)
    {
        HAL_UART_Transmit(&huart1, &bRxBuffer, 1, 1);
        HAL_UART_Receive_IT(&huart1, &bRxBuffer, 1);
        if(bRxBuffer=='!')
        {
            if(testFlag == 0)   testFlag = 1;
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    volatile static uint32_t uwIC1Value1 = 0, uwIC1Value2 = 0;
    static uint8_t tim_flag = 0;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_SET)
        {
            uwIC1Value1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            tim_flag = 0x01;
        }
        else
        {
            uwIC1Value2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            if(tim_flag==0x01)  tim_flag = 0x02;
            else tim_flag =0x00;
        }
        if(tim_flag==0x02)
        {
            if(uwIC1Value2 > uwIC1Value1)
                uwDiffCapture = uwIC1Value2 - uwIC1Value1; 
            else
                uwDiffCapture = (__HAL_TIM_GET_AUTORELOAD(&htim1) -uwIC1Value1 + 1) + uwIC1Value2;
            //printf("%d,%d,%d\r\n",uwIC1Value1,uwIC1Value2,uwDiffCapture);  // [debug] test input pwm duty

        }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
