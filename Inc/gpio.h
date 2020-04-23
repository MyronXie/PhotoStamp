/**
  ******************************************************************************
  * @file           : gpio.h
  * @brief          : Header for GPIO (EXTI)
  ******************************************************************************
  * @Version        : 1.3(200324)
  * @Author         : Myron Xie
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//#define BOARD_v1
#define BOARD_v2

#ifdef BOARD_v1
#define LED_GPIO        GPIOC
#define LED_PIN         GPIO_PIN_13
#define KEY_GPIO        GPIOA
#define KEY_PIN         GPIO_PIN_15

#define CTRL_GPIO       GPIOC
#define CTRL_PIN        GPIO_PIN_3
#define CTRL_IRQn       EXTI3_IRQn

#define CAM1_OUT_GPIO   GPIOE
#define CAM1_OUT_PIN    GPIO_PIN_0
#define CAM1_IN_GPIO    GPIOE
#define CAM1_IN_PIN     GPIO_PIN_1

#define CAM2_OUT_GPIO   GPIOE
#define CAM2_OUT_PIN    GPIO_PIN_6
#define CAM2_IN_GPIO    GPIOE
#define CAM2_IN_PIN     GPIO_PIN_2

#define CAM3_OUT_GPIO   GPIOE
#define CAM3_OUT_PIN    GPIO_PIN_5
#define CAM3_IN_GPIO    GPIOE
#define CAM3_IN_PIN     GPIO_PIN_3

#define CAM4_OUT_GPIO   GPIOE
#define CAM4_OUT_PIN    GPIO_PIN_4
#define CAM4_IN_GPIO    GPIOB
#define CAM4_IN_PIN     GPIO_PIN_3

#define CAM5_OUT_GPIO   GPIOD
#define CAM5_OUT_PIN    GPIO_PIN_7
#define CAM5_IN_GPIO    GPIOD
#define CAM5_IN_PIN     GPIO_PIN_6
#endif
#ifdef BOARD_v2
#define LED_GPIO        GPIOA           //D2
#define LED_PIN         GPIO_PIN_1

#define KEY_GPIO        GPIOA           //K1
#define KEY_PIN         GPIO_PIN_0

#define CTRL_GPIO       GPIOC
#define CTRL_PIN        GPIO_PIN_3
#define CTRL_IRQn       EXTI3_IRQn

#define CAM1_OUT_GPIO   GPIOB
#define CAM1_OUT_PIN    GPIO_PIN_10
#define CAM1_IN_GPIO    GPIOE
#define CAM1_IN_PIN     GPIO_PIN_15

#define CAM2_OUT_GPIO   GPIOE
#define CAM2_OUT_PIN    GPIO_PIN_14
#define CAM2_IN_GPIO    GPIOE
#define CAM2_IN_PIN     GPIO_PIN_13

#define CAM3_OUT_GPIO   GPIOE
#define CAM3_OUT_PIN    GPIO_PIN_12
#define CAM3_IN_GPIO    GPIOE
#define CAM3_IN_PIN     GPIO_PIN_11

#define CAM4_OUT_GPIO   GPIOE
#define CAM4_OUT_PIN    GPIO_PIN_10
#define CAM4_IN_GPIO    GPIOE
#define CAM4_IN_PIN     GPIO_PIN_9

#define CAM5_OUT_GPIO   GPIOE
#define CAM5_OUT_PIN    GPIO_PIN_8
#define CAM5_IN_GPIO    GPIOE
#define CAM5_IN_PIN     GPIO_PIN_7
#endif

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
