/**
  ******************************************************************************
  * @file           : gpio.c
  * @brief          : Driver for GPIO (EXTI)
  ******************************************************************************
  * @Version        : 1.3(200324)
  * @Author         : Myron Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

#if (!defined BOARD_v1) && (!defined BOARD_v2) 
#error Not defined board version!
#endif

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* KEY */
    GPIO_InitStruct.Pin = KEY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO, &GPIO_InitStruct);

    /* LED */
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);

    /* Camera Control Output */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = CAM1_OUT_PIN;
    HAL_GPIO_Init(CAM1_OUT_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM2_OUT_PIN;
    HAL_GPIO_Init(CAM2_OUT_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM3_OUT_PIN;
    HAL_GPIO_Init(CAM3_OUT_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM4_OUT_PIN;
    HAL_GPIO_Init(CAM4_OUT_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM5_OUT_PIN;
    HAL_GPIO_Init(CAM5_OUT_GPIO, &GPIO_InitStruct);

    /* Camera Control Input */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = CAM1_IN_PIN;
    HAL_GPIO_Init(CAM1_IN_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM2_IN_PIN;
    HAL_GPIO_Init(CAM2_IN_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM3_IN_PIN;
    HAL_GPIO_Init(CAM3_IN_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM4_IN_PIN;
    HAL_GPIO_Init(CAM4_IN_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CAM5_IN_PIN;
    HAL_GPIO_Init(CAM5_IN_GPIO, &GPIO_InitStruct);

    /* Camera Control Input for record GPS msg */
    GPIO_InitStruct.Pin = CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CTRL_GPIO, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CTRL_IRQn, 7, 1);
    HAL_NVIC_EnableIRQ(CTRL_IRQn);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
