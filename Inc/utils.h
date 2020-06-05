/**
  ******************************************************************************
  * @file           : util.h
  * @brief          : Driver for other devices
  ******************************************************************************
  * @version        : 2.0-beta(200605)
  * @author         : Myron Xie
  ******************************************************************************
  */

#ifndef __UTILS_H
#define __UTILS_H

#include "main.h"
#include "gpio.h"

/* ========== ========== PARAMETERS ========== ========== */
/* FUNCTION SELECT */
#define     FUNC_CAM                // Enbale camera function

/* TRIGGER MODE */
//#define     TRIG_MODE_FALLING
#define     TRIG_MODE_PWM

/* TRIGGER */
#define     TRIG_PWM_MIN        1600    // (us/20ms) minimum trigger of positive width of input PWM
#define     TRIG_PWM_MAX        2000    // (us/20ms) maximum trigger of positive width of input PWM
#define     TRIG_MARGIN         20      // (us/20ms) margin for PWM width
#define     TRIG_GUARD_TIME     500     // (ms) minimum time between two triggers

/* CAMERA */
#define     CAM_START_TIME_A    5000    // (ms) time between system startup and activate all cameras
#define     CAM_START_TIME_B    3000    // (ms) time between activate all cameras and re-activate 1st camera
#define     CAM_START_TIME_C    1500    // (ms) time between re-activate each camera
#define     CAM_TRIGGED_TIME    100     // (ms) time for low-level signal to activate camera
#define     CAM_TIMEOUT         100     // (ms) Timeout for camera // due to feedback issue, this value ignored (1500 original)
/* ========== ========== ========== ========== ========== */

/* System */
#if (!defined(TRIG_MODE_FALLING)&&(!defined(TRIG_MODE_PWM)))||(defined(TRIG_MODE_FALLING)&&(defined(TRIG_MODE_PWM)))
#error "Trigger mode error!"
#endif

#if (TRIG_PWM_MIN+TRIG_MARGIN>TRIG_PWM_MAX-TRIG_MARGIN)
#error "PWM trigger params error!"
#endif


/* PWM trigger */
#define     ISTRIGGED(x)        (((x)>TRIG_PWM_MIN-TRIG_MARGIN)&&((x)<TRIG_PWM_MAX+TRIG_MARGIN))

/* LED */
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

/* Camera */
typedef struct
{
    uint8_t         status;
    GPIO_TypeDef*   output_base;
    uint16_t        output_pin;
    GPIO_TypeDef*   input_base;
    uint16_t        input_pin;
    uint8_t         fb;         // feedback
}CamType;

#define     CAM_OFF     0x00        // Camera running status
#define     CAM_ON      0x01


#endif /*__UTILS_H */


/****END OF FILE****/
