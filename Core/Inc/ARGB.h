/**
 *******************************************
 * @file    ARGB.h
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @link    https://crazygeeks.ru
 * @version 1.33
 * @date	17-May-2022
 * @brief   Header file for ARGB Driver (Addressable RGB)
 *******************************************
 *
 * @note Repo: https://github.com/Crazy-Geeks/STM32-ARGB-DMA
 * @note RU article: https://crazygeeks.ru/stm32-argb-lib
 */

// https://cdn-shop.adafruit.com/product-files/2757/p2757_SK6812RGBW_REV01.pdf

#ifndef LEDB_H_
#define LEDB_H_

#include "libs.h"

/**
 * @addtogroup LEDB_Driver
 * @brief Addressable RGB LED Driver
 * @{
 * @addtogroup User_settings
 * @brief LED & Timer's settings
 * @{
 */


#define SK6812
#define USE_GAMMA_CORRECTION 1
#define TIM_NUM 2 ///< Timer number
#define DMA_SIZE_WORD ///< DMA Memory Data Width: {.._BYTE, .._HWORD, .._WORD}



// LED_BATT settings
 #define LEDB_TIM_CH TIM_CHANNEL_1     ///< Timer's PWM channel
 #define LEDB_DMA_HANDLE hdma_tim2_ch1 ///< DMA Channel
 #define LEDB_NUM_PIXELS 3 ///< Pixel quantity

// LED_MAIN settings
//#define TIM_CH TIM_CHANNEL_2         ///< Timer's PWM channel
//#define DMA_HANDLE hdma_tim2_ch2_ch4 ///< DMA Channel
//#define NUM_PIXELS 3 ///< Pixel quantity

/**
 * @addtogroup Global_entities
 * @brief All driver's methods
 * @{
 * @enum LEDB_STATE
 * @brief Driver's status enum
 */
typedef enum LEDB_STATE {
  LEDB_BUSY = 0,      ///< DMA Transfer in progress
  LEDB_READY = 1,     ///< DMA Ready to transfer
  LEDB_OK = 2,        ///< Function execution success
  LEDB_PARAM_ERR = 3, ///< Error in input parameters
} LEDB_STATE;

void LEDB_Init();  // Initialization
void LEDB_Clear(); // Clear strip

void LEDB_SetBrightness(u8_t br); // Set global brightness

void LEDB_SetRGB(u16_t i, u8_t r, u8_t g,
                 u8_t b); // Set single LED by RGB
void LEDB_SetHSV(u16_t i, u8_t hue, u8_t sat,
                 u8_t val); // Set single LED by HSV
void LEDB_SetWhite(u16_t i,
                   u8_t w); // Set white component in LED (RGBW)

void LEDB_FillRGB(u8_t r, u8_t g,
                  u8_t b); // Fill all strip with RGB color
void LEDB_FillHSV(u8_t hue, u8_t sat,
                  u8_t val); // Fill all strip with HSV color
void LEDB_FillWhite(u8_t w); // Fill all strip's white component (RGBW)

LEDB_STATE LEDB_Ready(); // Get DMA Ready state
LEDB_STATE LEDB_Show();  // Push data to the strip

/// @} @}
#endif /* LEDB_H_ */
