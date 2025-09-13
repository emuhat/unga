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
#define TIM_NUM 2     ///< Timer number
#define DMA_SIZE_WORD ///< DMA Memory Data Width: {.._BYTE, .._HWORD, .._WORD}

// LED_BATT settings
#define LEDB_TIM_CH TIM_CHANNEL_1     ///< Timer's PWM channel
#define LEDB_DMA_HANDLE hdma_tim2_ch1 ///< DMA Channel
#define LEDB_NUM_PIXELS 3             ///< Pixel quantity

// LED_MAIN settings
#define LEDM_TIM_CH TIM_CHANNEL_2         ///< Timer's PWM channel
#define LEDM_DMA_HANDLE hdma_tim2_ch2_ch4 ///< DMA Channel
#define LEDM_NUM_PIXELS 8                 ///< Pixel quantity

/**
 * @addtogroup Global_entities
 * @brief All driver's methods
 * @{
 * @enum LED_STATE
 * @brief Driver's status enum
 */
typedef enum LED_STATE {
  LED_BUSY = 0,      ///< DMA Transfer in progress
  LED_READY = 1,     ///< DMA Ready to transfer
  LED_OK = 2,        ///< Function execution success
  LED_PARAM_ERR = 3, ///< Error in input parameters
} LED_STATE;

// LEDB functions

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

LED_STATE LEDB_Ready(); // Get DMA Ready state
LED_STATE LEDB_Show();  // Push data to the strip

// LEDM functions

void LEDM_Init();  // Initialization
void LEDM_Clear(); // Clear strip

void LEDM_SetBrightness(u8_t br); // Set global brightness

void LEDM_SetRGB(u16_t i, u8_t r, u8_t g,
                 u8_t b); // Set single LED by RGB
void LEDM_SetHSV(u16_t i, u8_t hue, u8_t sat,
                 u8_t val); // Set single LED by HSV
void LEDM_SetWhite(u16_t i,
                   u8_t w); // Set white component in LED (RGBW)

void LEDM_FillRGB(u8_t r, u8_t g,
                  u8_t b); // Fill all strip with RGB color
void LEDM_FillHSV(u8_t hue, u8_t sat,
                  u8_t val); // Fill all strip with HSV color
void LEDM_FillWhite(u8_t w); // Fill all strip's white component (RGBW)

LED_STATE LEDM_Ready(); // Get DMA Ready state
LED_STATE LEDM_Show();  // Push data to the strip

/// @} @}
#endif /* LEDB_H_ */
