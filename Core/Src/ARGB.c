/**
 *******************************************
 * @file    ARGB.c
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @link    https://crazygeeks.ru
 * @version 1.33
 * @date	17-May-2022
 * @brief   Source file for ARGB Driver (Addressable RGB)
 *******************************************
 *
 * @note Repo: https://github.com/Crazy-Geeks/STM32-ARGB-DMA
 * @note RU article: https://crazygeeks.ru/stm32-argb-lib
 */

/* WS2811 Timings
 * Tolerance: +/- 150ns <-> +/- 0.15us
 * RES: >50us
 *
 * Slow mode:
 * Period: 2.5us <-> 400 KHz
 * T0H: 0.5us
 * T1H: 1.2us
 * T0L: 2.0us
 * T1L: 1.3us
 *
 * Fast mode:
 * Period: 1.25us <-> 800 KHz
 * T0H: 0.25us - 20%
 * T1H: 0.6us  - 48%
 * T0L: 1.0us
 * T1H: 0.65us
 *
 */

/* WS2811 Timings
 * Tolerance: +/- 150ns <-> +/- 0.15us
 * RES: >50us

 * Period: 1.25us <-> 800 KHz
 * T0H: 0.35us - 20%
 * T1H: 0.7us  - 48%
 * T0L: 0.8us
 * T1H: 0.6us
 *
 */

#include "ARGB.h" // include header file
#include "math.h"

/**
 * @addtogroup Private_entities
 * @brief Private methods and variables
 * @{
 */

/// Timer handler
#define TIM_HANDLE htim2

/// DMA Size
typedef u32_t dma_siz;
#define PWM_BUF_LEN (4 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs

static inline u8_t scale8(u8_t x, u8_t scale); // Gamma correction
static void HSV2RGB(u8_t hue, u8_t sat, u8_t val, u8_t *_r, u8_t *_g, u8_t *_b);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern TIM_HandleTypeDef(TIM_HANDLE); ///< Timer handler
extern DMA_HandleTypeDef(LEDB_DMA_HANDLE); ///< DMA handler

volatile u8_t LEDB_PWM_HI; ///< PWM Code HI Log.1 period
volatile u8_t LEDB_PWM_LO; ///< PWM Code LO Log.1 period

#define LEDB_NUM_BYTES (4 * LEDB_NUM_PIXELS) ///< Strip size in bytes

/// Static LED buffer
volatile u8_t LEDB_RGB_BUF[LEDB_NUM_BYTES] = {
    0,
};

/// Timer PWM value buffer
volatile dma_siz LEDB_PWM_BUF[PWM_BUF_LEN] = {
    0,
};
/// PWM buffer iterator
volatile u16_t LEDB_BUF_COUNTER = 0;

volatile LEDB_STATE LEDB_LEDB_LOC_ST; ///< Buffer send status

// Callbacks
static void LEDB_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
static void LEDB_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
/// @} //Private

/**
 * @brief Init timer & prescalers
 * @param none
 */
void LEDB_Init() {

  /* Auto-calculation! */
  u32_t APBfq; // Clock freq
  APBfq = HAL_RCC_GetPCLK2Freq();
  APBfq *= (RCC->CFGR & RCC_CFGR_PPRE2) == 0 ? 1 : 2;
  APBfq /= (uint32_t)(800 * 1000);                  // 800 KHz - 1.25us
  TIM_HANDLE.Instance->PSC = 0;                     // dummy hardcode now
  TIM_HANDLE.Instance->ARR = (uint16_t)(APBfq - 1); // set timer prescaler
  TIM_HANDLE.Instance->EGR = 1;                     // update timer registers
  LEDB_PWM_HI = (u8_t)(APBfq * 0.48) - 1;                // Log.1 - 48% - 0.60us
  LEDB_PWM_LO = (u8_t)(APBfq * 0.24) - 1;                // Log.0 - 24% - 0.30us

  // #if INV_SIGNAL
  //     TIM_POINTER->CCER |= TIM_CCER_CC2P; // set inv ch bit
  // #else
  //     TIM_POINTER->CCER &= ~TIM_CCER_CC2P;
  // #endif
  LEDB_LEDB_LOC_ST = LEDB_READY; // Set Ready Flag
  TIM_CCxChannelCmd(TIM_HANDLE.Instance, LEDB_TIM_CH,
                    TIM_CCx_ENABLE); // Enable GPIO to IDLE state
  HAL_Delay(1);                      // Make some delay
}

/**
 * @brief Fill ALL LEDs with (0,0,0)
 * @param none
 * @note Update strip after that
 */
void LEDB_Clear() {
  LEDB_FillRGB(0, 0, 0);
  LEDB_FillWhite(0);
}

/**
 * @brief Set LED with RGB color by index
 * @param[in] i LED position
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void LEDB_SetRGB(u16_t i, u8_t r, u8_t g, u8_t b) {
  // overflow protection
  if (i >= LEDB_NUM_PIXELS) {
    u16_t _i = i / LEDB_NUM_PIXELS;
    i -= _i * LEDB_NUM_PIXELS;
  }
#if USE_GAMMA_CORRECTION
  g = scale8(g, 0xB0);
  b = scale8(b, 0xF0);
#endif
  // Subpixel chain order
  const u8_t subp1 = g;
  const u8_t subp2 = r;
  const u8_t subp3 = b;
  // RGB or RGBW
  LEDB_RGB_BUF[4 * i] = subp1;     // subpixel 1
  LEDB_RGB_BUF[4 * i + 1] = subp2; // subpixel 2
  LEDB_RGB_BUF[4 * i + 2] = subp3; // subpixel 3
}

/**
 * @brief Set LED with HSV color by index
 * @param[in] i LED position
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void LEDB_SetHSV(u16_t i, u8_t hue, u8_t sat, u8_t val) {
  uint8_t _r, _g, _b;                    // init buffer color
  HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get RGB color
  LEDB_SetRGB(i, _r, _g, _b);      // set color
}

/**
 * @brief Set White component in strip by index
 * @param[in] i LED position
 * @param[in] w White component [0..255]
 */
void LEDB_SetWhite(u16_t i, u8_t w) {
  LEDB_RGB_BUF[4 * i + 3] = w;          // set white part
}

/**
 * @brief Fill ALL LEDs with RGB color
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void LEDB_FillRGB(u8_t r, u8_t g, u8_t b) {
  for (volatile u16_t i = 0; i < LEDB_NUM_PIXELS; i++)
    LEDB_SetRGB(i, r, g, b);
}

/**
 * @brief Fill ALL LEDs with HSV color
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void LEDB_FillHSV(u8_t hue, u8_t sat, u8_t val) {
  uint8_t _r, _g, _b;                    // init buffer color
  HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get color once (!)
  LEDB_FillRGB(_r, _g, _b);        // set color
}

/**
 * @brief Set ALL White components in strip
 * @param[in] w White component [0..255]
 */
void LEDB_FillWhite(u8_t w) {
  for (volatile u16_t i = 0; i < LEDB_NUM_PIXELS; i++)
    LEDB_SetWhite(i, w);
}

/**
 * @brief Get current DMA status
 * @param none
 * @return #LEDB_STATE enum
 */
LEDB_STATE LEDB_Ready() { return LEDB_LEDB_LOC_ST; }

/**
 * @brief Update strip
 * @param none
 * @return #LEDB_STATE enum
 */
LEDB_STATE LEDB_Show() {
  LEDB_LEDB_LOC_ST = LEDB_BUSY;
  if (LEDB_BUF_COUNTER != 0 || LEDB_DMA_HANDLE.State != HAL_DMA_STATE_READY) {
    return LEDB_BUSY;
  } else {
    for (volatile u8_t i = 0; i < 8; i++) {
      // set first transfer from first values
      LEDB_PWM_BUF[i] = (((LEDB_RGB_BUF[0] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 8] = (((LEDB_RGB_BUF[1] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 16] = (((LEDB_RGB_BUF[2] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 24] = (((LEDB_RGB_BUF[3] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 32] = (((LEDB_RGB_BUF[4] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 40] = (((LEDB_RGB_BUF[5] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
#ifdef SK6812
      LEDB_PWM_BUF[i + 48] = (((LEDB_RGB_BUF[6] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 56] = (((LEDB_RGB_BUF[7] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
#endif
    }
    HAL_StatusTypeDef DMA_Send_Stat = HAL_ERROR;
    while (DMA_Send_Stat != HAL_OK) {
      if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, LEDB_TIM_CH) ==
          HAL_TIM_CHANNEL_STATE_BUSY) {
        DMA_Send_Stat = HAL_BUSY;
        continue;
      } else if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, LEDB_TIM_CH) ==
                 HAL_TIM_CHANNEL_STATE_READY) {
        TIM_CHANNEL_STATE_SET(&TIM_HANDLE, LEDB_TIM_CH, HAL_TIM_CHANNEL_STATE_BUSY);
      } else {
        DMA_Send_Stat = HAL_ERROR;
        continue;
      }
#if LEDB_TIM_CH == TIM_CHANNEL_1
#define LEDB_TIM_DMA_ID TIM_DMA_ID_CC1
#define LEDB_TIM_DMA_CC TIM_DMA_CC1
#define LEDB_TIM_CCR CCR1
#elif LEDB_TIM_CH == TIM_CHANNEL_2
#define LEDB_TIM_DMA_ID TIM_DMA_ID_CC2
#define LEDB_TIM_DMA_CC TIM_DMA_CC2
#define LEDB_TIM_CCR CCR2
#elif LEDB_TIM_CH == TIM_CHANNEL_3
#define LEDB_TIM_DMA_ID TIM_DMA_ID_CC3
#define LEDB_TIM_DMA_CC TIM_DMA_CC3
#define LEDB_TIM_CCR CCR3
#elif LEDB_TIM_CH == TIM_CHANNEL_4
#define LEDB_TIM_DMA_ID TIM_DMA_ID_CC4
#define LEDB_TIM_DMA_CC TIM_DMA_CC4
#define LEDB_TIM_CCR CCR4
#endif
      TIM_HANDLE.hdma[LEDB_TIM_DMA_ID]->XferCpltCallback =
          LEDB_TIM_DMADelayPulseCplt;
      TIM_HANDLE.hdma[LEDB_TIM_DMA_ID]->XferHalfCpltCallback =
          LEDB_TIM_DMADelayPulseHalfCplt;
      TIM_HANDLE.hdma[LEDB_TIM_DMA_ID]->XferErrorCallback = TIM_DMAError;
      if (HAL_DMA_Start_IT(TIM_HANDLE.hdma[LEDB_TIM_DMA_ID], (u32_t)LEDB_PWM_BUF,
                           (u32_t)&TIM_HANDLE.Instance->LEDB_TIM_CCR,
                           (u16_t)PWM_BUF_LEN) != HAL_OK) {
        DMA_Send_Stat = HAL_ERROR;
        continue;
      }
      __HAL_TIM_ENABLE_DMA(&TIM_HANDLE, LEDB_TIM_DMA_CC);
      if (IS_TIM_BREAK_INSTANCE(TIM_HANDLE.Instance) != RESET)
        __HAL_TIM_MOE_ENABLE(&TIM_HANDLE);
      if (IS_TIM_SLAVE_INSTANCE(TIM_HANDLE.Instance)) {
        u32_t tmpsmcr = TIM_HANDLE.Instance->SMCR & TIM_SMCR_SMS;
        if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
          __HAL_TIM_ENABLE(&TIM_HANDLE);
      } else
        __HAL_TIM_ENABLE(&TIM_HANDLE);
      DMA_Send_Stat = HAL_OK;
    }
    LEDB_BUF_COUNTER = 2;
    return LEDB_OK;
  }
}

/**
 * @addtogroup Private_entities
 * @{ */


static void LEDB_TIM_DMADelayPulseCpltImpl(DMA_HandleTypeDef *hdma, uint32_t channel)
{
	  TIM_HandleTypeDef *htim =
	      (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
	  // if wrong handlers
	  if (hdma != &LEDB_DMA_HANDLE || htim != &TIM_HANDLE)
	    return;
	  if (LEDB_BUF_COUNTER == 0)
	    return; // if no data to transmit - return
	  if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
	    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	    if (hdma->Init.Mode == DMA_NORMAL) {
	      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_1, HAL_TIM_CHANNEL_STATE_READY);
	    }
	  } else if (hdma == htim->hdma[TIM_DMA_ID_CC2]) {
	    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;
	    if (hdma->Init.Mode == DMA_NORMAL) {
	      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
	    }
	  } else if (hdma == htim->hdma[TIM_DMA_ID_CC3]) {
	    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;
	    if (hdma->Init.Mode == DMA_NORMAL) {
	      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_3, HAL_TIM_CHANNEL_STATE_READY);
	    }
	  } else if (hdma == htim->hdma[TIM_DMA_ID_CC4]) {
	    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;
	    if (hdma->Init.Mode == DMA_NORMAL) {
	      TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_4, HAL_TIM_CHANNEL_STATE_READY);
	    }
	  } else {
	    /* nothing to do */
	  }
	  // if data transfer
	  if (LEDB_BUF_COUNTER < LEDB_NUM_PIXELS) {
	    // fill second part of buffer
	    for (volatile u8_t i = 0; i < 8; i++) {
	      LEDB_PWM_BUF[i + 32] =
	          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
	      LEDB_PWM_BUF[i + 40] =
	          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 1] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
	      LEDB_PWM_BUF[i + 48] =
	          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 2] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
	      LEDB_PWM_BUF[i + 56] =
	          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 3] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
	    }
	    LEDB_BUF_COUNTER++;
	  } else if (LEDB_BUF_COUNTER < LEDB_NUM_PIXELS + 2) { // if RET transfer
	    memset((dma_siz *)&LEDB_PWM_BUF[PWM_BUF_LEN / 2], 0,
	           (PWM_BUF_LEN / 2) * sizeof(dma_siz)); // second part
	    LEDB_BUF_COUNTER++;
	  } else { // if END of transfer
	    LEDB_BUF_COUNTER = 0;
	    // STOP DMA:
	#if LEDB_TIM_CH == TIM_CHANNEL_1
	    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	    (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC1]);
	#endif
	#if LEDB_TIM_CH == TIM_CHANNEL_2
	    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	    (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC2]);
	#endif
	#if LEDB_TIM_CH == TIM_CHANNEL_3
	    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	    (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC3]);
	#endif
	#if LEDB_TIM_CH == TIM_CHANNEL_4
	    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	    (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC4]);
	#endif
	    if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
	      /* Disable the Main Output */
	      __HAL_TIM_MOE_DISABLE(htim);
	    }
	    /* Disable the Peripheral */
	    __HAL_TIM_DISABLE(htim);
	    /* Set the TIM channel state */
	    TIM_CHANNEL_STATE_SET(htim, LEDB_TIM_CH, HAL_TIM_CHANNEL_STATE_READY);
	    LEDB_LEDB_LOC_ST = LEDB_READY;
	  }
	  htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

/**
 * @brief  TIM DMA Delay Pulse complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void LEDB_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma) {
	LEDB_TIM_DMADelayPulseCpltImpl(hdma, LEDB_TIM_CH);
}

/**
 * @brief  TIM DMA Delay Pulse half complete callback.
 * @param  hdma pointer to DMA handle.
 * @retval None
 */
static void LEDB_TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma) {
  TIM_HandleTypeDef *htim =
      (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  // if wrong handlers
  if (hdma != &LEDB_DMA_HANDLE || htim != &TIM_HANDLE)
    return;
  if (LEDB_BUF_COUNTER == 0)
    return; // if no data to transmit - return
  // if data transfer
  if (LEDB_BUF_COUNTER < LEDB_NUM_PIXELS) {
    // fill first part of buffer
    for (volatile u8_t i = 0; i < 8; i++) {
      LEDB_PWM_BUF[i] =
          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 8] =
          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 1] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 16] =
          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 2] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
      LEDB_PWM_BUF[i + 24] =
          (((LEDB_RGB_BUF[4 * LEDB_BUF_COUNTER + 3] << i) & 0x80) > 0) ? LEDB_PWM_HI : LEDB_PWM_LO;
    }
    LEDB_BUF_COUNTER++;
  } else if (LEDB_BUF_COUNTER < LEDB_NUM_PIXELS + 2) { // if RET transfer
    memset((dma_siz *)&LEDB_PWM_BUF[0], 0,
           (PWM_BUF_LEN / 2) * sizeof(dma_siz)); // first part
    LEDB_BUF_COUNTER++;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Private method for gamma correction
 * @param[in] x Param to scale
 * @param[in] scale Scale coefficient
 * @return Scaled value
 */
static inline u8_t scale8(u8_t x, u8_t scale) {
  return ((uint16_t)x * scale) >> 8;
}

/**
 * @brief Convert color in HSV to RGB
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 * @param[out] _r Pointer to RED component value
 * @param[out] _g Pointer to GREEN component value
 * @param[out] _b Pointer to BLUE component value
 */
static void HSV2RGB(u8_t hue, u8_t sat, u8_t val, u8_t *_r, u8_t *_g,
                    u8_t *_b) {
  if (sat == 0) { // if white color
    *_r = *_g = *_b = val;
    return;
  }
  // Float is smoother but check for FPU (Floating point unit) in your MCU
  // Otherwise it will take longer time in the code
  // FPU is in: F3/L3 and greater
  // Src: https://github.com/Inseckto/HSV-to-RGB
  float h = (float)hue / 255;
  float s = (float)sat / 255;
  float v = (float)val / 255;

  int i = (int)floorf(h * 6);
  float f = h * 6 - (float)i;
  u8_t p = (u8_t)(v * (1 - s) * 255.0);
  u8_t q = (u8_t)(v * (1 - f * s) * 255.0);
  u8_t t = (u8_t)(v * (1 - (1 - f) * s) * 255.0);

  switch (i % 6) {
    // Src: https://stackoverflow.com/questions/3018313
    //    uint8_t reg = hue / 43;
    //    uint8_t rem = (hue - (reg * 43)) * 6;
    //    uint8_t p = (val * (255 - sat)) >> 8;
    //    uint8_t q = (val * (255 - ((sat * rem) >> 8))) >> 8;
    //    uint8_t t = (val * (255 - ((sat * (255 - rem)) >> 8))) >> 8;
    //    switch (reg) {
  case 0:
    *_r = val, *_g = t, *_b = p;
    break;
  case 1:
    *_r = q, *_g = val, *_b = p;
    break;
  case 2:
    *_r = p, *_g = val, *_b = t;
    break;
  case 3:
    *_r = p, *_g = q, *_b = val;
    break;
  case 4:
    *_r = t, *_g = p, *_b = val;
    break;
  default:
    *_r = val, *_g = p, *_b = q;
    break;
  }
}

