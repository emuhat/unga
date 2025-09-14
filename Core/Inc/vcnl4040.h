#ifndef VCNL4040_H
#define VCNL4040_H

#include "stm32l4xx_hal.h"

// I2C Address for VCNL4040
#define VCNL4040_I2C_ADDR (0x13 << 1) // 7-bit address shifted for STM32 HAL

// VCNL4040 Register Map
#define VCNL4040_REG_ALS_CONF 0x00 // Ambient Light Sensor Configuration
#define VCNL4040_REG_ALS_DATA 0x85 // Ambient Light Sensor Data
#define VCNL4040_REG_PS_CONF 0x03  // Proximity Sensor Configuration
#define VCNL4040_REG_PS_DATA 0x08  // Proximity Sensor Data

/** Registers */
#define VCNL4010_COMMAND 0x80          ///< Command
#define VCNL4010_PRODUCTID 0x81        ///< Product ID Revision
#define VCNL4010_PROXRATE 0x82         ///< Proximity rate
#define VCNL4010_IRLED 0x83            ///< IR LED current
#define VCNL4010_AMBIENTPARAMETER 0x84 ///< Ambient light parameter
#define VCNL4010_AMBIENTDATA 0x85      ///< Ambient light result (16 bits)
#define VCNL4010_PROXIMITYDATA 0x87    ///< Proximity result (16 bits)
#define VCNL4010_INTCONTROL 0x89       ///< Interrupt control
#define VCNL4010_LOWTHRESHOLD 0x8A     ///< Low threshold value (16 bits)
#define VCNL4010_HITHRESHOLD 0x8C      ///< High threshold value (16 bits)
#define VCNL4010_INTSTAT 0x8E          ///< Interrupt status
#define VCNL4010_MODTIMING 0x8F ///< Proximity modulator timing adjustment

/** Proximity measurement rate */
typedef enum {
  VCNL4010_1_95 = 0,    // 1.95     measurements/sec (Default)
  VCNL4010_3_90625 = 1, // 3.90625  measurements/sec
  VCNL4010_7_8125 = 2,  // 7.8125   measurements/sec
  VCNL4010_16_625 = 3,  // 16.625   measurements/sec
  VCNL4010_31_25 = 4,   // 31.25    measurements/sec
  VCNL4010_62_5 = 5,    // 62.5     measurements/sec
  VCNL4010_125 = 6,     // 125      measurements/sec
  VCNL4010_250 = 7,     // 250      measurements/sec
} vcnl4010_freq;

/** Values for command register */
#define VCNL4010_MEASUREPROXIMITY                                              \
  0x08 ///< Start a single on-demand proximity measurement
#define VCNL4010_MEASUREAMBIENT                                                \
  0x10 ///< Start a single on-demand ambient light measurement
#define VCNL4010_PROXIMITYREADY                                                \
  0x20 ///< Read-only - Value = 1 when proximity measurement data is available
#define VCNL4010_AMBIENTREADY                                                  \
  0x40 ///< Read-only - Value = 1 when ambient light measurement data is
       ///< available

// Register addresses
#define VCNL4040_ID_REG 0x81            // Device ID register
#define VCNL4040_AMBIENT_LIGHT_LSB 0x50 // LSB register for ambient light data

HAL_StatusTypeDef VCNL4040_Init(void);
int VCNL4040_DoProxTest(void);
int VCNL4040_DoLightTest(void);

#endif // VCNL4040_H
