#include "vcnl4040.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

// https://github.com/adafruit/Adafruit_VCNL4010/blob/master/Adafruit_VCNL4010.cpp
// https://github.com/adafruit/Adafruit_VCNL4010/blob/master/Adafruit_VCNL4010.h
// https://github.com/adafruit/Adafruit_CircuitPython_VCNL4010/blob/main/adafruit_vcnl4010.py

// I2C Handle and VCNL4040 Address
extern I2C_HandleTypeDef hi2c1; // Replace with your I2C handle

int hepcat = 551;

HAL_StatusTypeDef write8(uint8_t address, uint8_t data) {
  uint8_t buffer[2] = {address, data};
  return HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_I2C_ADDR, buffer, 2,
                                 HAL_MAX_DELAY);
}

int read8(uint8_t reg, uint8_t *value) {
  uint8_t data;
  if (HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_I2C_ADDR, &reg, 1,
                              HAL_MAX_DELAY) != HAL_OK) {
    return 11;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, VCNL4040_I2C_ADDR, &data, 1,
                             HAL_MAX_DELAY) != HAL_OK) {
    return 22;
  }
  *value = data;
  //*value = (data[1] << 8) | data[0]; // Combine high and low bytes
  return data;
}

int read16(uint8_t reg, uint16_t *value) {
  uint8_t data[2] = {0, 0};
  if (HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_I2C_ADDR, &reg, 1,
                              HAL_MAX_DELAY) != HAL_OK) {
    return 11;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, VCNL4040_I2C_ADDR, data, 2,
                             HAL_MAX_DELAY) != HAL_OK) {
    return 22;
  }
  //*value = data;
  *value = ((uint16_t)(data[0]) << 8) | (uint16_t)(data[1]);
  return *value;
}

HAL_StatusTypeDef setLEDcurrent(uint8_t current_10mA) {
  if (current_10mA > 20)
    current_10mA = 20;
  return write8(VCNL4010_IRLED, current_10mA);
}

HAL_StatusTypeDef setFrequency(vcnl4010_freq freq) {
  return write8(VCNL4010_PROXRATE, freq);
}

// Initialize VCNL4040
HAL_StatusTypeDef VCNL4040_Init(void) {

  uint8_t data = 0;
  hepcat = 0;

  // Check the product ID
  if (read8(VCNL4010_PRODUCTID, &data) == HAL_ERROR) {
    return HAL_ERROR;
  }
  if ((data & 0xF0) != 0x20) {
    return HAL_ERROR;
  }

  if (setLEDcurrent(20) != HAL_OK) {
    return HAL_ERROR;
  }

  // 16.625 readings/second
  if (setFrequency(VCNL4010_16_625) != HAL_OK) {
    return HAL_ERROR;
  }

  if (write8(VCNL4010_INTCONTROL, 0x08) != HAL_OK) {
    return HAL_ERROR;
  }

  hepcat = 42;

  //    HAL_StatusTypeDef status;
  // Set ALS Configuration: Example - Enable ALS, continuous mode
  // status = VCNL4040_WriteRegister(VCNL4040_REG_ALS_CONF, 0x000D); // Modify
  // as per requirements if (status != HAL_OK) return status;

  // Set Proximity Configuration: Example - Enable PS, active force mode
  // status = VCNL4040_WriteRegister(VCNL4040_REG_PS_CONF, 0x030D); // Modify as
  // per requirements return status;
  return HAL_OK;
}

int VCNL4040_DoProxTest(void) {
  uint8_t i = 0;
  read8(VCNL4010_INTSTAT, &i);
  i &= ~0x80;
  write8(VCNL4010_INTSTAT, i);

  write8(VCNL4010_COMMAND, VCNL4010_MEASUREPROXIMITY);
  for (int j = 0; j < 5; ++j) {
    // Serial.println(read8(VCNL4010_INTSTAT), HEX);
    uint8_t result = 0;
    result = read8(VCNL4010_COMMAND, &result);

    // Serial.print("Ready = 0x"); Serial.println(result, HEX);
    //	    if (result & VCNL4010_PROXIMITYREADY) {
    //	      return read16(VCNL4010_PROXIMITYDATA);
    //	    }

    if (result & VCNL4010_PROXIMITYREADY) {

      uint16_t res = 0;

      res = read16(VCNL4010_PROXIMITYDATA, &res);
      return res;
    }

    HAL_Delay(1);
  }

  return 666;
}

int VCNL4040_DoLightTest(void) {
  uint8_t i = 0;
  read8(VCNL4010_INTSTAT, &i);
  i &= ~0x40;
  //	  i &= ~0x80;
  write8(VCNL4010_INTSTAT, i);

  write8(VCNL4010_COMMAND, VCNL4010_MEASUREAMBIENT);
  for (int j = 0; j < 100; ++j) {

    uint8_t result = 0;
    result = read8(VCNL4010_COMMAND, &result);

    if (result & VCNL4010_AMBIENTREADY) {
      uint16_t res = 0;
      res = read16(VCNL4010_AMBIENTDATA, &res);
      return res / 4;
    }

    HAL_Delay(1);
  }

  return 666;
}
