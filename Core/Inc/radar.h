#ifndef RADAR_H
#define RADAR_H

#include "nsp.h"
#include "serial_read.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

struct NSPData;
struct SerialRead;

#define BUFFER_SIZE 512

struct RadarData {
  UART_HandleTypeDef *uart_handle;
  struct NSPData *nsp;
  struct SerialRead sr;
  uint8_t tx_buffer[BUFFER_SIZE];
};

void radar_init(struct RadarData *radar, struct NSPData *nsp,
                UART_HandleTypeDef *uart_handle);

#endif
