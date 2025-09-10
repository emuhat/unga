#ifndef RADAR_H
#define RADAR_H

#include "serial_read.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

struct SerialRead;

#define BUFFER_SIZE 512

struct RadarData {
  struct SerialRead sr;
  uint8_t tx_buffer[BUFFER_SIZE];
};

void radar_init(struct RadarData *nsp);

#endif
