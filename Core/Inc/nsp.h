#ifndef NSP_H
#define NSP_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

struct SerialRead;

#define BUFFER_SIZE 512

struct NSPData {
  struct SerialRead *sr;
  uint8_t tx_buffer[BUFFER_SIZE];
  int packet_start;
  int num_bytes_to_read; // starting from the packet_start_ptr
  int recv_mode;
};

void nsp_init(struct NSPData *nsp, struct SerialRead *sr);
void nsp_send_ping_packet(struct NSPData *nsp_data);
void nsp_send_pong_packet(struct NSPData *nsp_data);
void nsp_process_rx(struct NSPData *nsp);
void nsp_print(struct NSPData *nsp_data, const char *fmt, ...);

#endif
