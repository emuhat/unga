#ifndef SERIAL_READ_H_
#define SERIAL_READ_H_

#include <stdint.h>

#define UART_RX_BUFFER_SIZE 40

struct SerialRead {
  uint8_t *buffer;
  uint32_t buf_size;
  int read_ptr; // the latest we've processed
  int write_ptr;
};

void sr_advance_read_ptr(struct SerialRead *sr, uint32_t count);
uint32_t sr_bytes_available(struct SerialRead *sr);
uint32_t sr_add_offset(struct SerialRead *sr, int start, int offset);
uint32_t sr_byte_diff(struct SerialRead *sr, int start, int end);
void sr_init(struct SerialRead *sr, uint32_t buf_size);

#endif
