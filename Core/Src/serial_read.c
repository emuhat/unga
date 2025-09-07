#include "serial_read.h"
#include <stdint.h>
#include <stdlib.h>

void sr_init(struct SerialRead *sr, uint32_t buf_size) {
  sr->buffer = (uint8_t *)malloc(buf_size);
  sr->buf_size = buf_size;
  sr->read_ptr = 0;
  sr->write_ptr = 0;
}

uint32_t sr_byte_diff(struct SerialRead *sr, int start, int end) {
  return start > end ? end + sr->buf_size - start : end - start;
}

uint32_t sr_add_offset(struct SerialRead *sr, int start, int offset) {
  return (start + offset) % sr->buf_size;
}

uint32_t sr_bytes_available(struct SerialRead *sr) {
  // if the read ptr is ahead of the write ptr, it means the write pointer has
  // wrapped
  return sr_byte_diff(sr, sr->read_ptr, sr->write_ptr);
}

void sr_advance_read_ptr(struct SerialRead *sr, uint32_t count) {

  //	nsp_print(&nsp_data, "adv: ba=%d, write is %d, read was %d, advancing by
  //%d", sr_bytes_available(sr), sr->write_ptr, sr->read_ptr, count);
  sr->read_ptr = (sr->read_ptr + count) % sr->buf_size;
}
