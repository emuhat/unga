#include "nsp.h"
#include "serial_read.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

const uint8_t PING_TYPE = 0;
const uint8_t PONG_TYPE = 1;
const uint8_t STRING_TYPE = 2;
const uint8_t TOGGLE_LIGHT_TYPE = 3;
const uint8_t VOLUME_UP_TYPE = 4;
const uint8_t VOLUME_DOWN_TYPE = 5;

#define MAX_PACKET_SIZE 512

// 6 byte header:
// three ident bytes
// 1 byte command
// N byte payload based on command
const uint8_t IDENT_BYTES[] = {'E', 'P', 'T'};
const size_t IDENT_LEN = sizeof(IDENT_BYTES) / sizeof(IDENT_BYTES[0]);

const uint32_t HEADER_BYTES = 6;
const uint32_t USER_0_POS = 4;
const uint32_t USER_1_POS = 5;
const uint32_t COMMAND_POS = 3;

const int MODE_SCANNING_FOR_HEADER = 0;
const int MODE_RECEIVING_HEADER = 1;
const int MODE_RECEIVING_PAYLOAD = 2;

void nsp_set_for_header_scan(struct NSPData *nsp_data) {
  nsp_data->recv_mode = MODE_SCANNING_FOR_HEADER;
  nsp_data->num_bytes_to_read = 1; // 1 byte at a time in scan mode
  nsp_data->packet_start = nsp_data->sr->read_ptr;
}

void nsp_set_for_header_receive(struct NSPData *nsp_data) {
  nsp_data->recv_mode = MODE_RECEIVING_HEADER;
  nsp_data->num_bytes_to_read = HEADER_BYTES; // Looking for the whole header
  nsp_data->packet_start = nsp_data->sr->read_ptr;
}

void nsp_init(struct NSPData *nsp, struct SerialRead *sr) {
  nsp->sr = sr;
  nsp_set_for_header_scan(nsp);
}

uint16_t nsp_packet_start(uint8_t *buffer, uint8_t ptype, uint8_t u0,
                          uint8_t u1) {
  uint16_t write_ptr = 0;

  buffer[write_ptr++] = IDENT_BYTES[0];
  buffer[write_ptr++] = IDENT_BYTES[1];
  buffer[write_ptr++] = IDENT_BYTES[2];
  buffer[write_ptr++] = ptype; // command type

  // user bytes
  buffer[write_ptr++] = u0;
  buffer[write_ptr++] = u1;

  return write_ptr;
}

void nsp_send_packet(uint8_t *buffer, uint16_t len) {
  HAL_UART_Transmit(&huart1, buffer, len, 0xFFFF);
}

void nsp_send_ping_packet(struct NSPData *nsp_data) {
  uint16_t write_ptr = nsp_packet_start(nsp_data->tx_buffer, PING_TYPE, 0, 0);
  nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

void nsp_send_pong_packet(struct NSPData *nsp_data) {
  uint16_t write_ptr = nsp_packet_start(nsp_data->tx_buffer, PONG_TYPE, 0, 0);
  nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

void nsp_send_volume_up(struct NSPData *nsp_data) {
  uint16_t write_ptr =
      nsp_packet_start(nsp_data->tx_buffer, VOLUME_UP_TYPE, 0, 0);
  nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

void nsp_send_volume_down(struct NSPData *nsp_data) {
  uint16_t write_ptr =
      nsp_packet_start(nsp_data->tx_buffer, VOLUME_DOWN_TYPE, 0, 0);
  nsp_send_packet(nsp_data->tx_buffer, write_ptr);
}

void nsp_print(struct NSPData *nsp_data, const char *fmt, ...) {
  char buf[256]; // adjust size to your needs
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n > 0) {
    uint8_t u0 = (n & 0xff);
    uint8_t u1 = (n >> 8);

    nsp_packet_start(nsp_data->tx_buffer, STRING_TYPE, u0, u1);
    strcpy((char *)nsp_data->tx_buffer + HEADER_BYTES, buf);
    nsp_send_packet(nsp_data->tx_buffer, HEADER_BYTES + n);
  }
}

void nsp_dispatch(struct NSPData *nsp) {
  //  nsp_print(nsp, "maybe dispatch a packet!!");
  struct SerialRead *sr = nsp->sr;

  int safe_cmd_offset = sr_add_offset(sr, nsp->packet_start, COMMAND_POS);
  int command = sr->buffer[safe_cmd_offset];

  if (command == TOGGLE_LIGHT_TYPE) {
    // actually_light_stuff_up = !actually_light_stuff_up;
  }
}

void nsp_process_rx(struct NSPData *nsp) {
  struct SerialRead *sr = nsp->sr;
  while (1) {
    uint32_t bytes_avail = sr_bytes_available(sr);

    // ignore if we don't have the number of required bytes
    if (bytes_avail < nsp->num_bytes_to_read) {
      return;
    }

    //		nsp_print(nsp, "BYtesAvail %d, read_pointer = %d", bytes_avail,
    // sr->read_ptr);

    if (nsp->recv_mode == MODE_SCANNING_FOR_HEADER) {

      int good = 0;
      int byte_to_check = sr_byte_diff(
          sr, nsp->packet_start, sr->read_ptr); // which byte we're checking

      //			nsp_print(nsp, "bc = %d", byte_check);

      if (byte_to_check < IDENT_LEN) {
        //				nsp_print(nsp, "bc = %d", byte_check);
        //				nsp_print(nsp, "exp = %d",
        // IDENT_BYTES[byte_check]); 				nsp_print(nsp,
        // "got = %d", sr->buffer[sr->read_ptr]);

        if (IDENT_BYTES[byte_to_check] == sr->buffer[sr->read_ptr]) {
//          nsp_print(nsp, "awesome");
          good = 1;
        }

        sr_advance_read_ptr(sr, 1); // move to next byte no matter what
      }

      // huzzah, we reached the end and successfully found all the ident bytes!
      else if (byte_to_check == IDENT_LEN) {
        nsp_print(nsp, "nsp synced");
        nsp->recv_mode = MODE_RECEIVING_HEADER;
        nsp->num_bytes_to_read =
            HEADER_BYTES -
            IDENT_LEN; // set up to receive the rest of the header
        good = 1;

        // Not advancing the read pointer here; it's already at the right spot
      }

      if (!good) {
        nsp_print(nsp, "not good!!");
        nsp_set_for_header_scan(nsp);
      }
    }

    else if (nsp->recv_mode == MODE_RECEIVING_HEADER) {

      // check here to see if ident matches
      int good = 1;
      for (int i = 0; i < IDENT_LEN; ++i) {
        int safe_off = sr_add_offset(sr, nsp->packet_start, i);
        //				nsp_print(nsp, "SafeOff %d", safe_off);
        if (IDENT_BYTES[i] != sr->buffer[safe_off]) {
          good = 0; // whuh oh, ident doesn't match
          break;
        }
      }

      if (good) {
        nsp_dispatch(nsp);

        // skip past the rest of the header
        sr_advance_read_ptr(sr, nsp->num_bytes_to_read);

        // set up to receive the next header
        nsp_set_for_header_receive(nsp);

        //				nsp_print(nsp, "set up for header recv
        // at %d", sr->read_ptr);
      }

      // jump to next byte and start scanning
      else {
        sr_advance_read_ptr(sr, nsp->num_bytes_to_read);
        nsp_set_for_header_scan(nsp);
      }
    }
  }
}
