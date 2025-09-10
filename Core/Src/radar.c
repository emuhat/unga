#include "radar.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void radar_init(struct RadarData *radar, struct NSPData *nsp,
                UART_HandleTypeDef *uart_handle) {
  //  sr_init(&nsp->sr, uart_handle, UART_RX_BUFFER_SIZE);
  nsp_print(nsp, "Radar Init!");
}
