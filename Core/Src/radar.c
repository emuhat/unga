#include "radar.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void radar_write_command_preamble(uint8_t *buffer, size_t *pos) {
  buffer[(*pos)++] = 0xFD;
  buffer[(*pos)++] = 0xFC;
  buffer[(*pos)++] = 0xFB;
  buffer[(*pos)++] = 0xFA;
}

void radar_write_command_postamble(uint8_t *buffer, size_t *pos) {
  buffer[(*pos)++] = 0x04;
  buffer[(*pos)++] = 0x03;
  buffer[(*pos)++] = 0x02;
  buffer[(*pos)++] = 0x01;
}

void radar_send_firmware_request(struct RadarData *radar) {
  size_t pos = 0;

  radar_write_command_preamble(radar->tx_buffer, &pos);
  radar->tx_buffer[pos++] = 0x02; // Command is two bytes long
  radar->tx_buffer[pos++] = 0x00;
  radar->tx_buffer[pos++] = 0xA0; // Request firmware version
  radar->tx_buffer[pos++] = 0x00;
  radar_write_command_preamble(radar->tx_buffer, &pos);

  // Send it
  HAL_UART_Transmit(radar->uart_handle, radar->tx_buffer, pos, 0xFFFF);
}

void radar_init(struct RadarData *radar, struct NSPData *nsp,
                UART_HandleTypeDef *uart_handle) {

  sr_init(&radar->sr, uart_handle, UART_RX_BUFFER_SIZE);

  radar->uart_handle = uart_handle;
  radar->nsp = nsp;

  nsp_print(radar->nsp, "Radar Init!");

  sr_recv_to_idle(&radar->sr);

  HAL_Delay(500);

  radar_send_firmware_request(radar);
}

#if 0
/*
 * A C library for the Hi-Link LD2410 24Ghz FMCW radar sensor.
 *
 * This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection
 * and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 * The code in this library is based off the manufacturer datasheet and reading of this initial piece of work
 * for ESPHome https://github.com/rain931215/ESPHome-LD2410.
 *
 * https://github.com/ncmreynolds/ld2410
 *
 * Released under LGPL-2.1 see https://github.com/ncmreynolds/ld2410/LICENSE for full license
 *
 * Translated to C from Arduino C++
 */

#ifndef LD2410_H
#define LD2410_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LD2410_MAX_FRAME_LENGTH 40
#ifndef LD2410_BUFFER_SIZE
#define LD2410_BUFFER_SIZE 256
#endif

// Debug defines - uncomment as needed
//#define LD2410_DEBUG_DATA
#define LD2410_DEBUG_COMMANDS
//#define LD2410_DEBUG_PARSE

// Forward declarations for generic stubs
typedef struct uart_handle uart_handle_t;
typedef struct debug_handle debug_handle_t;

// Frame data structure
typedef struct {
    const uint8_t* data;
    uint16_t length;
} frame_data_t;

// Parsing state enumeration
typedef enum {
    WAITING_START,
    READING_LENGTH,
    READING_DATA,
    CHECKING_END
} parse_state_t;

// Main LD2410 structure
typedef struct {
    // UART handles
    uart_handle_t *radar_uart;
    debug_handle_t *debug_uart;

    // Timing and timeouts
    uint32_t radar_uart_timeout;
    uint32_t radar_uart_last_packet;
    uint32_t radar_uart_last_command;
    uint32_t radar_uart_command_timeout;

    // Command handling
    uint8_t latest_ack;
    bool latest_command_success;

    // Frame handling
    uint8_t radar_data_frame[LD2410_MAX_FRAME_LENGTH];
    uint8_t radar_data_frame_position;
    bool frame_started;
    bool ack_frame;
    bool waiting_for_ack;

    // Target detection data
    uint8_t target_type;
    uint16_t moving_target_distance;
    uint8_t moving_target_energy;
    uint16_t stationary_target_distance;
    uint8_t stationary_target_energy;
    uint16_t last_valid_frame_length;

    // Circular buffer for data
    uint8_t circular_buffer[LD2410_BUFFER_SIZE];
    uint16_t buffer_head;
    uint16_t buffer_tail;

    // Configuration data
    uint8_t firmware_major_version;
    uint8_t firmware_minor_version;
    uint32_t firmware_bugfix_version;
    uint8_t max_gate;
    uint8_t max_moving_gate;
    uint8_t max_stationary_gate;
    uint16_t sensor_idle_time;
    uint8_t motion_sensitivity[9];
    uint8_t stationary_sensitivity[9];
} ld2410_t;

// Generic stub functions - replace these with actual implementations
// UART functions
int uart_available(uart_handle_t *uart);
uint8_t uart_read(uart_handle_t *uart);
void uart_write(uart_handle_t *uart, uint8_t byte);

// Debug/print functions
void debug_print(debug_handle_t *debug, const char *str);
void debug_print_int(debug_handle_t *debug, int value);
void debug_print_hex(debug_handle_t *debug, uint8_t value);
void debug_print_char(debug_handle_t *debug, char c);
void debug_println(debug_handle_t *debug, const char *str);

// Timing functions
uint32_t get_millis(void);
void delay_ms(uint32_t ms);
void yield_task(void);

// Task/threading functions (for FreeRTOS-like functionality)
typedef void (*task_function_t)(void *param);
void create_task(task_function_t func, const char *name, uint32_t stack_size,
                void *param, uint32_t priority, uint32_t core);
void task_delay(uint32_t ticks);

// Function prototypes
// Constructor/Destructor equivalents
void ld2410_init(ld2410_t *sensor);
void ld2410_cleanup(ld2410_t *sensor);

// Core functions
bool ld2410_begin(ld2410_t *sensor, uart_handle_t *uart_stream, bool wait_for_radar);
void ld2410_debug(ld2410_t *sensor, debug_handle_t *debug_stream);
bool ld2410_is_connected(ld2410_t *sensor);
bool ld2410_read(ld2410_t *sensor);

// Detection functions
bool ld2410_presence_detected(ld2410_t *sensor);
bool ld2410_stationary_target_detected(ld2410_t *sensor);
uint16_t ld2410_stationary_target_distance(ld2410_t *sensor);
uint8_t ld2410_stationary_target_energy(ld2410_t *sensor);
bool ld2410_moving_target_detected(ld2410_t *sensor);
uint16_t ld2410_moving_target_distance(ld2410_t *sensor);
uint8_t ld2410_moving_target_energy(ld2410_t *sensor);

// Configuration functions
bool ld2410_request_firmware_version(ld2410_t *sensor);
bool ld2410_request_current_configuration(ld2410_t *sensor);
bool ld2410_request_restart(ld2410_t *sensor);
bool ld2410_request_factory_reset(ld2410_t *sensor);
bool ld2410_request_start_engineering_mode(ld2410_t *sensor);
bool ld2410_request_end_engineering_mode(ld2410_t *sensor);
bool ld2410_set_max_values(ld2410_t *sensor, uint16_t moving, uint16_t stationary, uint16_t inactivity_timer);
bool ld2410_set_gate_sensitivity_threshold(ld2410_t *sensor, uint8_t gate, uint8_t moving, uint8_t stationary);

// Utility functions
frame_data_t ld2410_get_frame_data(ld2410_t *sensor);
void ld2410_auto_read_task(ld2410_t *sensor, uint32_t stack, uint32_t priority, uint32_t core);

// Private function prototypes
static void ld2410_add_to_buffer(ld2410_t *sensor, uint8_t byte);
static bool ld2410_read_from_buffer(ld2410_t *sensor, uint8_t *byte);
static bool ld2410_find_frame_start(ld2410_t *sensor);
static bool ld2410_check_frame_end(ld2410_t *sensor);
static bool ld2410_read_frame(ld2410_t *sensor);
static bool ld2410_read_frame_no_buffer(ld2410_t *sensor);
static bool ld2410_parse_data_frame(ld2410_t *sensor);
static bool ld2410_parse_command_frame(ld2410_t *sensor);
static void ld2410_print_frame(ld2410_t *sensor);
static void ld2410_send_command_preamble(ld2410_t *sensor);
static void ld2410_send_command_postamble(ld2410_t *sensor);
static bool ld2410_enter_configuration_mode(ld2410_t *sensor);
static bool ld2410_leave_configuration_mode(ld2410_t *sensor);
static void ld2410_task_function(void *param);

#endif // LD2410_H

/*
 * Implementation
 */

// Global variables for parsing state
static parse_state_t current_state = WAITING_START;
static uint8_t frame_buffer[1024]; // Pre-allocated frame buffer
static uint16_t frame_position = 0;
static uint16_t frame_length = 0;

// Constructor equivalent
void ld2410_init(ld2410_t *sensor) {
    sensor->radar_uart = NULL;
    sensor->debug_uart = NULL;
    sensor->radar_uart_timeout = 100;
    sensor->radar_uart_last_packet = 0;
    sensor->radar_uart_last_command = 0;
    sensor->radar_uart_command_timeout = 100;

    sensor->latest_ack = 0;
    sensor->latest_command_success = false;
    sensor->radar_data_frame_position = 0;
    sensor->frame_started = false;
    sensor->ack_frame = false;
    sensor->waiting_for_ack = false;

    sensor->target_type = 0;
    sensor->moving_target_distance = 0;
    sensor->moving_target_energy = 0;
    sensor->stationary_target_distance = 0;
    sensor->stationary_target_energy = 0;
    sensor->last_valid_frame_length = 0;

    sensor->buffer_head = 0;
    sensor->buffer_tail = 0;

    sensor->firmware_major_version = 0;
    sensor->firmware_minor_version = 0;
    sensor->firmware_bugfix_version = 0;
    sensor->max_gate = 0;
    sensor->max_moving_gate = 0;
    sensor->max_stationary_gate = 0;
    sensor->sensor_idle_time = 0;

    // Initialize sensitivity arrays
    for (int i = 0; i < 9; i++) {
        sensor->motion_sensitivity[i] = 0;
        sensor->stationary_sensitivity[i] = 0;
    }
}

// Destructor equivalent
void ld2410_cleanup(ld2410_t *sensor) {
    // Clean up any allocated resources if needed
    sensor->radar_uart = NULL;
    sensor->debug_uart = NULL;
}

// Add byte to circular buffer
static void ld2410_add_to_buffer(ld2410_t *sensor, uint8_t byte) {
    // Insert byte into circular buffer
    sensor->circular_buffer[sensor->buffer_head] = byte;
    sensor->buffer_head = (sensor->buffer_head + 1) % LD2410_BUFFER_SIZE;

    // Handle buffer overflow by overwriting oldest data
    if (sensor->buffer_head == sensor->buffer_tail) {
        sensor->buffer_tail = (sensor->buffer_tail + 1) % LD2410_BUFFER_SIZE;
    }
}

// Read from buffer
static bool ld2410_read_from_buffer(ld2410_t *sensor, uint8_t *byte) {
    if (sensor->buffer_head == sensor->buffer_tail) {
        return false;  // Buffer empty
    } else {
        *byte = sensor->circular_buffer[sensor->buffer_tail];
        sensor->buffer_tail = (sensor->buffer_tail + 1) % LD2410_BUFFER_SIZE;
        return true;
    }
}

// Find frame start
static bool ld2410_find_frame_start(ld2410_t *sensor) {
    uint8_t byte;

    // Continue reading from buffer until frame start is found
    while (ld2410_read_from_buffer(sensor, &byte)) {
        // Check if byte is frame start
        if (byte == 0xF4 || byte == 0xFD) {
            // Start new frame
            sensor->radar_data_frame[0] = byte;
            sensor->radar_data_frame_position = 1;
            sensor->frame_started = true;
            sensor->ack_frame = (byte == 0xFD);  // Determine frame type (ack or data)
            return true;
        }
    }

    return false;  // No frame found
}

bool ld2410_begin(ld2410_t *sensor, uart_handle_t *uart_stream, bool wait_for_radar) {
    sensor->radar_uart = uart_stream;

    if (sensor->debug_uart != NULL) {
        debug_println(sensor->debug_uart, "ld2410 started");
    }

    if (!wait_for_radar) {
        return true;
    }

    // Try to read firmware version
    if (sensor->debug_uart != NULL) {
        debug_print(sensor->debug_uart, "\nLD2410 firmware: ");
    }

    uint32_t start_time = get_millis();
    bool firmware_received = false;

    while (get_millis() - start_time < 1000) { // 1 second timeout
        if (ld2410_request_firmware_version(sensor)) {
            firmware_received = true;
            break;
        }
        yield_task();
    }

    if (firmware_received) {
        if (sensor->debug_uart != NULL) {
            debug_print(sensor->debug_uart, " v");
            debug_print_int(sensor->debug_uart, sensor->firmware_major_version);
            debug_print_char(sensor->debug_uart, '.');
            debug_print_int(sensor->debug_uart, sensor->firmware_minor_version);
            debug_print_char(sensor->debug_uart, '.');
            debug_print_int(sensor->debug_uart, sensor->firmware_bugfix_version);
        }
        return true;
    }

    if (sensor->debug_uart != NULL) {
        debug_print(sensor->debug_uart, "no response");
    }
    return false;
}

void ld2410_debug(ld2410_t *sensor, debug_handle_t *debug_stream) {
    sensor->debug_uart = debug_stream;
}

bool ld2410_is_connected(ld2410_t *sensor) {
    if (get_millis() - sensor->radar_uart_last_packet < sensor->radar_uart_timeout) {
        return true;
    }
    if (ld2410_read_frame(sensor)) {
        return true;
    }
    return false;
}

bool ld2410_read(ld2410_t *sensor) {
    bool new_data = false;
    // Read all available data from UART buffer
    while (uart_available(sensor->radar_uart)) {
        ld2410_add_to_buffer(sensor, uart_read(sensor->radar_uart));
        new_data = true;
    }

    // Try to read and process a frame
    bool frame_processed = ld2410_read_frame(sensor);

    // Return true if new data was read or frame was processed
    return new_data || frame_processed;
}

static void ld2410_task_function(void *param) {
    ld2410_t *sensor = (ld2410_t *)param;
    while (1) {
        // Read data from UART and add to circular buffer
        bool new_data = false;
        while (uart_available(sensor->radar_uart)) {
            ld2410_add_to_buffer(sensor, uart_read(sensor->radar_uart));
            new_data = true;
        }

        // If there's new data, try to process a frame
        if (new_data) {
            ld2410_read_frame(sensor);
        }

        // Delay to avoid task overload
        task_delay(10);
    }
}

// Start auto-read task
void ld2410_auto_read_task(ld2410_t *sensor, uint32_t stack, uint32_t priority, uint32_t core) {
    create_task(ld2410_task_function, "LD2410Task", stack, sensor, priority, core);
}

bool ld2410_presence_detected(ld2410_t *sensor) {
    return sensor->target_type != 0;
}

bool ld2410_stationary_target_detected(ld2410_t *sensor) {
    if ((sensor->target_type & 0x02) && sensor->stationary_target_distance > 0 && sensor->stationary_target_energy > 0) {
        return true;
    }
    return false;
}

uint16_t ld2410_stationary_target_distance(ld2410_t *sensor) {
    return sensor->stationary_target_distance;
}

uint8_t ld2410_stationary_target_energy(ld2410_t *sensor) {
    return sensor->stationary_target_energy;
}

bool ld2410_moving_target_detected(ld2410_t *sensor) {
    if ((sensor->target_type & 0x01) && sensor->moving_target_distance > 0 && sensor->moving_target_energy > 0) {
        return true;
    }
    return false;
}

uint16_t ld2410_moving_target_distance(ld2410_t *sensor) {
    return sensor->moving_target_distance;
}

uint8_t ld2410_moving_target_energy(ld2410_t *sensor) {
    if (sensor->moving_target_energy > 100) {
        return 100;  // Limit to 100 if value is higher
    }
    return sensor->moving_target_energy;
}

// Check frame end
static bool ld2410_check_frame_end(ld2410_t *sensor) {
    if (sensor->ack_frame) {
        return (sensor->radar_data_frame[0] == 0xFD &&
                sensor->radar_data_frame[1] == 0xFC &&
                sensor->radar_data_frame[2] == 0xFB &&
                sensor->radar_data_frame[3] == 0xFA &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 4] == 0x04 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 3] == 0x03 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 2] == 0x02 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 1] == 0x01);
    } else {
        return (sensor->radar_data_frame[0] == 0xF4 &&
                sensor->radar_data_frame[1] == 0xF3 &&
                sensor->radar_data_frame[2] == 0xF2 &&
                sensor->radar_data_frame[3] == 0xF1 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 4] == 0xF8 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 3] == 0xF7 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 2] == 0xF6 &&
                sensor->radar_data_frame[sensor->radar_data_frame_position - 1] == 0xF5);
    }
}

// Print frame for debugging
static void ld2410_print_frame(ld2410_t *sensor) {
    if (sensor->debug_uart != NULL) {
        if (sensor->ack_frame == true) {
            debug_print(sensor->debug_uart, "\nCmnd : ");
        } else {
            debug_print(sensor->debug_uart, "\nData : ");
        }
        for (uint8_t i = 0; i < sensor->radar_data_frame_position; i++) {
            if (sensor->radar_data_frame[i] < 0x10) {
                debug_print_char(sensor->debug_uart, '0');
            }
            debug_print_hex(sensor->debug_uart, sensor->radar_data_frame[i]);
            debug_print_char(sensor->debug_uart, ' ');
        }
    }
}

// Read frame from buffer
static bool ld2410_read_frame(ld2410_t *sensor) {
    uint8_t byte_read;
    while (ld2410_read_from_buffer(sensor, &byte_read)) {
        // If frame hasn't started, check for frame start
        if (!sensor->frame_started) {
            if (byte_read == 0xF4 || byte_read == 0xFD) {
                sensor->radar_data_frame[0] = byte_read;
                sensor->radar_data_frame_position = 1;
                sensor->frame_started = true;
                sensor->ack_frame = (byte_read == 0xFD);  // Determine frame type
            }
        } else {
            // Continue accumulating frame bytes
            sensor->radar_data_frame[sensor->radar_data_frame_position++] = byte_read;

            // After reading at least 8 bytes, verify frame length
            if (sensor->radar_data_frame_position == 8) {
                uint16_t intra_frame_data_length = sensor->radar_data_frame[4] | (sensor->radar_data_frame[5] << 8);

                // Check if frame length exceeds maximum allowed
                if (intra_frame_data_length + 10 > LD2410_MAX_FRAME_LENGTH) {
                    sensor->frame_started = false;
                    sensor->radar_data_frame_position = 0;
                    continue;  // Skip this frame
                }
            }

            // Check if frame is complete
            if (sensor->radar_data_frame_position >= 8 && ld2410_check_frame_end(sensor)) {
                sensor->frame_started = false;  // Reset state for next frame

                // Process frame (command or data)
                if (sensor->ack_frame) {
                    return ld2410_parse_command_frame(sensor);
                } else {
                    return ld2410_parse_data_frame(sensor);
                }
            }
        }
    }
    return false;  // No complete frame found
}

// Parse data frame
static bool ld2410_parse_data_frame(ld2410_t *sensor) {
    uint16_t intra_frame_data_length = sensor->radar_data_frame[4] | (sensor->radar_data_frame[5] << 8);

    // Verify frame length is correct
    if (sensor->radar_data_frame_position != intra_frame_data_length + 10) {
        return false;
    }

    // Check specific bytes to validate frame
    if (sensor->radar_data_frame[6] == 0x02 && sensor->radar_data_frame[7] == 0xAA &&
        sensor->radar_data_frame[17] == 0x55 && sensor->radar_data_frame[18] == 0x00) {

        sensor->target_type = sensor->radar_data_frame[8];

        // Extract target distances and energies
        sensor->stationary_target_distance = *(uint16_t*)(&sensor->radar_data_frame[9]);
        sensor->moving_target_distance = *(uint16_t*)(&sensor->radar_data_frame[15]);
        sensor->stationary_target_energy = sensor->radar_data_frame[14];
        sensor->moving_target_energy = sensor->radar_data_frame[11];

        sensor->last_valid_frame_length = sensor->radar_data_frame_position;
        sensor->radar_uart_last_packet = get_millis();
        return true;
    }

    return false;  // Invalid frame
}

// Parse command frame (simplified version - full implementation would be very long)
static bool ld2410_parse_command_frame(ld2410_t *sensor) {
    uint16_t intra_frame_data_length = sensor->radar_data_frame[4] + (sensor->radar_data_frame[5] << 8);

#ifdef LD2410_DEBUG_COMMANDS
    if (sensor->debug_uart != NULL) {
        ld2410_print_frame(sensor);
        debug_print(sensor->debug_uart, "\nACK frame payload: ");
        debug_print_int(sensor->debug_uart, intra_frame_data_length);
        debug_print(sensor->debug_uart, " bytes");
    }
#endif

    sensor->latest_ack = sensor->radar_data_frame[6];
    sensor->latest_command_success = (sensor->radar_data_frame[8] == 0x00 && sensor->radar_data_frame[9] == 0x00);

    // Handle various ACK types (simplified - full implementation would have all cases)
    if (intra_frame_data_length == 12 && sensor->latest_ack == 0xA0) {
        // Firmware version ACK
        if (sensor->latest_command_success) {
            sensor->firmware_major_version = sensor->radar_data_frame[13];
            sensor->firmware_minor_version = sensor->radar_data_frame[12];
            sensor->firmware_bugfix_version = sensor->radar_data_frame[14];
            sensor->firmware_bugfix_version += sensor->radar_data_frame[15] << 8;
            sensor->firmware_bugfix_version += sensor->radar_data_frame[16] << 16;
            sensor->firmware_bugfix_version += sensor->radar_data_frame[17] << 24;
            sensor->radar_uart_last_packet = get_millis();
            return true;
        }
    }

    if (sensor->latest_command_success) {
        sensor->last_valid_frame_length = sensor->radar_data_frame_position;
        sensor->radar_uart_last_packet = get_millis();
        return true;
    }

    return false;
}

// Command preamble
static void ld2410_send_command_preamble(ld2410_t *sensor) {
    uart_write(sensor->radar_uart, 0xFD);
    uart_write(sensor->radar_uart, 0xFC);
    uart_write(sensor->radar_uart, 0xFB);
    uart_write(sensor->radar_uart, 0xFA);
}

// Command postamble
static void ld2410_send_command_postamble(ld2410_t *sensor) {
    uart_write(sensor->radar_uart, 0x04);
    uart_write(sensor->radar_uart, 0x03);
    uart_write(sensor->radar_uart, 0x02);
    uart_write(sensor->radar_uart, 0x01);
}

// Enter configuration mode
static bool ld2410_enter_configuration_mode(ld2410_t *sensor) {
    ld2410_send_command_preamble(sensor);
    uart_write(sensor->radar_uart, 0x04);  // Command is four bytes long
    uart_write(sensor->radar_uart, 0x00);
    uart_write(sensor->radar_uart, 0xFF);  // Request enter command mode
    uart_write(sensor->radar_uart, 0x00);
    uart_write(sensor->radar_uart, 0x01);
    uart_write(sensor->radar_uart, 0x00);
    ld2410_send_command_postamble(sensor);

    sensor->radar_uart_last_command = get_millis();
    while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
        if (ld2410_read_frame_no_buffer(sensor)) {
            if (sensor->latest_ack == 0xFF && sensor->latest_command_success) {
                return true;
            }
        }
    }
    return false;
}

// Leave configuration mode
static bool ld2410_leave_configuration_mode(ld2410_t *sensor) {
    ld2410_send_command_preamble(sensor);
    uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
    uart_write(sensor->radar_uart, 0x00);
    uart_write(sensor->radar_uart, 0xFE);  // Request leave command mode
    uart_write(sensor->radar_uart, 0x00);
    ld2410_send_command_postamble(sensor);

    sensor->radar_uart_last_command = get_millis();
    while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
        if (ld2410_read_frame_no_buffer(sensor)) {
            if (sensor->latest_ack == 0xFE && sensor->latest_command_success) {
                return true;
            }
        }
    }
    return false;
}

// Read frame without buffer (simplified version)
static bool ld2410_read_frame_no_buffer(ld2410_t *sensor) {
    if (uart_available(sensor->radar_uart)) {
        if (sensor->frame_started == false) {
            uint8_t byte_read = uart_read(sensor->radar_uart);
            if (byte_read == 0xF4) {
                sensor->radar_data_frame[sensor->radar_data_frame_position++] = byte_read;
                sensor->frame_started = true;
                sensor->ack_frame = false;
            } else if (byte_read == 0xFD) {
                sensor->radar_data_frame[sensor->radar_data_frame_position++] = byte_read;
                sensor->frame_started = true;
                sensor->ack_frame = true;
            }
        } else {
            if (sensor->radar_data_frame_position < LD2410_MAX_FRAME_LENGTH) {
                sensor->radar_data_frame[sensor->radar_data_frame_position++] = uart_read(sensor->radar_uart);

                if (sensor->radar_data_frame_position > 7) {
                    // Check for frame end and parse if complete
                    if (ld2410_check_frame_end(sensor)) {
                        bool result;
                        if (sensor->ack_frame) {
                            result = ld2410_parse_command_frame(sensor);
                        } else {
                            result = ld2410_parse_data_frame(sensor);
                        }
                        sensor->frame_started = false;
                        sensor->radar_data_frame_position = 0;
                        return result;
                    }
                }
            } else {
                sensor->frame_started = false;
                sensor->radar_data_frame_position = 0;
            }
        }
    }
    return false;
}

// Request firmware version
bool ld2410_request_firmware_version(ld2410_t *sensor) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0xA0);  // Request firmware version
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            ld2410_read_frame_no_buffer(sensor);
            if (sensor->latest_ack == 0xA0 && sensor->latest_command_success) {
                delay_ms(50);
                ld2410_leave_configuration_mode(sensor);
                return true;
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Request current configuration
bool ld2410_request_current_configuration(ld2410_t *sensor) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x61);  // Request current configuration
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            if (ld2410_read_frame_no_buffer(sensor)) {
                if (sensor->latest_ack == 0x61 && sensor->latest_command_success) {
                    delay_ms(50);
                    ld2410_leave_configuration_mode(sensor);
                    return true;
                }
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Request restart
bool ld2410_request_restart(ld2410_t *sensor) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0xA3);  // Request restart
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            if (ld2410_read_frame(sensor)) {
                if (sensor->latest_ack == 0xA3 && sensor->latest_command_success) {
                    delay_ms(50);
                    ld2410_leave_configuration_mode(sensor);
                    return true;
                }
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Request factory reset
bool ld2410_request_factory_reset(ld2410_t *sensor) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0xA2);  // Request factory reset
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            if (ld2410_read_frame(sensor)) {
                if (sensor->latest_ack == 0xA2 && sensor->latest_command_success) {
                    delay_ms(50);
                    ld2410_leave_configuration_mode(sensor);
                    return true;
                }
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Start engineering mode
bool ld2410_request_start_engineering_mode(ld2410_t *sensor) {
    ld2410_send_command_preamble(sensor);
    uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
    uart_write(sensor->radar_uart, 0x00);
    uart_write(sensor->radar_uart, 0x62);  // Request enter engineering mode
    uart_write(sensor->radar_uart, 0x00);
    ld2410_send_command_postamble(sensor);

    sensor->radar_uart_last_command = get_millis();
    while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
        if (ld2410_read_frame_no_buffer(sensor)) {
            if (sensor->latest_ack == 0x62 && sensor->latest_command_success) {
                return true;
            }
        }
    }
    return false;
}

// End engineering mode
bool ld2410_request_end_engineering_mode(ld2410_t *sensor) {
    ld2410_send_command_preamble(sensor);
    uart_write(sensor->radar_uart, 0x02);  // Command is two bytes long
    uart_write(sensor->radar_uart, 0x00);
    uart_write(sensor->radar_uart, 0x63);  // Request leave engineering mode
    uart_write(sensor->radar_uart, 0x00);
    ld2410_send_command_postamble(sensor);

    sensor->radar_uart_last_command = get_millis();
    while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
        if (ld2410_read_frame_no_buffer(sensor)) {
            if (sensor->latest_ack == 0x63 && sensor->latest_command_success) {
                return true;
            }
        }
    }
    return false;
}

// Set maximum values
bool ld2410_set_max_values(ld2410_t *sensor, uint16_t moving, uint16_t stationary, uint16_t inactivity_timer) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x14);  // Command is 20 bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x60);  // Request set max values
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x00);  // Moving gate command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, (uint8_t)(moving & 0x00FF));  // Moving gate value
        uart_write(sensor->radar_uart, (uint8_t)((moving & 0xFF00) >> 8));
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x01);  // Stationary gate command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, (uint8_t)(stationary & 0x00FF));  // Stationary gate value
        uart_write(sensor->radar_uart, (uint8_t)((stationary & 0xFF00) >> 8));
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x02);  // Inactivity timer command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, (uint8_t)(inactivity_timer & 0x00FF));  // Inactivity timer
        uart_write(sensor->radar_uart, (uint8_t)((inactivity_timer & 0xFF00) >> 8));
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            if (ld2410_read_frame(sensor)) {
                if (sensor->latest_ack == 0x60 && sensor->latest_command_success) {
                    delay_ms(50);
                    ld2410_leave_configuration_mode(sensor);
                    return true;
                }
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Set gate sensitivity threshold
bool ld2410_set_gate_sensitivity_threshold(ld2410_t *sensor, uint8_t gate, uint8_t moving, uint8_t stationary) {
    if (ld2410_enter_configuration_mode(sensor)) {
        delay_ms(50);
        ld2410_send_command_preamble(sensor);
        uart_write(sensor->radar_uart, 0x14);  // Command is 20 bytes long
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x64);  // Request set sensitivity values
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x00);  // Gate command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, gate);  // Gate value
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x01);  // Motion sensitivity command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, moving);  // Motion sensitivity value
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x02);  // Stationary sensitivity command
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, stationary);  // Stationary sensitivity value
        uart_write(sensor->radar_uart, 0x00);
        uart_write(sensor->radar_uart, 0x00);  // Spacer
        uart_write(sensor->radar_uart, 0x00);
        ld2410_send_command_postamble(sensor);

        sensor->radar_uart_last_command = get_millis();
        while (get_millis() - sensor->radar_uart_last_command < sensor->radar_uart_command_timeout) {
            if (ld2410_read_frame(sensor)) {
                if (sensor->latest_ack == 0x64 && sensor->latest_command_success) {
                    delay_ms(50);
                    ld2410_leave_configuration_mode(sensor);
                    return true;
                }
            }
        }
    }
    delay_ms(50);
    ld2410_leave_configuration_mode(sensor);
    return false;
}

// Get frame data
frame_data_t ld2410_get_frame_data(ld2410_t *sensor) {
    frame_data_t result = {NULL, 0};

    // Use last_valid_frame_length as initial length
    uint16_t frame_length = sensor->last_valid_frame_length;

    // Verify header
    if (sensor->radar_data_frame[0] != 0xF4 ||
        sensor->radar_data_frame[1] != 0xF3 ||
        sensor->radar_data_frame[2] != 0xF2 ||
        sensor->radar_data_frame[3] != 0xF1) {
        // Invalid header
        return result;
    }

    // Verify frame length from bytes 4 and 5
    uint16_t reported_length = sensor->radar_data_frame[4] | (sensor->radar_data_frame[5] << 8);
    reported_length += 10;  // Add 10 for header and footer

    // Use the smaller length between reported and last_valid_frame_length
    frame_length = (reported_length < frame_length) ? reported_length : frame_length;

    // Verify frame length is valid
    if (frame_length > LD2410_MAX_FRAME_LENGTH || frame_length < 10) {
        // Invalid frame length
        return result;
    }

    // Verify footer
    if (sensor->radar_data_frame[frame_length - 4] != 0xF8 ||
        sensor->radar_data_frame[frame_length - 3] != 0xF7 ||
        sensor->radar_data_frame[frame_length - 2] != 0xF6 ||
        sensor->radar_data_frame[frame_length - 1] != 0xF5) {
        // Invalid footer
        return result;
    }

    // If all checks pass, return valid data
    result.data = sensor->radar_data_frame;
    result.length = frame_length;
    return result;
}

/*
 * Generic stub implementations - replace these with actual platform-specific code
 */

// UART stub functions
int uart_available(uart_handle_t *uart) {
    // Return number of bytes available in UART buffer
    // Platform-specific implementation needed
    return 0;
}

uint8_t uart_read(uart_handle_t *uart) {
    // Read one byte from UART
    // Platform-specific implementation needed
    return 0;
}

void uart_write(uart_handle_t *uart, uint8_t byte) {
    // Write one byte to UART
    // Platform-specific implementation needed
}

// Debug/print stub functions
void debug_print(debug_handle_t *debug, const char *str) {
    // Print string to debug output
    // Platform-specific implementation needed (printf, UART, etc.)
}

void debug_print_int(debug_handle_t *debug, int value) {
    // Print integer to debug output
    // Platform-specific implementation needed
}

void debug_print_hex(debug_handle_t *debug, uint8_t value) {
    // Print hex value to debug output
    // Platform-specific implementation needed
}

void debug_print_char(debug_handle_t *debug, char c) {
    // Print single character to debug output
    // Platform-specific implementation needed
}

void debug_println(debug_handle_t *debug, const char *str) {
    // Print string with newline to debug output
    // Platform-specific implementation needed
}

// Timing stub functions
uint32_t get_millis(void) {
    // Return milliseconds since system start
    // Platform-specific implementation needed
    return 0;
}

void delay_ms(uint32_t ms) {
    // Delay for specified milliseconds
    // Platform-specific implementation needed
}

void yield_task(void) {
    // Yield to other tasks/threads
    // Platform-specific implementation needed
}

// Task/threading stub functions
void create_task(task_function_t func, const char *name, uint32_t stack_size,
                void *param, uint32_t priority, uint32_t core) {
    // Create a new task/thread
    // Platform-specific implementation needed (FreeRTOS, pthreads, etc.)
}

void task_delay(uint32_t ticks) {
    // Task delay in ticks
    // Platform-specific implementation needed
}
#endif
