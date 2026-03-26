#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

typedef enum {
    ST_BOOT = 0,
	ST_WAIT_UI_INIT,
    ST_INIT_POWER,
    ST_INIT_IMU,
    ST_INIT_DAY,
    ST_INIT_IR,
    ST_INIT_FPGA_CAL,
	ST_INIT_TEMP,
    ST_OPERATION,
    ST_ERROR
} McuState;

typedef struct {
    McuState state;
    uint8_t init_seq;    /* sequence number from the CMD_INIT_SYSTEM command */
    uint8_t  day_idx;
    uint8_t  ir_idx;
    bool     imu_streaming;
    bool     debug_enabled;
    bool     log_capture_active;
    bool     log_capture_save_sent;
    uint32_t log_capture_start_ms;
} McuContext;

void mcu_fsm_start_init(McuContext *ctx);

void mcu_fsm_init(McuContext *ctx);
void mcu_fsm_step(McuContext *ctx);

// Called when IMU DRDY interrupt occurs (flag set)
void mcu_on_imu_drdy(McuContext *ctx);

// Called when FPGA VSYNC interrupt occurs (PC4 rising edge)
void mcu_on_vsync(McuContext *ctx);

// Called when a valid UI payload is received (already CRC/COBS-validated)
void mcu_on_ui_payload(McuContext *ctx, const uint8_t *p, uint16_t n);

/**
 * @brief Send a printf-style message to the UI log window.
 *
 * Builds a MSG_LOG frame and transmits it over UART3.
 * Callable from any file — pass &g_ctx from main.c.
 * No-op if ctx->debug_enabled is false.
 *
 * Example:
 *   mcu_ui_log(&g_ctx, "Verify Genlock: NV17=%u NV18=%u\r\n", nv17, nv18);
 */
void mcu_ui_log(McuContext *ctx, const char *fmt, ...);


