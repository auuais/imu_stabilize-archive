// mcu_fsm.c
// UART3 is reserved for the binary UI protocol (COBS + CRC16).
// Any "logs" must be sent as MSG_LOG frames (not raw printf).

#include "mcu_fsm.h"
#include "main.h"
#include "proto_uart.h"
#include "fpga_i2c.h"
#include "fpga_regs.h"
#include "imu_handler.h"
#include "imu_config.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stddef.h>
#include <limits.h>

#ifndef ICD_ONLY_UI_PROTOCOL
#define ICD_ONLY_UI_PROTOCOL 1
#endif

//
static void fpga_service_orient(void);
static int imu_stream_read_raw(uint8_t *dst);
static void ui_send_log(McuContext *ctx, const char *s);
static void icd_service_periodic(McuContext *ctx);
static void icd_send_pbit(uint32_t seq);
static void icd_send_status(uint32_t seq);
static void icd_maintenance_init_defaults(void);
static uint8_t icd_apply_maintenance_payload(const uint8_t *payload, uint16_t payload_len);
static void icd_fill_maintenance_ack(uint8_t p[43]);
static void icd_fill_maintenance_status(uint8_t p[109]);


// ---- externs from existing codebase ----
extern void Enable_All_Camera_Power(void);
extern int Power_Cycle_All_Cameras(void);
extern void IMU_Reset(void);
extern void IMU_SoftReset(void);
extern int send_imu_command(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name);
extern int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name, uint8_t *rx_out);
extern void Read_HDC3022_Sensor(float* temp, float* hum);
extern void Send_UART3_Debug(const char *format, ...);
extern int test_day_camera_controls(uint8_t camera_num);
extern int Initialize_IR_Camera(uint8_t camera_num);
extern int ir_camera_handshake(uint8_t camera_num);
extern int IR_PresenceProbe_Start(uint8_t camera_num); // 0=started (non-blocking)
extern int IR_PresenceProbe_Poll(void);  // -1 idle, 0 busy, 1 present, 2 absent, 3 aborted
extern const char *mcu_get_reset_cause_str(void);
extern int Apply_IR_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
extern int Apply_EO_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
extern void IR_Nuc_OnOperationEnter(void);
extern void IR_Nuc_Service(McuContext *ctx);
extern SPI_HandleTypeDef hspi2;

// UART3 (PC UI)
extern UART_HandleTypeDef huart3;

// Optional global debug mirror (so legacy code can keep calling Send_UART3_Debug safely)
extern volatile bool g_debug_enabled;
extern fpga_i2c_t g_fpga;
extern void IMU_Handler_SetSampleFreq(float sample_freq);
extern volatile uint32_t g_imu_drdy_count;
extern volatile uint32_t g_imu_drdy_edges_total;

// ---- config ----
#define UI_TX_MAX   512
#define UI_PAY_MAX  256

// ---- Message types ----
#define MSG_CMD   0x01
#define MSG_RESP  0x02
#define MSG_EVT   0x03
#define MSG_LOG   0x7F

// ---- Event subtypes ----
#define EVT_IMU_ORIENT  0x20   // roll_cd (int16 LE) + pitch_cd (int16 LE)
#define EVT_IMU_ODR     0x21   // rate_hz (uint16 LE)
#define EVT_IMU_STATS   0x22   // drdy_hz + pkt_ts_hz + parse_ok_hz (uint16 LE each)
#define EVT_CFG_BANK    0x30   // [sub][bank][mode][yaw/pitch/roll offsets + overlap + feather]
#define EVT_CFG_APPLY   0x31   // [sub][bank][ok][detail]
#define EVT_LOG_CTRL    0x40   // [sub][cmd]

// ---- Command IDs ----
#define CMD_CHECK_NODE   0x10
#define CMD_SET_MODE     0x11
#define CMD_SET_DEBUG    0x12
#define CMD_INIT_SYSTEM  0x13
#define CMD_CAMERA_RESET 0x14
#define CMD_PING         0x15
#define CMD_GET_VERSION  0x16
#define CMD_SET_CAMERA_POWER 0x17
#define CMD_GET_CONFIG   0x30
#define CMD_APPLY_CONFIG 0x31
#define CMD_CAMERA_CONTROL 0x32

#define ICD_RITA_PIPA_MODE_CONTROL       0x2201u
#define ICD_RITA_PIPA_CAMERA_CONTROL     0x2202u
#define ICD_RITA_PIPA_PANORAMA_CONTROL   0x2203u
#define ICD_RITA_PIPA_MAINTENANCE        0x2204u
#define ICD_RITA_PIPA_UTC_INFO           0x2205u
#define ICD_PIPA_RITA_STATUS             0x2210u
#define ICD_PIPA_RITA_MAINTENANCE_STATUS 0x2211u
/* Customer "Message List(종합)" maps 0x2212/0x2213/0x2214 directly to
 * PBIT/CBIT/IBIT. The older detailed-sheet PARAMETER row is not active here. */
/* #define ICD_PIPA_RITA_PARAMETER       0x2212u */
#define ICD_PIPA_RITA_PBIT               0x2212u
#define ICD_PIPA_RITA_CBIT               0x2213u
#define ICD_PIPA_RITA_IBIT               0x2214u
#define ICD_PIPA_RITA_MODE_CONTROL_ACK   0x22A1u
#define ICD_PIPA_RITA_CAMERA_CONTROL_ACK 0x22A2u
#define ICD_PIPA_RITA_PANORAMA_ACK       0x22A3u
#define ICD_PIPA_RITA_MAINTENANCE_ACK    0x22A4u

#define ICD_OPERATION_STANDBY      1u
#define ICD_OPERATION_TRACKING     2u
#define ICD_OPERATION_BASIC        3u
#define ICD_OPERATION_MAST         4u
#define ICD_OPERATION_MAINTENANCE  5u
#define ICD_OPERATION_IBIT         6u

// ---- Node types (for CHECK) ----
#define NODE_SYSTEM 0x00
#define NODE_IMU   0x01
#define NODE_TEMP  0x02
#define NODE_FPGA  0x03
#define NODE_DAY   0x04
#define NODE_IR    0x05

// ---- Error codes ----
#define ERR_OK     0x00
#define ERR_BUSY   0x01
#define ERR_BADARG 0x02
#define ERR_FAIL   0x03

#define ICD_CAMERA_ACK_LEN        17u
#define ICD_CAM_ACK_NA            0u
#define ICD_CAM_ACK_POWER_ON_OK   1u
#define ICD_CAM_ACK_POWER_OFF_OK  2u
#define ICD_CAM_ACK_NUC_OK        3u
#define ICD_CAM_ACK_FAIL          4u

static bool s_icd_enabled = false;
static uint8_t s_icd_operation_mode = ICD_OPERATION_STANDBY;
static uint32_t s_icd_last_cbit_ms = 0u;
static uint32_t s_icd_last_status_ms = 0u;
static bool s_icd_initial_pbit_sent = false;
static bool s_icd_time_valid = false;
static uint64_t s_icd_time_epoch_ms = 0u;
static uint32_t s_icd_time_base_ms = 0u;
static uint32_t s_icd_seq_status = 1u;
static uint32_t s_icd_seq_maintenance_status = 1u;
static uint32_t s_icd_seq_pbit = 1u;
static uint32_t s_icd_seq_cbit = 1u;
static uint32_t s_icd_seq_ibit = 1u;
static uint32_t s_icd_seq_mode_ack = 1u;
static uint32_t s_icd_seq_camera_ack = 1u;
static uint32_t s_icd_seq_panorama_ack = 1u;
static uint32_t s_icd_seq_maintenance_ack = 1u;
static uint32_t s_icd_seq_default = 1u;
static const uint32_t ICD_CBIT_INTERVAL_MS = 1000u;
static const uint32_t ICD_STATUS_INTERVAL_MS = 100u;
static const uint32_t ICD_NUC_STATUS_HOLD_MS = 1500u;
static uint8_t s_day_power_on[6] = { 1u, 1u, 1u, 1u, 1u, 1u };
static uint8_t s_ir_power_on[6] = { 1u, 1u, 1u, 1u, 1u, 1u };
static uint8_t s_ir_bit_ok[6] = { 0u, 0u, 0u, 0u, 0u, 0u };
static uint8_t s_ir_cbit_next_idx = 0u;
static uint8_t s_ir_probe_inflight_cam = 0xFFu;  // 0xFF = no background probe running
static uint32_t s_ir_nuc_status_until_ms[6] = {0};
static uint8_t s_icd_video_brightness = 0u;
static uint8_t s_icd_video_contrast = 0u;
static uint8_t s_icd_ir_image_polarity = 0u;
static uint8_t s_icd_video_stabilization = 0u;
static uint8_t s_icd_video_stabilization_method = 0u;
static uint16_t s_icd_blending_area_px = 0u;
static uint8_t s_icd_env_temp_c = 0u;
static uint8_t s_icd_env_humidity_pct = 0u;
static uint8_t s_icd_env_ok = 0u;
static uint8_t s_icd_env_valid = 0u;
static uint32_t s_icd_env_last_ms = 0u;
static uint8_t s_icd_maintenance_status[37] = {0};
enum {
    /* Golden customer frames use 21 bytes for 0x2210. The v0.92 workbook
     * shows a 22-byte variant with VIDEO_SYMBOL_ON_OFF, but that is not used
     * until customer confirms the live parser changed. */
    ICD_STATUS_LEN = 21,
    ICD_MAINT_ACK_LEN = 43,
    ICD_MAINT_STATUS_LEN = 109,
    ICD_IR_TNR_MAX_VALUE = 16383,
    ICD_IR_TNR_STEPS = 15,
    ICD_PANORAMA_BASE_LEN = 8,
    ICD_PANORAMA_EXT_LEN = 9,
    ICD_PANORAMA_FPGA_RESET_OFF = 8
};

enum {
    ICD_MAINT_SELECTED_IR_CAM = 0,
    ICD_MAINT_IR_NUC = 1,
    ICD_MAINT_IR_SCREEN_INV = 2,
    ICD_MAINT_IR_FRAME_RATE = 3,
    ICD_MAINT_IR_ICE_GAIN_L = 4,
    ICD_MAINT_IR_ICE_GAIN_H = 5,
    ICD_MAINT_IR_ICE_LEVEL_L = 6,
    ICD_MAINT_IR_ICE_LEVEL_H = 7,
    ICD_MAINT_IR_ICE_STRENGTH = 8,
    ICD_MAINT_IR_COLOR_MODE = 9,
    ICD_MAINT_IR_POLARITY = 10,
    ICD_MAINT_IR_DZOOM = 11,
    ICD_MAINT_IR_TNR_ENABLE = 12,
    ICD_MAINT_IR_TNR_L = 13,
    ICD_MAINT_IR_TNR_H = 14,
    ICD_MAINT_SELECTED_EO_CAM = 15,
    ICD_MAINT_EO_POWER_RESET = 16,
    ICD_MAINT_EO_DZOOM = 17,
    ICD_MAINT_EO_WHITE_BALANCE = 18,
    ICD_MAINT_EO_RED_GAIN = 19,
    ICD_MAINT_EO_BLUE_GAIN = 20,
    ICD_MAINT_EO_CHROMA_GAIN = 21,
    ICD_MAINT_EO_AGC = 22,
    ICD_MAINT_EO_AE_SHUTTER = 23,
    ICD_MAINT_EO_BRIGHTNESS = 24,
    ICD_MAINT_EO_BLC = 25,
    ICD_MAINT_EO_HLC_ONOFF = 26,
    ICD_MAINT_EO_HLC_LEVEL = 27,
    ICD_MAINT_EO_ACE_ONOFF = 28,
    ICD_MAINT_EO_ACE_LEVEL = 29,
    ICD_MAINT_EO_DEFOG_ONOFF = 30,
    ICD_MAINT_EO_DEFOG_LEVEL = 31,
    ICD_MAINT_EO_DNR = 32,
    ICD_MAINT_EO_GAMMA = 33,
    ICD_MAINT_EO_SHARPNESS = 34,
    ICD_MAINT_EO_MIRROR = 35,
    ICD_MAINT_EO_FLIP = 36,
    ICD_MAINT_LEN = 37
};

// s_icd_maintenance_status is one SHARED "current commanded values" snapshot -
// it does not remember each camera's own settings separately. The UI always
// sends a complete field set (not sparse deltas) whenever it targets a
// camera, so on every accepted maintenance command we additionally mirror the
// IR/EO field spans into that camera's own slot below (or all 6 slots for a
// broadcast/"All Cameras" command). This lets a later Maintenance-mode entry
// report each camera's own last-applied values instead of one shared one.
//
// IR fields span offsets [ICD_MAINT_IR_NUC .. ICD_MAINT_IR_TNR_H] - this
// deliberately excludes offset 0 (Selected IR Cam ID), which is a selector,
// not a per-camera setting.
#define ICD_MAINT_IR_FIELD_OFF  ICD_MAINT_IR_NUC
#define ICD_MAINT_IR_FIELD_LEN  (ICD_MAINT_IR_TNR_H - ICD_MAINT_IR_NUC + 1u)
// EO fields span offsets [ICD_MAINT_EO_POWER_RESET .. ICD_MAINT_EO_FLIP],
// excluding offset 15 (Selected EO Cam ID) for the same reason.
#define ICD_MAINT_EO_FIELD_OFF  ICD_MAINT_EO_POWER_RESET
#define ICD_MAINT_EO_FIELD_LEN  (ICD_MAINT_EO_FLIP - ICD_MAINT_EO_POWER_RESET + 1u)

static uint8_t s_icd_ir_cam_maint[6][ICD_MAINT_LEN];
static uint8_t s_icd_eo_cam_maint[6][ICD_MAINT_LEN];

// ---- Parameter/config protocol ----
#define CFG_BANK_EO      0x00
#define CFG_BANK_IR      0x01
#define CFG_CAM_COUNT    6
#define CFG_BANK_PAYLOAD_SIZE 42u
#define CFG_DEFAULT_MODE_EO_PANORAMA 0x15u

// ---- Orientation safety clamp (hard limits, not artificial) ----
#define ORIENT_CLAMP_DEG      (90.0f)
#define IMU_STREAM_BURST_MAX  16u
#define UI_ORIENT_DECIM       10u
#define LEGACY_IMU_UI_STREAM_ENABLE 0u
#define LOG_CTRL_CLEAR        0x01u
#define LOG_CTRL_SAVE         0x02u
#define LOG_CAPTURE_SAVE_DELAY_MS 45000u
#define FPGA_DEBUG_TOP_MODE_ONLY 0u
#define FW_VERSION_MAJOR 1u
#define FW_VERSION_MINOR 0u
#define FW_VERSION_YEAR  26u
#define FW_VERSION_MONTH 6u
#define FW_VERSION_DAY   17u

// ---- IMU command packets (matching ATMEL IIM4623x reference exactly) ----
// WHO_AM_I = ReadRegister(addr=0x00, len=1, page=0)
// IIM4623x_SetCMD_ReadRegister: SIZE_CMD_READ_REGS=12, checksum=sum(bytes[3..7])
//   bytes[3..7] = {0x11, 0x00, 0x01, 0x00, 0x00} -> checksum = 0x0012
static uint8_t who_am_i_packet[] = {
    0x24, 0x24,       // header
    0x0C,             // total size = 12
    0x11,             // CMD_TYPE_READ_REG
    0x00,             // reserved
    0x01,             // length = 1 byte
    0x00,             // first_addr = 0x00 (WHO_AM_I register)
    0x00,             // page_id = 0
    0x00, 0x12,       // checksum big-endian (0x11+0x00+0x01+0x00+0x00 = 0x12)
    0x0D, 0x0A        // footer
};

// START_STREAMING = Common command (cmd_type=0x27)
// IIM4623x_SetCMD_Common: SIZE_CMD_COMMON=8, checksum=sum(bytes[3..3])
//   bytes[3] = 0x27 -> checksum = 0x0027
static uint8_t start_streaming_packet[] = {
    0x24, 0x24,       // header
    0x08,             // total size = 8
    0x27,             // CMD_TYPE_START_STREAMING
    0x00, 0x27,       // checksum big-endian (0x27)
    0x0D, 0x0A        // footer
};

// STOP_STREAMING (cmd_type=0x28) is handled by IMU_SoftReset() in main.c


static volatile int16_t s_roll_cd;
static volatile int16_t s_pitch_cd;
static volatile float s_imu_temp_c;
static volatile int16_t s_rate_hz;
static volatile bool s_vsync_pending = false;
static uint16_t s_orient_tx_seq = 0;
static uint32_t s_odr_accum_us = 0;
static uint16_t s_odr_samples = 0;
static uint8_t s_stream_valid_log_sent = 0u;
static uint8_t s_stream_odr_log_sent = 0u;
static uint8_t s_stream_diag_count = 0;
static uint8_t s_prev_sample_counter = 0;
static uint64_t s_prev_timestamp_us = 0;
static uint16_t s_ui_throttle = 0u;
static uint32_t s_stats_window_start_ms = 0u;
static uint32_t s_stats_edges_base = 0u;
static uint32_t s_stats_parse_ok = 0u;
static uint32_t s_stats_ts_delta_us = 0u;
static uint32_t s_stats_ts_delta_count = 0u;
static float s_roll_deg_filt = 0.0f;
static float s_pitch_deg_filt = 0.0f;
static bool s_orient_filter_init = false;

/* -------------------------------------------------------------------------- */
/* Light output-stage smoothing and 0.1 degree quantization.                  */
/*                                                                             */
/* Madgwick IS the filter. Its quaternion integrator already smooths the      */
/* gyro + accel fusion ??we do not need a second software filter on top.      */
/* A very small EMA here targets only the final visible flutter and is tuned  */
/* to stay well below 10 ms of added lag.                                     */
/*                                                                             */
/* Output behavior:                                                            */
/*   - EMA time constant is about 5 ms                                        */
/*   - Rounds centidegrees to the nearest 10 cd (= 0.1 degree)                */
/*   - Any residual flutter smaller than +-5 cd (+-0.05 degree) is absorbed   */
/* -------------------------------------------------------------------------- */
#define QUANT_STEP_CD            10   /* 0.1 degree = 10 centidegrees */
#define ORIENT_SMOOTH_TAU_MS      10.0f

/**
 * @brief Round val_cd to the nearest QUANT_STEP_CD increment.
 *
 * Works correctly for positive and negative values.
 * Example: 137 cd ??125 cd,  138 cd ??150 cd,  -137 cd ??-125 cd
 */
static inline int16_t quantize_cd(int16_t val_cd)
{
    /* Add half-step with correct sign for rounding, then truncate */
    int16_t half = QUANT_STEP_CD / 2;
    if (val_cd >= 0) {
        return (int16_t)(((val_cd + half) / QUANT_STEP_CD) * QUANT_STEP_CD);
    } else {
        return (int16_t)(((val_cd - half) / QUANT_STEP_CD) * QUANT_STEP_CD);
    }
}

typedef struct {
    uint16_t yaw_cd[CFG_CAM_COUNT];
    int16_t pitch_cd[CFG_CAM_COUNT];
    int16_t roll_cd[CFG_CAM_COUNT];
    uint16_t overlap_px;
    int32_t feather_strength_q16;
} ParamBankQ16;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t crc32;
    uint8_t mode;
    uint8_t reserved0[3];
    ParamBankQ16 eo;
    ParamBankQ16 ir;
    uint32_t write_count;
    uint32_t reserved1;
} ParamStoreBlob;

#define PARAM_STORE_MAGIC      0x5041524Du /* "PARM" */
#define PARAM_STORE_VERSION    0x0003u
#define PARAM_FLASH_PAGE_COUNT 1u

enum {
    CFG_APPLY_OK = 0,
    CFG_APPLY_ERR_BADARG = 1,
    CFG_APPLY_ERR_I2C_WRITE = 2,
    CFG_APPLY_ERR_I2C_READ = 3,
    CFG_APPLY_ERR_I2C_VERIFY = 4,
    CFG_APPLY_ERR_FLASH = 5
};

static ParamStoreBlob s_cfg_store;

static inline int32_t float_to_q16_sat(float v)
{
    double scaled = (double)v * 65536.0;
    if (scaled > (double)INT32_MAX) return INT32_MAX;
    if (scaled < (double)INT32_MIN) return INT32_MIN;
    return (int32_t)lround(scaled);
}

static uint32_t crc32_sw(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t cfg_crc_payload(const ParamStoreBlob* cfg)
{
    const uint8_t* start = (const uint8_t*)&cfg->mode;
    const size_t size = sizeof(ParamStoreBlob) - offsetof(ParamStoreBlob, mode);
    return crc32_sw(start, size);
}

static uint32_t cfg_flash_address(void)
{
    uint32_t flash_kb = (uint32_t)(*((uint16_t*)FLASHSIZE_BASE));
    uint32_t flash_bytes = flash_kb * 1024u;
    return FLASH_BASE + flash_bytes - FLASH_PAGE_SIZE;
}

static bool cfg_mode_valid(uint8_t mode)
{
    if (mode == 0x00u || mode == 0x01u || mode == 0x02u || mode == 0x03u || mode == 0x06u || mode == 0x13u || mode == 0x14u || mode == 0x15u) {
        return true;
    }
    if (mode >= 0x07u && mode <= 0x12u) {
        return true;
    }
    return false;
}

static bool cfg_mode_uses_vsync(uint8_t mode)
{
    return (mode == 0x01u || mode == 0x02u);
}

static void cfg_set_bank_defaults(ParamBankQ16* bank, uint8_t which)
{
    if (which == CFG_BANK_IR) {
        static const uint16_t kIrYawCd[CFG_CAM_COUNT]   = {  9000u, 15000u, 21000u, 27000u, 33000u, 39000u };
        static const int16_t  kIrPitchCd[CFG_CAM_COUNT] = {     0,     0,     0,     0,     0,     0 };
        static const int16_t  kIrRollCd[CFG_CAM_COUNT]  = {     0,     0,     0,     0,     0,     0 };
        memcpy(bank->yaw_cd, kIrYawCd, sizeof(kIrYawCd));
        memcpy(bank->pitch_cd, kIrPitchCd, sizeof(kIrPitchCd));
        memcpy(bank->roll_cd, kIrRollCd, sizeof(kIrRollCd));
        bank->overlap_px = 30u;
        bank->feather_strength_q16 = float_to_q16_sat(1.0f);
        return;
    }

    static const uint16_t kEoYawCd[CFG_CAM_COUNT]   = {     0u,  6000u, 12000u, 18000u, 24000u, 30000u };
    static const int16_t  kEoPitchCd[CFG_CAM_COUNT] = {  -500,   500,  -500,   500,  -500,   500 };
    static const int16_t  kEoRollCd[CFG_CAM_COUNT]  = {     0,     0,     0,     0,     0,     0 };
    memcpy(bank->yaw_cd, kEoYawCd, sizeof(kEoYawCd));
    memcpy(bank->pitch_cd, kEoPitchCd, sizeof(kEoPitchCd));
    memcpy(bank->roll_cd, kEoRollCd, sizeof(kEoRollCd));
    bank->overlap_px = 100u;
    bank->feather_strength_q16 = float_to_q16_sat(1.0f);
}

static void cfg_set_defaults(ParamStoreBlob* cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = PARAM_STORE_MAGIC;
    cfg->version = PARAM_STORE_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);
    cfg->mode = CFG_DEFAULT_MODE_EO_PANORAMA;
    cfg_set_bank_defaults(&cfg->eo, CFG_BANK_EO);
    cfg_set_bank_defaults(&cfg->ir, CFG_BANK_IR);
    cfg->write_count = 0;
    cfg->crc32 = cfg_crc_payload(cfg);
}

static bool cfg_bank_values_valid(const ParamBankQ16* bank)
{
    if (!bank) return false;

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (bank->pitch_cd[i] < -500 || bank->pitch_cd[i] > 500) return false;
        if (bank->roll_cd[i] < -500 || bank->roll_cd[i] > 500) return false;
    }

    if (bank->overlap_px > 1920u) return false;

    const int32_t min_feather_q16 = float_to_q16_sat(0.1f);
    const int32_t max_feather_q16 = float_to_q16_sat(10.0f);
    if (bank->feather_strength_q16 < min_feather_q16 || bank->feather_strength_q16 > max_feather_q16) return false;

    return true;
}

static bool cfg_is_valid(const ParamStoreBlob* cfg)
{
    if (cfg->magic != PARAM_STORE_MAGIC) return false;
    if (cfg->version != PARAM_STORE_VERSION) return false;
    if (cfg->length != (uint16_t)sizeof(*cfg)) return false;
    if (!cfg_mode_valid(cfg->mode)) return false;
    if (!cfg_bank_values_valid(&cfg->eo)) return false;
    if (!cfg_bank_values_valid(&cfg->ir)) return false;
    return (cfg->crc32 == cfg_crc_payload(cfg));
}

static bool cfg_load_from_flash(void)
{
    const uint32_t base = cfg_flash_address();
    ParamStoreBlob flash_cfg;
    memcpy(&flash_cfg, (const void*)base, sizeof(flash_cfg));
    if (cfg_is_valid(&flash_cfg)) {
        s_cfg_store = flash_cfg;
        return true;
    }
    cfg_set_defaults(&s_cfg_store);
    return false;
}

static bool cfg_save_to_flash(void)
{
    s_cfg_store.magic = PARAM_STORE_MAGIC;
    s_cfg_store.version = PARAM_STORE_VERSION;
    s_cfg_store.length = (uint16_t)sizeof(s_cfg_store);
    s_cfg_store.crc32 = cfg_crc_payload(&s_cfg_store);

    const uint32_t base = cfg_flash_address();
    const uint32_t page = (base - FLASH_BASE) / FLASH_PAGE_SIZE;

    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = page;
    erase.NbPages = PARAM_FLASH_PAGE_COUNT;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return false;
    }

    const uint8_t* src = (const uint8_t*)&s_cfg_store;
    for (uint32_t off = 0; off < (uint32_t)sizeof(s_cfg_store); off += 8u) {
        uint64_t word = 0xFFFFFFFFFFFFFFFFull;
        memcpy(&word, src + off, 8u);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, base + off, word) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();

    ParamStoreBlob verify;
    memcpy(&verify, (const void*)base, sizeof(verify));
    return cfg_is_valid(&verify) && (memcmp(&verify, &s_cfg_store, sizeof(verify)) == 0);
}

static ParamBankQ16* cfg_bank_mut(uint8_t bank)
{
    return (bank == CFG_BANK_IR) ? &s_cfg_store.ir : &s_cfg_store.eo;
}

static const ParamBankQ16* cfg_bank_const(uint8_t bank)
{
    return (bank == CFG_BANK_IR) ? &s_cfg_store.ir : &s_cfg_store.eo;
}

static uint16_t read_le_u16_local(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_le_i16_local(const uint8_t* p)
{
    return (int16_t)read_le_u16_local(p);
}

static void put_le_u16_local(uint8_t* dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le_i16_local(uint8_t* dst, int16_t v)
{
    put_le_u16_local(dst, (uint16_t)v);
}

static bool cfg_bank_from_payload(const uint8_t* p, uint16_t n, ParamBankQ16* out)
{
    if (!p || !out) return false;
    if (n < CFG_BANK_PAYLOAD_SIZE) return false;

    uint16_t off = 0u;
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->yaw_cd[i] = read_le_u16_local(&p[off]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->pitch_cd[i] = read_le_i16_local(&p[off]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->roll_cd[i] = read_le_i16_local(&p[off]);
    }
    out->overlap_px = read_le_u16_local(&p[off]);
    off += 2u;
    out->feather_strength_q16 = (int32_t)(
        ((uint32_t)p[off + 0u]) |
        ((uint32_t)p[off + 1u] << 8) |
        ((uint32_t)p[off + 2u] << 16) |
        ((uint32_t)p[off + 3u] << 24));
    return true;
}

static void cfg_bank_copy_aligned(ParamBankQ16* dst, const ParamBankQ16* src)
{
    memcpy(dst, src, sizeof(*dst));
}

static bool fpga_write_verify_reg8(uint8_t reg, uint8_t value, uint8_t* detail_out)
{
    uint8_t rd = 0;
    HAL_StatusTypeDef st = fpga_write_reg8(&g_fpga, reg, value, 200);
    if (st != HAL_OK) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_WRITE;
        return false;
    }
    st = fpga_read_reg8(&g_fpga, reg, &rd, 200);
    if (st != HAL_OK) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_READ;
        return false;
    }
    if (rd != value) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_VERIFY;
        return false;
    }
    if (detail_out) *detail_out = CFG_APPLY_OK;
    return true;
}

static bool fpga_write_verify_i32(uint8_t reg, int32_t value, uint8_t* detail_out)
{
    uint8_t b[4];
    b[0] = (uint8_t)(value & 0xFF);
    b[1] = (uint8_t)((value >> 8) & 0xFF);
    b[2] = (uint8_t)((value >> 16) & 0xFF);
    b[3] = (uint8_t)((value >> 24) & 0xFF);

    for (uint8_t i = 0; i < 4u; ++i) {
        if (!fpga_write_verify_reg8((uint8_t)(reg + i), b[i], detail_out)) {
            return false;
        }
    }
    return true;
}

static bool fpga_write_verify_u16(uint8_t reg, uint16_t value, uint8_t* detail_out)
{
    uint8_t b[2];
    uint8_t rd[2] = {0};

    b[0] = (uint8_t)(value & 0xFFu);
    b[1] = (uint8_t)((value >> 8) & 0xFFu);

    for (uint8_t i = 0; i < 2u; ++i) {
        HAL_StatusTypeDef st = fpga_write_reg8(&g_fpga, (uint8_t)(reg + i), b[i], 200);
        if (st != HAL_OK) {
            if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_WRITE;
            return false;
        }
    }

    for (uint8_t i = 0; i < 2u; ++i) {
        HAL_StatusTypeDef st = fpga_read_reg8(&g_fpga, (uint8_t)(reg + i), &rd[i], 200);
        if (st != HAL_OK) {
            if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_READ;
            return false;
        }
    }

    if (memcmp(b, rd, sizeof(b)) != 0) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_VERIFY;
        return false;
    }
    return true;
}

static uint8_t fpga_push_bank(uint8_t bank, const ParamBankQ16* cfg)
{
    const uint8_t base = fpga_cfg_bank_base(bank);
    uint8_t detail = CFG_APPLY_OK;
    ParamBankQ16 local_cfg;

    cfg_bank_copy_aligned(&local_cfg, cfg);

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_YAW_CD(i)), local_cfg.yaw_cd[i], &detail)) return detail;
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_PITCH_CD(i)), (uint16_t)local_cfg.pitch_cd[i], &detail)) return detail;
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_ROLL_CD(i)), (uint16_t)local_cfg.roll_cd[i], &detail)) return detail;
    }
    if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_OVERLAP_U16), local_cfg.overlap_px, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_FEATHER_Q16), local_cfg.feather_strength_q16, &detail)) return detail;

    return CFG_APPLY_OK;
}

static uint8_t fpga_push_mode(uint8_t mode)
{
    uint8_t detail = CFG_APPLY_OK;
    if (!fpga_write_verify_reg8(FPGA_REG_MODE_CURRENT, mode, &detail)) return detail;
    return CFG_APPLY_OK;
}

static uint8_t fpga_push_all_runtime_config(void)
{
    uint8_t detail = fpga_push_mode(s_cfg_store.mode);
    if (detail != CFG_APPLY_OK) return detail;
    detail = fpga_push_bank(CFG_BANK_EO, &s_cfg_store.eo);
    if (detail != CFG_APPLY_OK) return detail;
    detail = fpga_push_bank(CFG_BANK_IR, &s_cfg_store.ir);
    return detail;
}



// ===================== UI TX =====================
static void ui_send_frame(uint8_t msg_type, uint8_t seq, const uint8_t *payload, uint16_t plen)
{
#if ICD_ONLY_UI_PROTOCOL
    (void)msg_type;
    (void)seq;
    (void)payload;
    (void)plen;
    return;
#else
    uint8_t buf[UI_TX_MAX];
    uint8_t tmp[UI_PAY_MAX];

    // Inner payload = [type][seq][lenL][lenH][payload...]
    if ((uint32_t)plen + 4u > sizeof(tmp)) return;

    tmp[0] = msg_type;
    tmp[1] = seq;
    tmp[2] = (uint8_t)(plen & 0xFF);
    tmp[3] = (uint8_t)((plen >> 8) & 0xFF);
    if (plen) memcpy(&tmp[4], payload, plen);

    size_t frame_len = uart_frame_build(tmp, (size_t)(plen + 4u), buf, sizeof(buf));
    if (frame_len) {
        (void)HAL_UART_Transmit(&huart3, buf, (uint16_t)frame_len, 50);
    }
#endif
}

static void ui_send_resp_simple(uint8_t seq, uint8_t err)
{
    uint8_t p[1] = { err };
    ui_send_frame(MSG_RESP, seq, p, sizeof(p));
}

static void ui_send_version(uint8_t seq)
{
    uint8_t p[9] = {
        ERR_OK,
        NODE_SYSTEM,
        0xFEu,
        1u,
        FW_VERSION_MAJOR,
        FW_VERSION_MINOR,
        FW_VERSION_YEAR,
        FW_VERSION_MONTH,
        FW_VERSION_DAY
    };
    ui_send_frame(MSG_RESP, seq, p, sizeof(p));
}

static int camera_power_set(uint8_t node_type, uint8_t id, uint8_t on)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0u;

    if (id >= 6u) return -1;

    if (node_type == NODE_DAY) {
        switch (id) {
        case 0u: port = GPIOC; pin = GPIO_PIN_11; break;
        case 1u: port = GPIOC; pin = GPIO_PIN_12; break;
        case 2u: port = GPIOC; pin = GPIO_PIN_13; break;
        case 3u: port = GPIOD; pin = GPIO_PIN_5;  break;
        case 4u: port = GPIOD; pin = GPIO_PIN_4;  break;
        case 5u: port = GPIOD; pin = GPIO_PIN_3;  break;
        default: return -1;
        }
    } else if (node_type == NODE_IR) {
        switch (id) {
        case 0u: port = GPIOD; pin = GPIO_PIN_1;  break;
        case 1u: port = GPIOD; pin = GPIO_PIN_0;  break;
        case 2u: port = GPIOC; pin = GPIO_PIN_9;  break;
        case 3u: port = GPIOC; pin = GPIO_PIN_8;  break;
        case 4u: port = GPIOA; pin = GPIO_PIN_15; break;
        case 5u: port = GPIOA; pin = GPIO_PIN_8;  break;
        default: return -1;
        }
    } else {
        return -1;
    }

    HAL_GPIO_WritePin(port, pin, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (node_type == NODE_DAY) {
        s_day_power_on[id] = (on != 0u) ? 1u : 0u;
    } else if (node_type == NODE_IR) {
        s_ir_power_on[id] = (on != 0u) ? 1u : 0u;
    }
    return 0;
}

static void ui_send_camera_apply_result(uint8_t seq,
                                        uint8_t node_type,
                                        uint8_t camera_id,
                                        uint16_t param_id,
                                        uint16_t value,
                                        uint8_t ok,
                                        uint8_t err)
{
    uint8_t resp[8];
    resp[0] = err;
    resp[1] = node_type;
    resp[2] = camera_id;
    resp[3] = ok;
    resp[4] = (uint8_t)(param_id & 0xFF);
    resp[5] = (uint8_t)((param_id >> 8) & 0xFF);
    resp[6] = (uint8_t)(value & 0xFF);
    resp[7] = (uint8_t)((value >> 8) & 0xFF);
    ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
}

static void ui_send_log(McuContext *ctx, const char *s)
{
    if (!ctx->debug_enabled) return;
    uint16_t n = (uint16_t)strlen(s);
    if (n > 200) n = 200;
    ui_send_frame(MSG_LOG, 0, (const uint8_t*)s, n);
}

/* Public version ??callable from main.c and any other translation unit.
 * Accepts a printf-style format so callers don't need a local snprintf.
 * Usage:  mcu_ui_log(&g_ctx, "Verify Genlock: NV17=%u NV18=%u\r\n", nv17, nv18); */
void mcu_ui_log(McuContext *ctx, const char *fmt, ...)
{
    if (!ctx || !ctx->debug_enabled) return;
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uint16_t n = (uint16_t)strlen(buf);
    if (n > 200) n = 200;
    ui_send_frame(MSG_LOG, 0, (const uint8_t*)buf, n);
}

static void ui_send_init_ack(uint8_t seq, uint8_t node, uint8_t id, uint8_t ok, uint8_t err)
{
    uint8_t resp[4] = { err, node, id, ok };
    ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
}

// ---- IMU orientation event (roll + pitch in centidegrees, int16 LE) ----
// Frame payload: [EVT_IMU_ORIENT][roll_lo][roll_hi][pitch_lo][pitch_hi]
static void ui_send_imu_orient(int16_t roll_cd, int16_t pitch_cd)
{
    uint8_t p[5];
    p[0] = EVT_IMU_ORIENT;
    p[1] = (uint8_t)( roll_cd        & 0xFF);
    p[2] = (uint8_t)((roll_cd  >> 8) & 0xFF);
    p[3] = (uint8_t)( pitch_cd       & 0xFF);
    p[4] = (uint8_t)((pitch_cd >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_imu_odr(uint16_t rate_hz)
{
    uint8_t p[3];
    p[0] = EVT_IMU_ODR;
    p[1] = (uint8_t)(rate_hz & 0xFF);
    p[2] = (uint8_t)((rate_hz >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_imu_stats(uint16_t drdy_hz, uint16_t pkt_ts_hz, uint16_t parse_ok_hz)
{
    uint8_t p[7];
    p[0] = EVT_IMU_STATS;
    p[1] = (uint8_t)(drdy_hz & 0xFF);
    p[2] = (uint8_t)((drdy_hz >> 8) & 0xFF);
    p[3] = (uint8_t)(pkt_ts_hz & 0xFF);
    p[4] = (uint8_t)((pkt_ts_hz >> 8) & 0xFF);
    p[5] = (uint8_t)(parse_ok_hz & 0xFF);
    p[6] = (uint8_t)((parse_ok_hz >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void put_le_i32(uint8_t* dst, int32_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void ui_send_cfg_bank(uint8_t bank)
{
    ParamBankQ16 cfg_local;
    cfg_bank_copy_aligned(&cfg_local, cfg_bank_const(bank));
    uint8_t p[3 + CFG_BANK_PAYLOAD_SIZE];
    uint16_t off = 3u;
    p[0] = EVT_CFG_BANK;
    p[1] = bank;
    p[2] = s_cfg_store.mode;

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_u16_local(&p[off], cfg_local.yaw_cd[i]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_i16_local(&p[off], cfg_local.pitch_cd[i]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_i16_local(&p[off], cfg_local.roll_cd[i]);
    }
    put_le_u16_local(&p[off], cfg_local.overlap_px);
    off += 2u;
    put_le_i32(&p[off], cfg_local.feather_strength_q16);

    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_cfg_apply_result(uint8_t bank, uint8_t ok, uint8_t detail)
{
    uint8_t p[4];
    p[0] = EVT_CFG_APPLY;
    p[1] = bank;
    p[2] = ok;
    p[3] = detail;
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_log_ctrl(uint8_t cmd)
{
    uint8_t p[2];
    p[0] = EVT_LOG_CTRL;
    p[1] = cmd;
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void mcu_service_log_capture(McuContext *ctx)
{
    if (ctx == NULL || !ctx->log_capture_active || ctx->log_capture_save_sent) {
        return;
    }

    if ((HAL_GetTick() - ctx->log_capture_start_ms) >= LOG_CAPTURE_SAVE_DELAY_MS) {
        ui_send_log_ctrl(LOG_CTRL_SAVE);
        ctx->log_capture_save_sent = true;
    }
}

static float orient_output_alpha(void)
{
    float rate_hz = (s_rate_hz > 0) ? (float)s_rate_hz : 1000.0f;
    float dt_ms = 1000.0f / rate_hz;
    float alpha = dt_ms / (ORIENT_SMOOTH_TAU_MS + dt_ms);

    if (alpha < 0.02f) alpha = 0.02f;
    if (alpha > 1.0f) alpha = 1.0f;
    return alpha;
}


// ===================== FPGA helpers =====================
//static int fpga_write16(uint16_t reg, const uint8_t *data, uint8_t len)
//{
//    return fpga_write(reg, data, len);
//}

static int fpga_rw_smoke_test(void)
{
  uint8_t rd = 0;
  const uint8_t wr = 0xA5;
  HAL_StatusTypeDef st;

  st = fpga_is_ready(&g_fpga, 2u, 50u);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA ready fail HAL=%d I2Cerr=0x%08lX addr7=0x%02X\r\n",
                     (int)st, (unsigned long)e, (unsigned)g_fpga.addr7);
    return -10;
  }

  st = fpga_write_reg8(&g_fpga, FPGA_REG_MODE_CURRENT, wr, 200);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA write fail HAL=%d I2Cerr=0x%08lX reg=0x%02X wr=0x%02X\r\n",
                     (int)st, (unsigned long)e, FPGA_REG_MODE_CURRENT, wr);
    return -11;
  }

  st = fpga_read_reg8(&g_fpga, FPGA_REG_MODE_CURRENT, &rd, 200);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA read fail HAL=%d I2Cerr=0x%08lX reg=0x%02X\r\n",
                     (int)st, (unsigned long)e, FPGA_REG_MODE_CURRENT);
    return -12;
  }

  Send_UART3_Debug("FPGA smoke OK wr=0x%02X rd=0x%02X\r\n", wr, rd);
  return (rd == wr) ? 0 : -2;
}

// ===================== SAFE NODE STATUS CHECKS =====================
// These are handshake/liveness checks ONLY. They must never re-initialize a node
// (no IMU_Reset, no FPGA register writes, no IR FactoryResetNV/NV reconfigure),
// so they can run periodically (e.g. the 30 s UI status poll) without disturbing
// live peripherals or tripping the power IC.

// IMU: g_imu_drdy_edges_total is incremented by the IMU DRDY EXTI ISR (~1 kHz
// while healthy). We watch it advance; if it stalls for longer than the timeout,
// the IMU data has stopped. No SPI traffic, no reset.
#define IMU_STALE_TIMEOUT_MS 250u
static uint32_t s_imu_fresh_last_edges = 0u;
static uint32_t s_imu_fresh_last_ms    = 0u;
static volatile bool s_imu_data_stale  = true;   // assume stale until proven fresh

static void imu_freshness_monitor(void)
{
    uint32_t now   = HAL_GetTick();
    uint32_t edges = g_imu_drdy_edges_total;

    if (edges != s_imu_fresh_last_edges) {
        s_imu_fresh_last_edges = edges;
        s_imu_fresh_last_ms    = now;
        s_imu_data_stale       = false;
    } else if ((uint32_t)(now - s_imu_fresh_last_ms) > IMU_STALE_TIMEOUT_MS) {
        s_imu_data_stale = true;
    }
}

static bool imu_status_ok(void)
{
    return !s_imu_data_stale;
}

// FPGA: pure I2C address-ACK probe. Confirms the FPGA's I2C slave is alive with
// no register write (the old smoke test clobbered FPGA_REG_MODE_CURRENT).
static bool fpga_status_ok(void)
{
    return (fpga_is_ready(&g_fpga, 2u, 50u) == HAL_OK);
}


// ===================== FSM =====================
void mcu_fsm_init(McuContext *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = ST_BOOT;
    ctx->debug_enabled = true;
    g_debug_enabled = true;
    ctx->log_capture_active = false;
    ctx->log_capture_save_sent = false;
    ctx->log_capture_start_ms = 0u;

    const bool loaded = cfg_load_from_flash();
    if (!loaded) {
        (void)cfg_save_to_flash();
    }
    /* Bring-up always starts in EO panorama. Keep loaded EO/IR bank calibration,
     * but do not let a previously saved runtime mode override startup. */
    s_cfg_store.mode = CFG_DEFAULT_MODE_EO_PANORAMA;
    if (loaded) {
        /* Keep the persisted config aligned with the boot-time default so
         * any later runtime re-apply also comes up in EO panorama. */
        (void)cfg_save_to_flash();
    }

    icd_maintenance_init_defaults();
    s_icd_enabled = true;
    s_icd_operation_mode = ICD_OPERATION_STANDBY;
    s_icd_last_cbit_ms = 0u;
    s_icd_last_status_ms = 0u;
    s_icd_initial_pbit_sent = false;
    s_icd_time_valid = false;
    s_icd_time_epoch_ms = 0u;
    s_icd_time_base_ms = HAL_GetTick();
    s_icd_seq_status = 1u;
    s_icd_seq_maintenance_status = 1u;
    s_icd_seq_pbit = 1u;
    s_icd_seq_cbit = 1u;
    s_icd_seq_ibit = 1u;
    s_icd_seq_mode_ack = 1u;
    s_icd_seq_camera_ack = 1u;
    s_icd_seq_panorama_ack = 1u;
    s_icd_seq_maintenance_ack = 1u;
    s_icd_seq_default = 1u;
    memset(s_ir_nuc_status_until_ms, 0, sizeof(s_ir_nuc_status_until_ms));
    s_icd_video_brightness = 0u;
    s_icd_video_contrast = 0u;
    s_icd_ir_image_polarity = 0u;
    s_icd_video_stabilization = 0u;
    s_icd_video_stabilization_method = 0u;
    s_icd_blending_area_px = 0u;
    s_vsync_pending = false;
    s_orient_tx_seq = 0;
}

void mcu_fsm_start_init(McuContext *ctx)
{
    if (ctx == NULL) return;

    // ctx->init_seq is already set by the caller (CMD_INIT_SYSTEM handler)
    // to the UI's sequence number ??do NOT reset it here.
    ctx->day_idx = 0u;
    ctx->ir_idx = 0u;
    ctx->imu_streaming = false;
    ctx->log_capture_active = false;
    ctx->log_capture_save_sent = false;
    ctx->log_capture_start_ms = 0u;
    ctx->state = ST_BOOT;
    // No extra ACK here ??the caller (CMD_INIT_SYSTEM handler) already sent it
}


void mcu_fsm_step(McuContext *ctx)
{
    mcu_service_log_capture(ctx);
    imu_freshness_monitor();   // keep the IMU data-stale flag current every step

	switch (ctx->state) {




    case ST_WAIT_UI_INIT: {
    	ctx->state = ST_BOOT;
        break;
    }


    case ST_BOOT: {
        ui_send_log_ctrl(LOG_CTRL_CLEAR);
        ctx->log_capture_active = true;
        ctx->log_capture_save_sent = false;
        ctx->log_capture_start_ms = HAL_GetTick();
        ctx->state = ST_INIT_POWER;
        break;
    }

    case ST_INIT_POWER: {
        Enable_All_Camera_Power();
        HAL_Delay(1000);
        ctx->state = ST_INIT_IMU;
        break;
    }

    case ST_INIT_IMU: {
        // ----------------------------------------------------------------
        // INIT SEQUENCE (batch 14):
        //   Reset ??WHO_AM_I ??Reset ??imu_configure_odr_lpf ??START_STREAM
        //
        // SPI fixes applied (batch 14):
        //   - spi2_flush() now re-asserts FRXTH=1 on every call
        //   - IMU_NSS_HIGH_US raised from 20 µs to 500 µs so the sensor
        //     has time to latch the command before DRDY is polled
        //   - Phase 2 DRDY also checks pin level, not just interrupt flag,
        //     to catch the race where the edge fires before the poll loop
        //
        // With these fixes multiple consecutive commands work, so ODR
        // and SELECT_OUT_DATA are written explicitly every boot ??we no
        // longer rely on whatever was saved to flash.
        // ----------------------------------------------------------------

        /* ---- First reset: clean boot ---- */
        IMU_Reset();
        HAL_Delay(10);
        /* ---- WHO_AM_I ??confirm sensor is alive ---- */
        uint8_t wai_rx[SIZE_RESP_READ_REGS_BASE + 2u] = {0};
        int r = send_imu_command_ex(who_am_i_packet, sizeof(who_am_i_packet),
                                    (uint32_t)(SIZE_RESP_READ_REGS_BASE + 1u), "WHO_AM_I",
                                    wai_rx);

        if (r == 0) {
            uint8_t wai_val = 0;
            for (int i = 0; i < (int)sizeof(wai_rx) - 1; i++) {
                if (wai_rx[i] == 0xEAu || wai_rx[i] == 0xE6u) {
                    wai_val = wai_rx[i];
                    break;
                }
            }
            char msg[80];
            snprintf(msg, sizeof(msg), "[IMU] WHO_AM_I=0x%02X (%s)\r\n", wai_val,
                     (wai_val == 0xEAu) ? "IIM46234 OK" :
                     (wai_val == 0xE6u) ? "IIM46230 OK" : "UNKNOWN");
            ui_send_log(ctx, msg);
        }

        uint8_t ok = (r == 0) ? 1 : 0;
        ui_send_init_ack(ctx->init_seq, NODE_IMU, 0, ok, ok ? ERR_OK : ERR_FAIL);
        if (r != 0) {
            ui_send_log(ctx, "[IMU] WHO_AM_I failed -- IMU offline\r\n");
            ctx->imu_streaming = false;
            ctx->day_idx = 0;
            ctx->state = ST_INIT_DAY;
            break;
        }

        float imu_odr_hz = 1000.0f;
        {
            int rc = imu_ensure_default_profile();
            if (rc == 0) {
                ui_send_log(ctx, "[IMU] Flash-backed default profile verified\r\n");
            } else {
                ui_send_log(ctx, "[IMU] WARN: Default profile verify/restore failed\r\n");
            }
        }

        {
            float t = 0.0f, h = 0.0f;
            for (int i = 0; i < 3; i++) {
                Read_HDC3022_Sensor(&t, &h);
                HAL_Delay(10);
            }

            uint8_t ok_temp = (t > -900.0f && h > -900.0f) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq, NODE_TEMP, 0, ok_temp,
                             ok_temp ? ERR_OK : ERR_FAIL);

            {
                int16_t  t_cd = (int16_t)lroundf(t * 100.0f);
                uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Temperature %.2f C, Humidity %.2f %%\r\n",
                         t_cd / 100.0f,
                         h_cp / 100.0f);
                Send_UART3_Debug(msg);
            }
        }

        /* This board still behaves reliably only when START_STREAM is the
         * first command after reset. Keep the final reset immediately before
         * sending START_STREAM. */
        IMU_Reset();
        HAL_Delay(10);

        IMU_Handler_Init(imu_odr_hz, 0.12f);
        s_stream_valid_log_sent = 0u;
        s_stream_odr_log_sent = 0u;
        s_stream_diag_count = 0u;
        s_prev_sample_counter = 0u;
        s_prev_timestamp_us = 0u;
        s_odr_accum_us = 0u;
        s_odr_samples = 0u;
        s_ui_throttle = 0u;
        s_roll_deg_filt = 0.0f;
        s_pitch_deg_filt = 0.0f;
        s_orient_filter_init = false;
        s_stats_window_start_ms = HAL_GetTick();
        s_stats_edges_base = g_imu_drdy_edges_total;
        s_stats_parse_ok = 0u;
        s_stats_ts_delta_us = 0u;
        s_stats_ts_delta_count = 0u;
        HAL_Delay(50u);

        int rs = send_imu_command(start_streaming_packet,
                                  sizeof(start_streaming_packet),
                                  0, "START_STREAM");

        if (rs == 0) {
            ctx->imu_streaming = true;
            ui_send_log(ctx, "[IMU] START_STREAM sent; waiting for first DRDY packet\r\n");
        } else {
            ctx->imu_streaming = false;
            uint8_t err_payload[2] = { NODE_IMU, ERR_FAIL };
            ui_send_frame(MSG_EVT, 0, err_payload, sizeof(err_payload));
            ui_send_log(ctx, "[IMU] START_STREAM failed\r\n");
        }

        ctx->state = ST_INIT_DAY;
        ui_send_log(ctx, "Entered OPERATION\r\n");
        break;
    }


    case ST_INIT_DAY: {
        if (ctx->day_idx < 6) {
            HAL_Delay(20);
            int r = test_day_camera_controls(ctx->day_idx);   // change it to return status if it doesn?�t
            uint8_t ok = (r == 0) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq,NODE_DAY, ctx->day_idx, ok, ok ? ERR_OK : ERR_FAIL);
            // Default Shutter Speed (CAM_Shutter, step 10 = "X2" digital slow
            // shutter, one step brighter than baseline 1/30 at step 11) at
            // boot. Unlike Brightness/ExpComp, Shutter Speed stays directly
            // settable even in MANUAL/External Trigger mode.
            (void)Apply_EO_Camera_Param((uint8_t)(ctx->day_idx + 1u), PARAM_ID_EO_AE_SHUTTER, 10u);
            ctx->day_idx++;
        } else {
            ctx->ir_idx = 0;
            ctx->state = ST_INIT_IR;
        }
        break;
    }


    case ST_INIT_IR: {
    	HAL_Delay(1000);
    	if (ctx->ir_idx < 6) {
        	int r = Initialize_IR_Camera(ctx->ir_idx);   // change to return status if it doesn?�t
            uint8_t ok = (r == 0) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq,NODE_IR, ctx->ir_idx, ok, ok ? ERR_OK : ERR_FAIL);

            ctx->ir_idx++;

        } else {
            ctx->state = ST_INIT_FPGA_CAL;
//        	ctx->state = ST_INIT_TEMP;
        }
        break;
    }


    case ST_INIT_FPGA_CAL: {
        ui_send_log(ctx, "FPGA init build: full-regmap diag v9\r\n");
        int st = fpga_rw_smoke_test();

        if (st == 0) {
            ui_send_log(ctx, "FPGA I2C ping OK (0xA5)\r\n");
            Send_UART3_Debug("FPGA I2C ping OK (0xA5)\r\n\n\n");
            uint8_t detail = CFG_APPLY_OK;
            uint32_t t0 = HAL_GetTick();
            uint8_t probe_detail = CFG_APPLY_OK;

            {
                static const uint8_t probe_regs[4]  = { 0x20u, 0x21u, 0x22u, 0x23u };
                static const uint8_t probe_vals[4]  = { 0x5Au, 0xA5u, 0x3Cu, 0xC3u };
                char msg[96];
                for (uint8_t i = 0; i < 4u; ++i) {
                    snprintf(msg, sizeof(msg), "FPGA probe reg 0x%02X\r\n", (unsigned)probe_regs[i]);
                    ui_send_log(ctx, msg);
                    if (!fpga_write_verify_reg8(probe_regs[i], probe_vals[i], &probe_detail)) {
                        snprintf(msg, sizeof(msg), "FPGA probe reg 0x%02X FAIL detail=%u\r\n",
                                 (unsigned)probe_regs[i], (unsigned)probe_detail);
                        ui_send_log(ctx, msg);
                        detail = probe_detail;
                        break;
                    }
                }
            }

            if (detail == CFG_APPLY_OK) {
                #if FPGA_DEBUG_TOP_MODE_ONLY
                ui_send_log(ctx, "FPGA bank push skipped (debug top)\r\n");
                ui_send_log(ctx, "FPGA push stage: mode\r\n");
                detail = fpga_push_mode(s_cfg_store.mode);
                #else
                /* Push the display mode FIRST so a later bank-config failure
                 * cannot leave the FPGA stuck on its power-on default (IR0).
                 * The mode register (0x00) is independent of the config banks
                 * (0x20+), so setting it first is safe. */
                ui_send_log(ctx, "FPGA push stage: mode\r\n");
                detail = fpga_push_mode(s_cfg_store.mode);

                if (detail == CFG_APPLY_OK) {
                    ui_send_log(ctx, "FPGA push stage: EO bank\r\n");
                    detail = fpga_push_bank(CFG_BANK_EO, &s_cfg_store.eo);
                }

                if (detail == CFG_APPLY_OK) {
                    ui_send_log(ctx, "FPGA push stage: IR bank\r\n");
                    detail = fpga_push_bank(CFG_BANK_IR, &s_cfg_store.ir);
                }
                #endif
            }

            if (detail == CFG_APPLY_OK) {
                char msg[96];
                snprintf(msg, sizeof(msg), "FPGA runtime config push OK (%lums)\r\n",
                         (unsigned long)(HAL_GetTick() - t0));
                ui_send_log(ctx, msg);
                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 1, ERR_OK);
            } else {
                char msg[96];
                snprintf(msg, sizeof(msg), "FPGA runtime config push FAIL detail=%u (%lums)\r\n",
                         (unsigned)detail,
                         (unsigned long)(HAL_GetTick() - t0));
                ui_send_log(ctx, msg);
                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
            }

        } else {
            ui_send_log(ctx, "FPGA I2C ping FAIL\r\n");
            Send_UART3_Debug("FPGA I2C ping FAIL\r\n");

             ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
        }

        // Do NOT say Entered OPERATION here; we are not in OPERATION yet.
        ctx->state = ST_INIT_TEMP;
        break;
    }

    case ST_INIT_TEMP: {
        float t = 0.0f, h = 0.0f;

        // Speedy init: do a few quick reads (or just 1). This avoids ~1s boot delay.
        // Keep 3 reads to stabilize without blocking too long.
        for (int i = 0; i < 3; i++) {
            Read_HDC3022_Sensor(&t, &h);
            HAL_Delay(10);
        }

        // basic validity check (matches your error behavior)
        uint8_t ok = (t > -900.0f && h > -900.0f) ? 1 : 0;

        // If you already have node ACK helper:
         ui_send_init_ack(ctx->init_seq,NODE_TEMP, 0, ok, ok ? ERR_OK : ERR_FAIL);

        // Optional debug print
        int16_t  t_cd = (int16_t)lroundf(t * 100.0f);
        uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);

        char msg[80];
        snprintf(msg, sizeof(msg),
                 "Temperature %.2f C, Humidity %.2f %%\r\n",
                 t_cd / 100.0f,
                 h_cp / 100.0f);
        Send_UART3_Debug(msg);

        IR_Nuc_OnOperationEnter();
        ctx->state = ST_OPERATION;
        s_icd_enabled = true;
        s_icd_last_cbit_ms = HAL_GetTick();
        s_icd_last_status_ms = HAL_GetTick();
        ui_send_init_ack(ctx->init_seq, NODE_SYSTEM, 1, 1, ERR_OK);
        ui_send_log(ctx, "Entered OPERATION\r\n");
        if (!s_icd_initial_pbit_sent) {
            icd_send_pbit(0u);
            s_icd_initial_pbit_sent = true;
        }
        break;
    }

    case ST_OPERATION: {
        icd_service_periodic(ctx);
        fpga_service_orient();
        IR_Nuc_Service(ctx);
        break;
    }

    case ST_ERROR:
    default: {
        break;
    }
    }
}

// ===================== IMU DRDY handler =====================
// Called from main loop for each pending DRDY edge counted from EXTI PB9.

static void quat_to_roll_pitch_deg(const Quaternion_t *q, float *roll_deg, float *pitch_deg)
{
    const float RAD2DEG = 180.0f / 3.14159265358979f;
    const float q0 = q->q0;
    const float q1 = q->q1;
    const float q2 = q->q2;
    const float q3 = q->q3;

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float sinp = 2.0f * (q0 * q2 - q3 * q1);

    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;
    *pitch_deg = asinf(sinp) * RAD2DEG;
}

static void update_shadow_roll_pitch(const Quaternion_t *q)
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float alpha = 1.0f;

    quat_to_roll_pitch_deg(q, &roll_deg, &pitch_deg);

    if (roll_deg  >  ORIENT_CLAMP_DEG) roll_deg  =  ORIENT_CLAMP_DEG;
    if (roll_deg  < -ORIENT_CLAMP_DEG) roll_deg  = -ORIENT_CLAMP_DEG;
    if (pitch_deg >  ORIENT_CLAMP_DEG) pitch_deg =  ORIENT_CLAMP_DEG;
    if (pitch_deg < -ORIENT_CLAMP_DEG) pitch_deg = -ORIENT_CLAMP_DEG;

    if (!s_orient_filter_init) {
        s_roll_deg_filt = roll_deg;
        s_pitch_deg_filt = pitch_deg;
        s_orient_filter_init = true;
    } else {
        alpha = orient_output_alpha();
        s_roll_deg_filt += alpha * (roll_deg - s_roll_deg_filt);
        s_pitch_deg_filt += alpha * (pitch_deg - s_pitch_deg_filt);
    }

    s_roll_cd = quantize_cd((int16_t)lroundf(s_roll_deg_filt * 100.0f));
    s_pitch_cd = quantize_cd((int16_t)lroundf(s_pitch_deg_filt * 100.0f));
}

/* Confirmed from log (2026-02-26, batch 11):
 * The sensor outputs 1 dummy byte (0x00) before the 46-byte stream packet.
 * Total SPI read = 47 bytes.  Packet starts at byte[1] (0x23 0x23 header).
 *   byte[0]       = 0x00  (dummy / SPI pipeline flush byte ??ignore)
 *   byte[1..2]    = 0x23 0x23  (stream packet header)
 *   byte[3]       = packet size byte (0x2E = 46)
 *   byte[4..46]   = remainder of 46-byte stream packet
 *
 * SELECT_OUT_DATA = 0x07 (accel+gyro+temp, 7 outputs):
 *   SIZE_PACKET_BASE_DATA(18) + 4×7 = 46 bytes payload
 */
#define IMU_STREAM_DUMMY_BYTES  1u               /* one byte of alignment slack */
#define IMU_STREAM_PKT_BYTES    46u              /* actual packet bytes */
#define IMU_STREAM_READ_TOTAL   (IMU_STREAM_DUMMY_BYTES + IMU_STREAM_PKT_BYTES)  /* 47 */

static uint8_t s_imu_rx_buf[IMU_STREAM_READ_TOTAL];

static int imu_stream_read_raw(uint8_t *dst)
{
    return IMU_SPI_ReadSession(dst, IMU_STREAM_READ_TOTAL, 10u);
}

static int mcu_process_imu_stream_packet(McuContext *ctx, uint8_t *delta_counter_out)
{
    if (delta_counter_out) *delta_counter_out = 0u;

    if (imu_stream_read_raw(s_imu_rx_buf) != 0) return -1;

    uint8_t *pkt = NULL;
    uint32_t pkt_avail = IMU_STREAM_PKT_BYTES;

    if (s_imu_rx_buf[0] == 0x23u && s_imu_rx_buf[1] == 0x23u) {
        pkt = &s_imu_rx_buf[0];
    } else if (s_imu_rx_buf[1] == 0x23u && s_imu_rx_buf[2] == 0x23u) {
        pkt = &s_imu_rx_buf[1];
    } else {
        return 0;
    }

    // ---- Parse packet + run Madgwick ----
    IMU_Sample_t sample;
    int parse_result = IMU_Handler_ProcessStreamPacket(pkt, pkt_avail, &sample);
    if (parse_result != IMU_ERR_OK) {
        /* Discard silently ??no Send_UART3_Debug here, blocking UART kills 1kHz */
        return 0;
    }

    s_stats_parse_ok++;
    s_imu_temp_c = sample.temperature;

    if (!s_stream_valid_log_sent) {
        s_stream_valid_log_sent = 1u;
#if LEGACY_IMU_UI_STREAM_ENABLE
        ui_send_log(ctx, "[IMU] First stream packet parsed\r\n");
#endif
    }

    uint64_t timestamp_us =
        ((uint64_t)sample.timestamp_high << 32) | (uint64_t)sample.timestamp_low;
    uint8_t delta_counter = 0u;
    uint32_t delta_us = 0u;

    if (s_prev_timestamp_us != 0u) {
        delta_counter = (uint8_t)(sample.sample_counter - s_prev_sample_counter);
        delta_us = (uint32_t)(timestamp_us - s_prev_timestamp_us);
        if (delta_counter > 0u && delta_us > 0u && delta_us < 1000000u) {
            s_stats_ts_delta_us += delta_us;
            s_stats_ts_delta_count += delta_counter;
            uint32_t per_sample_us = delta_us / (uint32_t)delta_counter;
            if (per_sample_us > 0u) {
                s_odr_accum_us += per_sample_us;
                s_odr_samples++;
                if (s_odr_samples >= 32u) {
                    uint32_t avg_us = s_odr_accum_us / s_odr_samples;
                    if (avg_us > 0u) {
                        uint16_t rate_hz = (uint16_t)((1000000u + (avg_us / 2u)) / avg_us);
                        s_rate_hz = (int16_t)rate_hz;
                        IMU_Handler_SetSampleFreq((float)rate_hz);
#if LEGACY_IMU_UI_STREAM_ENABLE
                        ui_send_imu_odr(rate_hz);
                        if (!s_stream_odr_log_sent) {
                            char odr_msg[64];
                            s_stream_odr_log_sent = 1u;
                            snprintf(odr_msg, sizeof(odr_msg), "[IMU] Sensor ODR ~= %u Hz\r\n",
                                     (unsigned)rate_hz);
                            ui_send_log(ctx, odr_msg);
                        }
#endif
                    }
                    s_odr_accum_us = 0u;
                    s_odr_samples = 0u;
                }
            }
        }
    }
    if (delta_counter_out) *delta_counter_out = delta_counter;

    if (s_stream_diag_count < 8u) {
#if LEGACY_IMU_UI_STREAM_ENABLE
        char diag_msg[128];

        if (s_stream_diag_count == 0u) {
            snprintf(diag_msg, sizeof(diag_msg),
                     "[IMU] pkt0 sc=%u ts=%lu\r\n",
                     (unsigned)sample.sample_counter,
                     (unsigned long)timestamp_us);
        } else {
            uint8_t delta_counter = (uint8_t)(sample.sample_counter - s_prev_sample_counter);
            uint32_t delta_us = (uint32_t)(timestamp_us - s_prev_timestamp_us);
            snprintf(diag_msg, sizeof(diag_msg),
                     "[IMU] pkt%u sc=%u dsc=%u dts_us=%lu\r\n",
                     (unsigned)s_stream_diag_count,
                     (unsigned)sample.sample_counter,
                     (unsigned)delta_counter,
                     (unsigned long)delta_us);
        }
        ui_send_log(ctx, diag_msg);
#endif
        s_stream_diag_count++;
    }
    s_prev_sample_counter = sample.sample_counter;
    s_prev_timestamp_us = timestamp_us;

    {
        uint32_t now_ms = HAL_GetTick();
        uint32_t elapsed_ms = now_ms - s_stats_window_start_ms;
        if (elapsed_ms >= 1000u) {
            uint32_t edge_now = 0u;

            __disable_irq();
            edge_now = g_imu_drdy_edges_total;
            __enable_irq();

            s_stats_window_start_ms = now_ms;
            s_stats_edges_base = edge_now;
            s_stats_parse_ok = 0u;
            s_stats_ts_delta_us = 0u;
            s_stats_ts_delta_count = 0u;
        }
    }

    s_ui_throttle++;
    {
        const bool ui_due = (s_ui_throttle >= UI_ORIENT_DECIM);
        if (s_vsync_pending || ui_due) {
            update_shadow_roll_pitch(&sample.quaternion);
        }

        if (ui_due) {
            s_ui_throttle = 0u;
#if LEGACY_IMU_UI_STREAM_ENABLE
            ui_send_imu_orient((int16_t)s_roll_cd, (int16_t)s_pitch_cd);
#endif
        }
    }

    return 1;
}

void mcu_on_imu_drdy(McuContext *ctx)
{
    if (ctx->state != ST_OPERATION) return;

    /* The main loop intentionally drains IMU DRDY before the normal FSM step.
     * At high ODR that can delay periodic ICD work, so service CBIT here too.
     */
    icd_service_periodic(ctx);

    /* The IMU streams into a 40-entry FIFO in streaming mode. If we process only
     * one packet per DRDY event and fall behind, the queue can stay non-empty and
     * the delivered DRDY edge rate collapses well below the internal sample rate.
     * When sample_counter jumps by N, immediately drain up to N-1 more packets to
     * catch the host back up to the FIFO. */
    uint32_t processed = 0u;
    uint32_t burst_budget = 1u;

    while (processed < IMU_STREAM_BURST_MAX && burst_budget > 0u) {
        uint8_t delta_counter = 0u;
        int packet_status = 0;

        burst_budget--;
        packet_status = mcu_process_imu_stream_packet(ctx, &delta_counter);
        if (packet_status <= 0) break;

        processed++;

        if (delta_counter > 1u && processed < IMU_STREAM_BURST_MAX) {
            uint32_t backlog = (uint32_t)(delta_counter - 1u);
            uint32_t room = IMU_STREAM_BURST_MAX - processed;
            if (backlog > room) backlog = room;
            burst_budget += backlog;
        }
    }
}

void mcu_on_vsync(McuContext *ctx)
{
    if (ctx == NULL) return;
    if (ctx->state != ST_OPERATION) return;
    if (!cfg_mode_uses_vsync(s_cfg_store.mode)) return;
    s_vsync_pending = true;
}


static void fpga_service_orient(void)
{
    if (!s_vsync_pending) return;
    s_vsync_pending = false;

    const int16_t roll  = s_roll_cd;
    const int16_t pitch = s_pitch_cd;
    uint8_t payload[4];

    payload[0] = (uint8_t)(roll & 0xFF);
    payload[1] = (uint8_t)((roll >> 8) & 0xFF);
    payload[2] = (uint8_t)(pitch & 0xFF);
    payload[3] = (uint8_t)((pitch >> 8) & 0xFF);

    (void)s_rate_hz;
    s_orient_tx_seq++;

    /* VSYNC path is latency-sensitive: push only roll/pitch in one burst. */
    (void)fpga_write_regs(&g_fpga,
                          FPGA_REG_ORIENT_ROLL_CD8,
                          payload,
                          (uint16_t)sizeof(payload),
                          20);
}




// ===================== UI command handler =====================
// p/n are the *decoded* bytes from proto_uart (CRC OK, COBS OK).
// Format: [type][seq][lenL][lenH][...payload...]
void mcu_on_ui_payload(McuContext *ctx, const uint8_t *p, uint16_t n)
{
    if (n < 4) return;

    uint8_t  msg_type = p[0];
    uint8_t  seq      = p[1];
    uint16_t plen     = (uint16_t)p[2] | ((uint16_t)p[3] << 8);

    if ((uint16_t)(plen + 4u) != n) return;

    const uint8_t *d = &p[4];

    // Allow CMD_INIT_SYSTEM, CMD_GET_CONFIG, CMD_GET_VERSION, and CMD_PING from any state.
    // For any other command, reply BUSY until OPERATION.
    if (ctx->state != ST_OPERATION) {
        if (msg_type == MSG_CMD && plen >= 1) {
            uint8_t maybe_cmd = d[0];
            if (maybe_cmd != CMD_INIT_SYSTEM && maybe_cmd != CMD_GET_CONFIG &&
                maybe_cmd != CMD_GET_VERSION && maybe_cmd != CMD_PING) {
                ui_send_resp_simple(seq, ERR_BUSY);
                return;
            }
            // else: allow regardless of state
        } else if (msg_type == MSG_CMD) {
            ui_send_resp_simple(seq, ERR_BADARG);
            return;
        }
    }


    if (msg_type != MSG_CMD || plen < 1) {
        ui_send_resp_simple(seq, ERR_BADARG);
        return;
    }

    uint8_t cmd_id = d[0];

    switch (cmd_id) {

    case CMD_INIT_SYSTEM: {
        // Accept INIT when command byte exists; tolerate future payload extensions.
        if (plen < 1u) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }
        mcu_ui_log(ctx, "INIT cmd rx: seq=%u plen=%u\r\n", (unsigned)seq, (unsigned)plen);

        ctx->init_seq = seq;

        ui_send_init_ack(seq, NODE_SYSTEM, 0, 1, ERR_OK);
        ui_send_version(seq);
        ui_send_log(ctx, "INIT: bypassed - autonomous boot path is active\r\n");
        ui_send_log(ctx, mcu_get_reset_cause_str());   // surface why the MCU last reset
        Send_UART3_Debug("INIT: bypassed (state=%d)\r\n", (int)ctx->state);
        if (ctx->state == ST_WAIT_UI_INIT) {
            ctx->state = ST_BOOT;
        }

        break;
    }




    case CMD_GET_CONFIG: {
        ui_send_cfg_bank(CFG_BANK_EO);
        ui_send_cfg_bank(CFG_BANK_IR);
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_SET_DEBUG: {
        if (plen < 2) { ui_send_resp_simple(seq, ERR_BADARG); break; }
        ctx->debug_enabled = (d[1] != 0);
        g_debug_enabled = ctx->debug_enabled;
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_SET_MODE: {
      if (plen < 2) { ui_send_resp_simple(seq, ERR_BADARG); break; }
      uint8_t mode_id = d[1];

      if (!cfg_mode_valid(mode_id)) { ui_send_resp_simple(seq, ERR_BADARG); break; }

      uint8_t detail = fpga_push_mode(mode_id);
      if (detail != CFG_APPLY_OK) { ui_send_resp_simple(seq, ERR_FAIL); break; }

      ParamStoreBlob prev = s_cfg_store;
      s_cfg_store.mode = mode_id;
      s_cfg_store.write_count = prev.write_count + 1u;
      if (!cfg_save_to_flash()) {
          s_cfg_store = prev;
          (void)fpga_push_mode(prev.mode);
          ui_send_resp_simple(seq, ERR_FAIL);
          break;
      }

      ui_send_resp_simple(seq, ERR_OK);
      ui_send_cfg_bank(CFG_BANK_EO);
      ui_send_cfg_bank(CFG_BANK_IR);
      break;
    }

    case CMD_CAMERA_RESET: {
        mcu_ui_log(ctx, "Camera power cycle requested\r\n");
        if (Power_Cycle_All_Cameras() != 0) {
            mcu_ui_log(ctx, "Camera power cycle failed\r\n");
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }
        mcu_ui_log(ctx, "Camera power cycle complete\r\n");
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_PING: {
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_GET_VERSION: {
        ui_send_version(seq);
        break;
    }

    case CMD_SET_CAMERA_POWER: {
        if (plen < 4u) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        uint8_t node_type = d[1];
        uint8_t id = d[2];
        uint8_t on = (d[3] != 0u) ? 1u : 0u;

        if ((node_type != NODE_DAY && node_type != NODE_IR) || (id >= 6u && id != 7u)) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        uint8_t ok = 1u;
        if (id == 7u) {
            for (uint8_t i = 0u; i < 6u; ++i) {
                if (camera_power_set(node_type, i, on) != 0) {
                    ok = 0u;
                }
            }
        } else {
            ok = (camera_power_set(node_type, id, on) == 0) ? 1u : 0u;
        }
        uint8_t err = ok ? ERR_OK : ERR_FAIL;
        uint8_t resp[4] = { err, node_type, id, ok };
        ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
        break;
    }

    case CMD_APPLY_CONFIG: {
        if (plen < (uint16_t)(2u + CFG_BANK_PAYLOAD_SIZE)) {
            ui_send_cfg_apply_result(0xFFu, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        const uint8_t bank = d[1];
        if (bank != CFG_BANK_EO && bank != CFG_BANK_IR) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        ParamBankQ16 parsed;
        if (!cfg_bank_from_payload(&d[2], (uint16_t)(plen - 2u), &parsed)) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }
        if (!cfg_bank_values_valid(&parsed)) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        ParamStoreBlob prev = s_cfg_store;
        *cfg_bank_mut(bank) = parsed;

        uint8_t detail = fpga_push_bank(bank, cfg_bank_const(bank));
        if (detail != CFG_APPLY_OK) {
            s_cfg_store = prev;
            ui_send_cfg_apply_result(bank, 0u, detail);
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }

        s_cfg_store.write_count = prev.write_count + 1u;
        if (!cfg_save_to_flash()) {
            s_cfg_store = prev;
            (void)fpga_push_bank(bank, cfg_bank_const(bank));
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_FLASH);
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }

        ui_send_cfg_apply_result(bank, 1u, CFG_APPLY_OK);
        ui_send_cfg_bank(bank);
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_CAMERA_CONTROL: {
        if (plen < 4u) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        const uint8_t node_type = d[1];
        const uint8_t camera_sel = d[2];
        const uint8_t count = d[3];
        const uint16_t expected = (uint16_t)(4u + ((uint16_t)count * 4u));
        uint8_t overall_err = ERR_OK;

        if (count == 0u || plen != expected) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        if (node_type != NODE_DAY && node_type != NODE_IR) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        for (uint8_t i = 0u; i < count; ++i) {
            const uint16_t off = (uint16_t)(4u + ((uint16_t)i * 4u));
            const uint16_t param_id = (uint16_t)d[off] | ((uint16_t)d[off + 1u] << 8);
            const uint16_t value = (uint16_t)d[off + 2u] | ((uint16_t)d[off + 3u] << 8);
            int rc = -3;

            if (node_type == NODE_DAY) {
                rc = Apply_EO_Camera_Param(camera_sel, param_id, value);
            } else {
                rc = Apply_IR_Camera_Param(camera_sel, param_id, value);
            }

            {
                const uint8_t ok = (rc == 0) ? 1u : 0u;
                const uint8_t err = ok ? ERR_OK : ((rc == -2) ? ERR_BUSY : ERR_FAIL);
                const uint8_t resp_id = (camera_sel == 0u) ? 0xFFu : (uint8_t)(camera_sel - 1u);
                ui_send_camera_apply_result(seq, node_type, resp_id, param_id, value, ok, err);
                if (!ok) overall_err = err;
            }
        }

        ui_send_resp_simple(seq, overall_err);
        break;
    }

    case CMD_CHECK_NODE: {
        if (plen < 3) { ui_send_resp_simple(seq, ERR_BADARG); break; }

        uint8_t node_type = d[1];
        uint8_t id        = d[2];

        uint8_t err = ERR_OK;
        uint8_t ok  = 0;
        bool response_sent = false;

        switch (node_type) {
        case NODE_FPGA:
            // Safe: I2C address-ACK probe only (was: smoke-test write to mode reg).
            ok = fpga_status_ok() ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_DAY:
            if (id >= 6) { err = ERR_BADARG; break; }
            // Non-destructive VISCA version inquiry (no re-init).
            ok = (test_day_camera_controls(id) == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_IR:
            if (id >= 6) { err = ERR_BADARG; break; }
            // Safe: handshake ping only (was: full Initialize_IR_Camera re-init).
            ok = (ir_camera_handshake(id) == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_IMU:
            // Safe: read the data-freshness flag (was: IMU_Reset + blocking SPI).
            ok = imu_status_ok() ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_TEMP: {
            float t = 0.0f, h = 0.0f;
            Read_HDC3022_Sensor(&t, &h);

            // basic validity check (matches your error behavior)
            ok = (t > -900.0f && h > -900.0f) ? 1 : 0;

            // Return extended response for TEMP:
            // [ERR_OK][node_type][id][ok][t_centi_deg_L][t_centi_deg_H][h_centi_pct_L][h_centi_pct_H]
            int16_t t_cd = (int16_t)lroundf(t * 100.0f);
            uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);

            uint8_t resp[8];
            resp[0] = ERR_OK;
            resp[1] = node_type;
            resp[2] = id;
            resp[3] = ok;
            resp[4] = (uint8_t)(t_cd & 0xFF);
            resp[5] = (uint8_t)((t_cd >> 8) & 0xFF);
            resp[6] = (uint8_t)(h_cp & 0xFF);
            resp[7] = (uint8_t)((h_cp >> 8) & 0xFF);

            ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
            response_sent = true;
//            char msg[64];
//                    snprintf(msg, sizeof(msg),
//                             "Temperature %.2f C, Humidity %.2f %%\r\n",
//                             t_cd / 100.0f,
//                             h_cp / 100.0f);
//
//                    Send_UART3_Debug(msg);

            break;
        }

        default:
        err = ERR_BADARG;
        break;
       }

        if (!response_sent) {
            uint8_t resp[4] = { err, node_type, id, ok };
            ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
        }

        break;
    }

    default:
        ui_send_resp_simple(seq, ERR_BADARG);
        break;
    }
}

static void icd_write_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void icd_write_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t icd_crc16_ccitt_payload(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t *icd_sequence_slot(uint16_t msg_id)
{
    switch (msg_id) {
    case ICD_PIPA_RITA_STATUS: return &s_icd_seq_status;
    case ICD_PIPA_RITA_MAINTENANCE_STATUS: return &s_icd_seq_maintenance_status;
    case ICD_PIPA_RITA_PBIT: return &s_icd_seq_pbit;
    case ICD_PIPA_RITA_CBIT: return &s_icd_seq_cbit;
    case ICD_PIPA_RITA_IBIT: return &s_icd_seq_ibit;
    case ICD_PIPA_RITA_MODE_CONTROL_ACK: return &s_icd_seq_mode_ack;
    case ICD_PIPA_RITA_CAMERA_CONTROL_ACK: return &s_icd_seq_camera_ack;
    case ICD_PIPA_RITA_PANORAMA_ACK: return &s_icd_seq_panorama_ack;
    case ICD_PIPA_RITA_MAINTENANCE_ACK: return &s_icd_seq_maintenance_ack;
    default: return &s_icd_seq_default;
    }
}

static uint32_t icd_next_sequence(uint16_t msg_id)
{
    uint32_t *slot = icd_sequence_slot(msg_id);
    uint32_t seq = *slot;
    *slot = seq + 1u;
    if (*slot == 0u) {
        *slot = 1u;
    }
    return seq;
}

static uint64_t icd_read_u64(const uint8_t *p)
{
    uint64_t v = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        v |= ((uint64_t)p[i]) << (8u * i);
    }
    return v;
}

static void icd_write_u64(uint8_t *p, uint64_t value)
{
    for (uint8_t i = 0u; i < 8u; ++i) {
        p[i] = (uint8_t)((value >> (8u * i)) & 0xFFu);
    }
}

static int64_t icd_days_from_civil(int32_t y, uint32_t m, uint32_t d)
{
    y -= (m <= 2u) ? 1 : 0;
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t mp = m + ((m > 2u) ? (uint32_t)-3 : 9u);
    const uint32_t doy = (153u * mp + 2u) / 5u + d - 1u;
    const uint32_t doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static void icd_set_time_from_epoch_ms(uint64_t epoch_ms)
{
    s_icd_time_epoch_ms = epoch_ms;
    s_icd_time_base_ms = HAL_GetTick();
    s_icd_time_valid = true;
}

static void icd_set_time_from_tag(const uint8_t tag[8])
{
    if (tag == NULL) {
        return;
    }

    uint64_t epoch_ms = icd_read_u64(tag);
    if (epoch_ms == 0u) {
        return;
    }
    icd_set_time_from_epoch_ms(epoch_ms);
}

// Current PIPA time estimate: last-synced epoch plus elapsed ticks since that
// sync. Returns 0 if never synced - callers that care must check
// s_icd_time_valid themselves (icd_write_time_tag already does).
static uint64_t icd_current_time_ms(void)
{
    if (!s_icd_time_valid) {
        return 0u;
    }
    return s_icd_time_epoch_ms + (uint64_t)(HAL_GetTick() - s_icd_time_base_ms);
}

static void icd_write_time_tag(uint8_t *p)
{
    if (!s_icd_time_valid) {
        memset(p, 0, 8u);
        return;
    }

    icd_write_u64(p, icd_current_time_ms());
}

static uint32_t icd_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static void icd_set_time_from_utc_payload(const uint8_t *payload, uint32_t payload_len)
{
    if (payload == NULL || payload_len < 24u) {
        return;
    }

    uint32_t year = icd_read_u32(&payload[0]);
    uint32_t month = icd_read_u32(&payload[4]);
    uint32_t day = icd_read_u32(&payload[8]);
    uint32_t hour = icd_read_u32(&payload[12]);
    uint32_t min = icd_read_u32(&payload[16]);
    uint32_t sec = icd_read_u32(&payload[20]);
    if (year <= 9999u && month >= 1u && month <= 12u && day >= 1u && day <= 31u &&
        hour <= 23u && min <= 59u && sec <= 59u) {
        int64_t days = icd_days_from_civil((int32_t)year, month, day);
        if (days >= 0) {
            uint64_t seconds = ((uint64_t)days * 86400u) +
                               ((uint64_t)hour * 3600u) +
                               ((uint64_t)min * 60u) +
                               (uint64_t)sec;
            uint64_t new_ms = seconds * 1000u;

            // UI sends this at 1Hz. Only actually re-sync (jump the clock)
            // when PIPA's own free-running estimate has drifted from it by
            // more than 1000ms - the UTC_INFO payload only carries
            // whole-second resolution, so anything within a second is just
            // that quantization, not real drift. Always sync on the very
            // first message (nothing to compare against yet).
            if (!s_icd_time_valid) {
                icd_set_time_from_epoch_ms(new_ms);
            } else {
                uint64_t cur_ms = icd_current_time_ms();
                uint64_t diff_ms = (new_ms > cur_ms) ? (new_ms - cur_ms) : (cur_ms - new_ms);
                if (diff_ms > 1000u) {
                    icd_set_time_from_epoch_ms(new_ms);
                }
            }
        }
    }
}

static void icd_send_frame(uint16_t msg_id, uint32_t seq, const uint8_t *payload, uint32_t payload_len)
{
    (void)seq;
    uint8_t frame[UI_TX_MAX];
    const uint32_t total_len = 22u + payload_len + 6u;
    if (payload_len > 400u || total_len > sizeof(frame)) {
        return;
    }

    icd_write_le32(&frame[0], 0x70717883u);
    icd_write_le16(&frame[4], msg_id);
    icd_write_le32(&frame[6], icd_next_sequence(msg_id));
    icd_write_le32(&frame[10], payload_len);
    icd_write_time_tag(&frame[14]);
    if (payload_len && payload) {
        memcpy(&frame[22], payload, payload_len);
    }

    const uint16_t crc = icd_crc16_ccitt_payload(payload, payload_len);
    const uint32_t crc_off = 22u + payload_len;
    icd_write_le16(&frame[crc_off], crc);
    icd_write_le32(&frame[crc_off + 2u], 0x83787170u);
    (void)HAL_UART_Transmit(&huart3, frame, (uint16_t)total_len, 50);
}

static uint8_t icd_payload_all_zero(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL) return 1u;
    for (uint16_t i = 0u; i < payload_len; ++i) {
        if (payload[i] != 0u) return 0u;
    }
    return 1u;
}

static uint8_t icd_payload_changed(const uint8_t *a, const uint8_t *b, uint8_t off, uint8_t len)
{
    for (uint8_t i = 0u; i < len; ++i) {
        if (a[off + i] != b[off + i]) return 1u;
    }
    return 0u;
}

static uint16_t icd_read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void icd_write_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void icd_maintenance_init_defaults(void)
{
    memset(s_icd_maintenance_status, 0, sizeof(s_icd_maintenance_status));
    memset(s_icd_ir_cam_maint, 0, sizeof(s_icd_ir_cam_maint));
    memset(s_icd_eo_cam_maint, 0, sizeof(s_icd_eo_cam_maint));

    // Values here mirror the ICD defaults or harmless camera defaults. Real
    // register readback requires camera-driver query APIs, which do not exist
    // in the current firmware; until then 0x2211 reports the last accepted ICD
    // maintenance values plus live board-owned state.
    s_icd_maintenance_status[ICD_MAINT_IR_FRAME_RATE] = 0u;      // 60 Hz
    s_icd_maintenance_status[ICD_MAINT_IR_POLARITY] = 0u;
    // Golden frame default (3700K Manual). ICD White Balance is 0=Off,
    // 1=Auto, 2=One Push AWB, 3=3700K Manual, 4=5100K Manual, 5=8000K
    // Manual, 6=One Push AWB Trigger - value 3, not 2.
    s_icd_maintenance_status[ICD_MAINT_EO_WHITE_BALANCE] = 3u;
    s_icd_maintenance_status[ICD_MAINT_EO_GAMMA] = 0u;

    // Seed every camera's own remembered snapshot with the same baseline
    // defaults (the memset above already zeroed everything else to 0, which
    // matches the rest of the defaults above).
    for (uint8_t i = 0u; i < 6u; ++i) {
        s_icd_eo_cam_maint[i][ICD_MAINT_EO_WHITE_BALANCE] = 3u;
    }
}

static uint8_t icd_apply_ir_maint_u8(uint8_t cam_sel, uint8_t offset, uint16_t param_id)
{
    uint8_t value = s_icd_maintenance_status[offset];
    int rc = Apply_IR_Camera_Param(cam_sel, param_id, value);
    return (rc == 0) ? ERR_OK : ((rc == -2) ? ERR_BUSY : ERR_FAIL);
}

static uint8_t icd_apply_ir_maint_u16(uint8_t cam_sel, uint8_t offset_l, uint16_t param_id)
{
    uint16_t value = icd_read_u16(&s_icd_maintenance_status[offset_l]);
    int rc = Apply_IR_Camera_Param(cam_sel, param_id, value);
    return (rc == 0) ? ERR_OK : ((rc == -2) ? ERR_BUSY : ERR_FAIL);
}

static uint16_t icd_tnr_icd_value_to_camera_step(uint16_t value)
{
    if (value >= ICD_IR_TNR_MAX_VALUE) {
        return (uint16_t)(ICD_IR_TNR_STEPS - 1u);
    }

    return (uint16_t)((((uint32_t)value * (ICD_IR_TNR_STEPS - 1u)) +
                       (ICD_IR_TNR_MAX_VALUE / 2u)) / ICD_IR_TNR_MAX_VALUE);
}

static uint8_t icd_apply_eo_maint_u8(uint8_t cam_sel, uint8_t offset, uint16_t param_id)
{
    uint8_t value = s_icd_maintenance_status[offset];
    int rc = Apply_EO_Camera_Param(cam_sel, param_id, value);
    return (rc == 0) ? ERR_OK : ((rc == -2) ? ERR_BUSY : ERR_FAIL);
}

static uint8_t icd_apply_maintenance_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len < ICD_MAINT_LEN) {
        return ERR_BADARG;
    }

    if (icd_payload_all_zero(payload, ICD_MAINT_LEN)) {
        return ERR_OK; // All-zero frame is treated as a status request.
    }

    uint8_t prev[ICD_MAINT_LEN];
    memcpy(prev, s_icd_maintenance_status, ICD_MAINT_LEN);
    memcpy(s_icd_maintenance_status, payload, ICD_MAINT_LEN);

    uint8_t overall = ERR_OK;
    uint8_t ir_cam = s_icd_maintenance_status[ICD_MAINT_SELECTED_IR_CAM];
    uint8_t eo_cam = s_icd_maintenance_status[ICD_MAINT_SELECTED_EO_CAM];

    if (ir_cam > 6u || eo_cam > 6u) {
        return ERR_BADARG;
    }

    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_NUC, 1u) &&
        s_icd_maintenance_status[ICD_MAINT_IR_NUC] == 1u) {
        int rc = Apply_IR_Camera_Param(ir_cam, PARAM_ID_IR_NUC_TRIGGER, 3u);
        if (rc != 0) overall = ERR_FAIL;
        // NUC is a momentary trigger, not a latched state. Reset immediately so
        // the next request - even with the same mode byte - is seen as a fresh
        // 0->1/2 edge instead of being silently swallowed as "unchanged".
        s_icd_maintenance_status[ICD_MAINT_IR_NUC] = 0u;
    } else if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_NUC, 1u) &&
               s_icd_maintenance_status[ICD_MAINT_IR_NUC] == 2u) {
        int rc = Apply_IR_Camera_Param(ir_cam, PARAM_ID_IR_NUC_TRIGGER, 4u);
        if (rc != 0) overall = ERR_FAIL;
        s_icd_maintenance_status[ICD_MAINT_IR_NUC] = 0u;
    }

    uint8_t err;
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_SCREEN_INV, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_SCREEN_INV, PARAM_ID_IR_IMAGE_FLIP);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_FRAME_RATE, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_FRAME_RATE, PARAM_ID_IR_FRAME_RATE);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_ICE_GAIN_L, 2u)) {
        err = icd_apply_ir_maint_u16(ir_cam, ICD_MAINT_IR_ICE_GAIN_L, PARAM_ID_IR_ICE_GAIN_BIAS);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_ICE_LEVEL_L, 2u)) {
        err = icd_apply_ir_maint_u16(ir_cam, ICD_MAINT_IR_ICE_LEVEL_L, PARAM_ID_IR_ICE_LEVEL_BIAS);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_ICE_STRENGTH, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_ICE_STRENGTH, PARAM_ID_IR_ICE_STRENGTH);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_COLOR_MODE, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_COLOR_MODE, PARAM_ID_IR_PALETTE);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_POLARITY, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_POLARITY, PARAM_ID_IR_POLARITY);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_DZOOM, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_DZOOM, PARAM_ID_IR_D_ZOOM);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_TNR_ENABLE, 1u)) {
        err = icd_apply_ir_maint_u8(ir_cam, ICD_MAINT_IR_TNR_ENABLE, PARAM_ID_IR_TNR_ENABLE);
        if (err != ERR_OK) overall = err;
    }

    uint16_t tnr = icd_read_u16(&s_icd_maintenance_status[ICD_MAINT_IR_TNR_L]);
    if (tnr > ICD_IR_TNR_MAX_VALUE) {
        tnr = ICD_IR_TNR_MAX_VALUE;
        icd_write_u16(&s_icd_maintenance_status[ICD_MAINT_IR_TNR_L], tnr);
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_IR_TNR_L, 2u)) {
        uint16_t camera_tnr_step = icd_tnr_icd_value_to_camera_step(tnr);
        int rc = Apply_IR_Camera_Param(ir_cam, PARAM_ID_IR_TNR_THRESHOLD, camera_tnr_step);
        if (rc != 0) overall = (rc == -2) ? ERR_BUSY : ERR_FAIL;
    }

    if (s_icd_maintenance_status[ICD_MAINT_EO_POWER_RESET] != 0u &&
        icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_POWER_RESET, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_POWER_RESET, PARAM_ID_EO_POWER_RESET);
        if (err != ERR_OK) overall = err;
    }

    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_DZOOM, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_DZOOM, PARAM_ID_EO_D_ZOOM);
        if (err != ERR_OK) overall = err;
        // Values 6-11 are momentary digital-zoom motion commands (On/Off/
        // Stop/Tele/Wide/x1-Max toggle), not a persisted zoom-level state like
        // 0-5. Reset immediately after firing so the next request - even with
        // the same command byte - is seen as a fresh edge instead of being
        // silently swallowed as "unchanged" (same fix already applied to NUC).
        if (s_icd_maintenance_status[ICD_MAINT_EO_DZOOM] >= 6u) {
            s_icd_maintenance_status[ICD_MAINT_EO_DZOOM] = 0u;
        }
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_WHITE_BALANCE, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_WHITE_BALANCE, PARAM_ID_EO_WHITE_BALANCE);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_RED_GAIN, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_RED_GAIN, PARAM_ID_EO_RED_GAIN);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_BLUE_GAIN, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_BLUE_GAIN, PARAM_ID_EO_BLUE_GAIN);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_CHROMA_GAIN, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_CHROMA_GAIN, PARAM_ID_EO_CHROMA_GAIN);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_AGC, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_AGC, PARAM_ID_EO_AGC);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_AE_SHUTTER, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_AE_SHUTTER, PARAM_ID_EO_AE_SHUTTER);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_BRIGHTNESS, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_BRIGHTNESS, PARAM_ID_EO_BRIGHTNESS);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_BLC, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_BLC, PARAM_ID_EO_BLC);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_HLC_ONOFF, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_HLC_ONOFF, PARAM_ID_EO_HLC);
        if (err != ERR_OK) overall = err;
    }
    if (s_icd_maintenance_status[ICD_MAINT_EO_HLC_LEVEL] != 0u &&
        icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_HLC_LEVEL, 1u)) {
        int rc = Apply_EO_Camera_Param(eo_cam, PARAM_ID_EO_HLC,
                                       (uint16_t)(100u + s_icd_maintenance_status[ICD_MAINT_EO_HLC_LEVEL]));
        if (rc != 0) overall = ERR_FAIL;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_ACE_ONOFF, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_ACE_ONOFF, PARAM_ID_EO_ACE);
        if (err != ERR_OK) overall = err;
    }
    if (s_icd_maintenance_status[ICD_MAINT_EO_ACE_LEVEL] != 0u &&
        icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_ACE_LEVEL, 1u)) {
        int rc = Apply_EO_Camera_Param(eo_cam, PARAM_ID_EO_ACE,
                                       (uint16_t)(100u + s_icd_maintenance_status[ICD_MAINT_EO_ACE_LEVEL]));
        if (rc != 0) overall = ERR_FAIL;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_DEFOG_ONOFF, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_DEFOG_ONOFF, PARAM_ID_EO_DEFOG);
        if (err != ERR_OK) overall = err;
    }
    if (s_icd_maintenance_status[ICD_MAINT_EO_DEFOG_LEVEL] != 0u &&
        icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_DEFOG_LEVEL, 1u)) {
        int rc = Apply_EO_Camera_Param(eo_cam, PARAM_ID_EO_DEFOG,
                                       (uint16_t)(100u + s_icd_maintenance_status[ICD_MAINT_EO_DEFOG_LEVEL]));
        if (rc != 0) overall = ERR_FAIL;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_DNR, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_DNR, PARAM_ID_EO_DNR);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_GAMMA, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_GAMMA, PARAM_ID_EO_GAMMA);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_SHARPNESS, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_SHARPNESS, PARAM_ID_EO_SHARPNESS);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_MIRROR, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_MIRROR, PARAM_ID_EO_MIRROR);
        if (err != ERR_OK) overall = err;
    }
    if (icd_payload_changed(prev, s_icd_maintenance_status, ICD_MAINT_EO_FLIP, 1u)) {
        err = icd_apply_eo_maint_u8(eo_cam, ICD_MAINT_EO_FLIP, PARAM_ID_EO_IMAGE_FLIP);
        if (err != ERR_OK) overall = err;
    }

    // Remember this camera's (or, for a broadcast/"All Cameras" command,
    // every camera's) resulting field values, so a later Maintenance-mode
    // entry can report each camera's own last-applied values instead of one
    // shared snapshot. Recorded unconditionally, same as s_icd_maintenance_status
    // itself above: this ICD's ACK/status reflects what was commanded, not a
    // per-field verified camera-health verdict.
    if (ir_cam == 0u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            memcpy(&s_icd_ir_cam_maint[i][ICD_MAINT_IR_FIELD_OFF],
                   &s_icd_maintenance_status[ICD_MAINT_IR_FIELD_OFF], ICD_MAINT_IR_FIELD_LEN);
        }
    } else {
        memcpy(&s_icd_ir_cam_maint[ir_cam - 1u][ICD_MAINT_IR_FIELD_OFF],
               &s_icd_maintenance_status[ICD_MAINT_IR_FIELD_OFF], ICD_MAINT_IR_FIELD_LEN);
    }

    if (eo_cam == 0u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            memcpy(&s_icd_eo_cam_maint[i][ICD_MAINT_EO_FIELD_OFF],
                   &s_icd_maintenance_status[ICD_MAINT_EO_FIELD_OFF], ICD_MAINT_EO_FIELD_LEN);
        }
    } else {
        memcpy(&s_icd_eo_cam_maint[eo_cam - 1u][ICD_MAINT_EO_FIELD_OFF],
               &s_icd_maintenance_status[ICD_MAINT_EO_FIELD_OFF], ICD_MAINT_EO_FIELD_LEN);
    }

    return overall;
}

static void icd_fill_maintenance_ack(uint8_t p[ICD_MAINT_ACK_LEN])
{
    memset(p, 0, ICD_MAINT_ACK_LEN);
    memcpy(p, s_icd_maintenance_status, ICD_MAINT_LEN);

    // Report selected camera IDs from the last accepted maintenance command.
    if (p[ICD_MAINT_SELECTED_IR_CAM] > 6u) p[ICD_MAINT_SELECTED_IR_CAM] = 0u;
    if (p[ICD_MAINT_SELECTED_EO_CAM] > 6u) p[ICD_MAINT_SELECTED_EO_CAM] = 0u;

    // Power reset is a momentary command, not a latched state.
    p[ICD_MAINT_EO_POWER_RESET] = 0u;

    // Keep ICD TNR packed little-endian. The wire value is 0..16383 and is
    // mapped to the camera's 15 available hardware steps only when applying.
    uint16_t tnr = icd_read_u16(&p[ICD_MAINT_IR_TNR_L]);
    if (tnr > ICD_IR_TNR_MAX_VALUE) {
        icd_write_u16(&p[ICD_MAINT_IR_TNR_L], ICD_IR_TNR_MAX_VALUE);
    }

    // Customer maintenance wire frame appends Roll/Pitch/Heading as uint16.
    // 0xFFFF means unavailable/invalid in the golden frames.
    icd_write_u16(&p[37], 0xFFFFu);
    icd_write_u16(&p[39], 0xFFFFu);
    icd_write_u16(&p[41], 0xFFFFu);
}

// ICD v0.92 0x2211 payload = 37 maintenance bytes followed by per-camera
// Roll, Pitch, Yaw uint16 triples for IR cam1..6 then EO cam1..6. Shared by
// every 0x2211 builder below - camera pose is independent of which camera is
// "selected" in the leading 37 bytes.
static void icd_append_maintenance_pose(uint8_t p[ICD_MAINT_STATUS_LEN])
{
    uint8_t off = ICD_MAINT_LEN;
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        icd_write_u16(&p[off + 0u], (uint16_t)s_cfg_store.ir.roll_cd[i]);
        icd_write_u16(&p[off + 2u], (uint16_t)s_cfg_store.ir.pitch_cd[i]);
        icd_write_u16(&p[off + 4u], s_cfg_store.ir.yaw_cd[i]);
        off = (uint8_t)(off + 6u);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        icd_write_u16(&p[off + 0u], (uint16_t)s_cfg_store.eo.roll_cd[i]);
        icd_write_u16(&p[off + 2u], (uint16_t)s_cfg_store.eo.pitch_cd[i]);
        icd_write_u16(&p[off + 4u], s_cfg_store.eo.yaw_cd[i]);
        off = (uint8_t)(off + 6u);
    }
}

static void icd_fill_maintenance_status(uint8_t p[ICD_MAINT_STATUS_LEN])
{
    memset(p, 0, ICD_MAINT_STATUS_LEN);
    memcpy(p, s_icd_maintenance_status, ICD_MAINT_LEN);

    if (p[ICD_MAINT_SELECTED_IR_CAM] > 6u) p[ICD_MAINT_SELECTED_IR_CAM] = 0u;
    if (p[ICD_MAINT_SELECTED_EO_CAM] > 6u) p[ICD_MAINT_SELECTED_EO_CAM] = 0u;
    p[ICD_MAINT_EO_POWER_RESET] = 0u;

    uint16_t tnr = icd_read_u16(&p[ICD_MAINT_IR_TNR_L]);
    if (tnr > ICD_IR_TNR_MAX_VALUE) {
        icd_write_u16(&p[ICD_MAINT_IR_TNR_L], ICD_IR_TNR_MAX_VALUE);
    }

    icd_append_maintenance_pose(p);
}

// Builds a 0x2211 maintenance-status frame for ONE physical camera (0-5),
// reporting that camera's own remembered last-applied IR/EO values rather
// than the single shared "most recently commanded" snapshot.
static void icd_fill_maintenance_status_for_camera(uint8_t p[ICD_MAINT_STATUS_LEN], uint8_t cam_idx)
{
    memset(p, 0, ICD_MAINT_STATUS_LEN);

    p[ICD_MAINT_SELECTED_IR_CAM] = (uint8_t)(cam_idx + 1u);
    memcpy(&p[ICD_MAINT_IR_FIELD_OFF], &s_icd_ir_cam_maint[cam_idx][ICD_MAINT_IR_FIELD_OFF], ICD_MAINT_IR_FIELD_LEN);

    p[ICD_MAINT_SELECTED_EO_CAM] = (uint8_t)(cam_idx + 1u);
    memcpy(&p[ICD_MAINT_EO_FIELD_OFF], &s_icd_eo_cam_maint[cam_idx][ICD_MAINT_EO_FIELD_OFF], ICD_MAINT_EO_FIELD_LEN);

    uint16_t tnr = icd_read_u16(&p[ICD_MAINT_IR_TNR_L]);
    if (tnr > ICD_IR_TNR_MAX_VALUE) {
        icd_write_u16(&p[ICD_MAINT_IR_TNR_L], ICD_IR_TNR_MAX_VALUE);
    }

    icd_append_maintenance_pose(p);
}

// Sent on every MODE_CONTROL request into Maintenance mode: one 0x2211 frame
// per physical camera (0-5), each selecting that camera and reporting its own
// last-applied/default IR+EO maintenance values, instead of a single shared
// snapshot for whichever camera was last touched.
static void icd_send_maintenance_camera_burst(uint32_t seq)
{
    for (uint8_t cam_idx = 0u; cam_idx < 6u; ++cam_idx) {
        uint8_t status[ICD_MAINT_STATUS_LEN];
        icd_fill_maintenance_status_for_camera(status, cam_idx);
        icd_send_frame(ICD_PIPA_RITA_MAINTENANCE_STATUS, seq, status, sizeof(status));
    }
}

static uint8_t icd_status_from_power(uint8_t on)
{
    return on ? 1u : 2u;
}

static uint8_t icd_refresh_ir_presence(uint8_t cam_idx)
{
    if (cam_idx >= 6u) {
        return 0u;
    }

    if (!s_ir_power_on[cam_idx]) {
        s_ir_bit_ok[cam_idx] = 0u;
        return 0u;
    }

    s_ir_bit_ok[cam_idx] = (ir_camera_handshake(cam_idx) == 0) ? 1u : 0u;
    return s_ir_bit_ok[cam_idx];
}

static void icd_refresh_all_ir_presence(void)
{
    for (uint8_t i = 0u; i < 6u; ++i) {
        (void)icd_refresh_ir_presence(i);
    }
}

// Advance any in-flight background IR presence probe and apply its result to
// s_ir_bit_ok. Non-blocking; called frequently (every icd_service_periodic).
// This replaces the per-second BLOCKING ir_camera_handshake() that used to run
// on this path and freeze the whole main loop (and thus UI command handling)
// for up to ~300 ms once per second.
static void icd_service_ir_presence_probe(void)
{
    if (s_ir_probe_inflight_cam == 0xFFu) {
        return;
    }
    int r = IR_PresenceProbe_Poll();
    switch (r) {
        case 1: // completed - present
            s_ir_bit_ok[s_ir_probe_inflight_cam] = 1u;
            s_ir_probe_inflight_cam = 0xFFu;
            break;
        case 2: // completed - absent
            s_ir_bit_ok[s_ir_probe_inflight_cam] = 0u;
            s_ir_probe_inflight_cam = 0xFFu;
            break;
        case 3:  // aborted (mux disturbed by a command): leave bit_ok, retry later
        case -1: // probe unexpectedly idle: stop tracking
            s_ir_probe_inflight_cam = 0xFFu;
            break;
        default: // 0: still in progress
            break;
    }
}

// Non-blocking replacement for the old blocking per-CBIT presence refresh on
// the 1 Hz path: advance the round-robin by one and, if that camera is powered,
// kick a background probe whose result lands later via
// icd_service_ir_presence_probe(). Unpowered cameras are marked not-present
// immediately, exactly as before.
static void icd_start_next_ir_presence(void)
{
    if (s_ir_probe_inflight_cam != 0xFFu) {
        return;  // previous probe still running (< ~300 ms << 1 s CBIT); skip
    }

    uint8_t cam = s_ir_cbit_next_idx;
    s_ir_cbit_next_idx = (uint8_t)((s_ir_cbit_next_idx + 1u) % 6u);

    if (!s_ir_power_on[cam]) {
        s_ir_bit_ok[cam] = 0u;
        return;
    }
    if (IR_PresenceProbe_Start(cam) == 0) {
        s_ir_probe_inflight_cam = cam;
    }
}

static uint8_t icd_ir_status_from_power_or_nuc(uint8_t cam_idx, uint32_t now)
{
    if (cam_idx < 6u && s_ir_nuc_status_until_ms[cam_idx] != 0u &&
        (int32_t)(now - s_ir_nuc_status_until_ms[cam_idx]) < 0) {
        return 3u;
    }
    return (s_ir_power_on[cam_idx] && s_ir_bit_ok[cam_idx]) ? 1u : 2u;
}

static uint8_t icd_scale_u8_to_range(uint8_t value, uint8_t in_min, uint8_t in_max, uint8_t out_max)
{
    if (value <= in_min) return 0u;
    if (value >= in_max) return out_max;
    return (uint8_t)((((uint32_t)(value - in_min) * out_max) + ((in_max - in_min) / 2u)) / (in_max - in_min));
}

static uint8_t icd_apply_panorama_camera_params(const uint8_t *payload)
{
    uint8_t overall = ERR_OK;

    if (payload[1] <= 99u) {
        uint8_t eo_brightness = payload[1] == 0u ? 0u : icd_scale_u8_to_range(payload[1], 1u, 99u, 20u);
        int rc = Apply_EO_Camera_Param(0u, PARAM_ID_EO_BRIGHTNESS, eo_brightness);
        if (rc == 0) {
            s_icd_video_brightness = payload[1];
        } else {
            overall = (rc == -2) ? ERR_BUSY : ERR_FAIL;
        }
    } else {
        overall = ERR_BADARG;
    }

    if (payload[2] <= 99u) {
        uint16_t eo_contrast = payload[2] == 0u
            ? 0u
            : (uint16_t)(100u + icd_scale_u8_to_range(payload[2], 1u, 99u, 11u));
        int rc = Apply_EO_Camera_Param(0u, PARAM_ID_EO_ACE, eo_contrast);
        if (rc == 0) {
            s_icd_video_contrast = payload[2];
        } else if (overall == ERR_OK) {
            overall = (rc == -2) ? ERR_BUSY : ERR_FAIL;
        }
    } else if (overall == ERR_OK) {
        overall = ERR_BADARG;
    }

    if (payload[3] <= 2u) {
        if (payload[3] == 0u) {
            s_icd_ir_image_polarity = 0u;
        } else {
            uint16_t ir_polarity = (payload[3] == 1u) ? 0u : 1u;
            int rc = Apply_IR_Camera_Param(0u, PARAM_ID_IR_POLARITY, ir_polarity);
            if (rc == 0) {
                s_icd_ir_image_polarity = payload[3];
                s_icd_maintenance_status[ICD_MAINT_IR_POLARITY] = (uint8_t)ir_polarity;
            } else if (overall == ERR_OK) {
                overall = (rc == -2) ? ERR_BUSY : ERR_FAIL;
            }
        }
    } else if (overall == ERR_OK) {
        overall = ERR_BADARG;
    }

    if (payload[4] <= 2u) {
        s_icd_video_stabilization = payload[4];
    } else if (overall == ERR_OK) {
        overall = ERR_BADARG;
    }

    if (payload[5] <= 2u) {
        s_icd_video_stabilization_method = payload[5];
    } else if (overall == ERR_OK) {
        overall = ERR_BADARG;
    }

    uint16_t blend = icd_read_u16(&payload[6]);
    if (blend <= 999u) {
        s_icd_blending_area_px = blend;
    } else if (overall == ERR_OK) {
        overall = ERR_BADARG;
    }

    return overall;
}

static uint8_t icd_video_select_from_mode(uint8_t mode)
{
    if (mode >= 0x07u && mode <= 0x0Cu) return (uint8_t)(1u + (mode - 0x07u));
    if (mode >= 0x0Du && mode <= 0x12u) return (uint8_t)(7u + (mode - 0x0Du));
    if (mode == 0x14u) return 13u;
    if (mode == 0x15u) return 14u;
    return 0u;
}

static uint8_t icd_mode_from_video_select(uint8_t select)
{
    if (select >= 1u && select <= 6u) return (uint8_t)(0x07u + (select - 1u));
    if (select >= 7u && select <= 12u) return (uint8_t)(0x0Du + (select - 7u));
    if (select == 13u) return 0x14u;
    if (select == 14u) return 0x15u;
    return 0xFFu;
}

static uint8_t icd_apply_mode(uint8_t mode_id)
{
    if (!cfg_mode_valid(mode_id)) {
        return ERR_BADARG;
    }

    uint8_t detail = fpga_push_mode(mode_id);
    if (detail != CFG_APPLY_OK) {
        return ERR_FAIL;
    }

    ParamStoreBlob prev = s_cfg_store;
    s_cfg_store.mode = mode_id;
    s_cfg_store.write_count = prev.write_count + 1u;
    if (!cfg_save_to_flash()) {
        s_cfg_store = prev;
        (void)fpga_push_mode(prev.mode);
        return ERR_FAIL;
    }
    return ERR_OK;
}

static void icd_fill_status(uint8_t p[ICD_STATUS_LEN])
{
    memset(p, 0, ICD_STATUS_LEN);
    p[0] = s_icd_operation_mode;
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0u; i < 6u; ++i) {
        p[1u + i] = icd_status_from_power(s_day_power_on[i]);
        p[7u + i] = icd_ir_status_from_power_or_nuc(i, now);
    }
    p[13] = icd_video_select_from_mode(s_cfg_store.mode);
    p[14] = s_icd_video_brightness;
    p[15] = s_icd_video_contrast;
    p[16] = s_icd_ir_image_polarity;
    p[17] = s_icd_video_stabilization;
    p[18] = s_icd_video_stabilization_method;
    icd_write_u16(&p[19], s_icd_blending_area_px);
}

static void icd_fill_panorama_ack(uint8_t *p, uint8_t len, uint8_t fpga_reset_ack)
{
    memset(p, 0, len);
    p[0] = icd_video_select_from_mode(s_cfg_store.mode);
    p[1] = s_icd_video_brightness;
    p[2] = s_icd_video_contrast;
    p[3] = s_icd_ir_image_polarity;
    p[4] = s_icd_video_stabilization;
    p[5] = s_icd_video_stabilization_method;
    icd_write_u16(&p[6], s_icd_blending_area_px);
    if (len > ICD_PANORAMA_FPGA_RESET_OFF) {
        p[ICD_PANORAMA_FPGA_RESET_OFF] = fpga_reset_ack;
    }
}

static void icd_send_status(uint32_t seq)
{
    uint8_t p[ICD_STATUS_LEN];
    icd_fill_status(p);
    icd_send_frame(ICD_PIPA_RITA_STATUS, seq, p, sizeof(p));
}

static uint8_t icd_ok_code(uint8_t ok)
{
    return ok ? 0u : 1u;
}

static uint8_t icd_clamp_u8_from_float(float value, float min_value, float max_value)
{
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;
    int32_t rounded = (int32_t)lroundf(value);
    if (rounded < 0) return 0u;
    if (rounded > 255) return 255u;
    return (uint8_t)rounded;
}

static void icd_update_env_sample(uint8_t force)
{
    const uint32_t now = HAL_GetTick();
    if (!force && s_icd_env_valid && (uint32_t)(now - s_icd_env_last_ms) < 900u) {
        return;
    }

    float t = -999.0f;
    float h = -999.0f;
    Read_HDC3022_Sensor(&t, &h);

    s_icd_env_ok = (t >= -40.0f && t <= 85.0f && h >= 0.0f && h <= 100.0f) ? 1u : 0u;
    if (s_icd_env_ok) {
        /* ICD declares these fields as UINT8. Current UI/customer use positive
         * board temperatures, so clamp the transmitted display value to 0..85 C.
         */
        s_icd_env_temp_c = icd_clamp_u8_from_float(t, 0.0f, 85.0f);
        s_icd_env_humidity_pct = icd_clamp_u8_from_float(h, 0.0f, 100.0f);
    } else {
        s_icd_env_temp_c = 0u;
        s_icd_env_humidity_pct = 0u;
    }

    s_icd_env_valid = 1u;
    s_icd_env_last_ms = now;
}

static void fpga_reset_pulse_active_low(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

static void icd_fill_pbit(uint8_t *p, uint32_t len)
{
    memset(p, 0, len);
    if (len < 23u) return;

    icd_refresh_all_ir_presence();

    p[0] = FW_VERSION_MAJOR;
    p[1] = FW_VERSION_MINOR;
    p[2] = FW_VERSION_YEAR;
    p[3] = FW_VERSION_MONTH;
    p[4] = FW_VERSION_DAY;
    p[5] = 0u;
    for (uint8_t i = 0u; i < 6u; ++i) {
        p[6u + i] = icd_ok_code(s_ir_power_on[i] && s_ir_bit_ok[i]);
        p[12u + i] = icd_ok_code(s_day_power_on[i]);
    }
    p[18] = icd_ok_code(fpga_status_ok() ? 1u : 0u);
    p[19] = 0u;
    icd_update_env_sample(0u);
    p[20] = icd_ok_code(s_icd_env_ok);
    p[21] = icd_ok_code(imu_status_ok() ? 1u : 0u);
    p[22] = 0u;
}

static void icd_send_pbit(uint32_t seq)
{
    uint8_t p[23];
    icd_fill_pbit(p, sizeof(p));
    icd_send_frame(ICD_PIPA_RITA_PBIT, seq, p, sizeof(p));
}

static void icd_fill_cbit(uint8_t p[21])
{
    memset(p, 0, 21u);
    p[0] = 0u;
    for (uint8_t i = 0u; i < 6u; ++i) {
        p[1u + i] = icd_ok_code(s_ir_power_on[i] && s_ir_bit_ok[i]);
        p[7u + i] = icd_ok_code(s_day_power_on[i]);
    }
    p[13] = icd_ok_code(imu_status_ok() ? 1u : 0u);
    p[14] = icd_clamp_u8_from_float(s_imu_temp_c, 0.0f, 85.0f);
    p[15] = icd_ok_code(imu_status_ok() ? 1u : 0u);
    p[16] = 0u;
    icd_update_env_sample(0u);
    p[17] = s_icd_env_temp_c;
    p[18] = s_icd_env_humidity_pct;
    p[19] = s_icd_env_ok ? 0u : 1u;
    p[20] = 0u;
}

static void icd_fill_ibit(uint8_t p[28])
{
    memset(p, 0, 28u);
    icd_update_env_sample(1u);
    icd_fill_pbit(p, 23u);

    // icd_fill_pbit() wrote bytes 20-22 using PBIT's own field meaning at
    // those offsets (Temp/Humidity Init, IMU Init, HD-SDI Init). IBIT uses a
    // different, more detailed layout at the same offsets - IMU Output, IMU
    // Temp (a true degC reading, not an ok/fail code), and IMU Temp Status -
    // so overwrite them here with the real values instead of leaving PBIT's
    // unrelated bytes in place.
    p[20] = icd_ok_code(imu_status_ok() ? 1u : 0u);
    p[21] = icd_clamp_u8_from_float(s_imu_temp_c, 0.0f, 85.0f);
    bool imu_temp_in_range = (s_imu_temp_c >= 0.0f) && (s_imu_temp_c <= 85.0f);
    p[22] = icd_ok_code(imu_temp_in_range ? 1u : 0u);

    p[23] = 0u;
    p[24] = s_icd_env_temp_c;
    p[25] = s_icd_env_humidity_pct;
    p[26] = s_icd_env_ok ? 0u : 1u;
    p[27] = 0u;
}

static void icd_service_periodic(McuContext *ctx)
{
    if (!s_icd_enabled || ctx->state != ST_OPERATION) {
        return;
    }

    // Advance any in-flight background IR presence probe (non-blocking). This
    // runs on every service call so the probe progresses smoothly across
    // iterations rather than blocking the main loop.
    icd_service_ir_presence_probe();

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_icd_last_cbit_ms) >= ICD_CBIT_INTERVAL_MS) {
        s_icd_last_cbit_ms = now;
        // Kick the next round-robin presence probe (non-blocking) instead of the
        // old blocking handshake that froze the main loop for up to ~300 ms here.
        icd_start_next_ir_presence();

        uint8_t p[21];
        icd_fill_cbit(p);
        icd_send_frame(ICD_PIPA_RITA_CBIT, 0u, p, sizeof(p));
    }

    // PIPA_RITA_STATUS is periodic in every Accessible Mode, not just BASIC -
    // do not gate this on s_icd_operation_mode.
    if ((uint32_t)(now - s_icd_last_status_ms) >= ICD_STATUS_INTERVAL_MS) {
        s_icd_last_status_ms = now;
        icd_send_status(0u);
    }
}

static uint8_t icd_apply_camera_power_action_single(uint8_t node,
                                                    uint8_t cam_idx,
                                                    uint8_t on_off)
{
    if (cam_idx >= 6u || on_off == 0u || on_off > 2u) {
        return ICD_CAM_ACK_NA;
    }

    uint8_t on = (on_off == 1u) ? 1u : 0u;
    if (camera_power_set(node, cam_idx, on) != 0) {
        return ICD_CAM_ACK_FAIL;
    }

    if (node == NODE_IR && !on) {
        s_ir_bit_ok[cam_idx] = 0u;
    }

    return on ? ICD_CAM_ACK_POWER_ON_OK : ICD_CAM_ACK_POWER_OFF_OK;
}

static void icd_apply_camera_power_action(uint8_t node,
                                          uint8_t cam_id,
                                          uint8_t on_off,
                                          uint8_t *ack_status)
{
    if (ack_status == NULL || on_off == 0u) {
        return;
    }

    if (on_off > 2u || cam_id == 0u || cam_id > 7u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            ack_status[i] = ICD_CAM_ACK_FAIL;
        }
        return;
    }

    if (cam_id == 7u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            ack_status[i] = icd_apply_camera_power_action_single(node, i, on_off);
        }
        return;
    }

    ack_status[cam_id - 1u] = icd_apply_camera_power_action_single(node, (uint8_t)(cam_id - 1u), on_off);
}

static uint8_t icd_apply_ir_nuc_action_single(uint8_t cam_id, uint8_t nuc)
{
    if (cam_id == 0u || cam_id > 6u || nuc == 0u || nuc > 2u) {
        return ICD_CAM_ACK_NA;
    }

    const uint16_t trigger = (nuc == 1u) ? 3u : 4u;
    int rc = Apply_IR_Camera_Param(cam_id, PARAM_ID_IR_NUC_TRIGGER, trigger);
    if (rc != 0) {
        return ICD_CAM_ACK_FAIL;
    }

    s_ir_nuc_status_until_ms[cam_id - 1u] = HAL_GetTick() + ICD_NUC_STATUS_HOLD_MS;
    return ICD_CAM_ACK_NUC_OK;
}

static void icd_apply_ir_nuc_action(uint8_t cam_id, uint8_t nuc, uint8_t *ack_status)
{
    if (ack_status == NULL || nuc == 0u) {
        return;
    }

    if (nuc > 2u || cam_id == 0u || cam_id > 7u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            ack_status[i] = ICD_CAM_ACK_FAIL;
        }
        return;
    }

    if (cam_id == 7u) {
        for (uint8_t i = 0u; i < 6u; ++i) {
            ack_status[i] = icd_apply_ir_nuc_action_single((uint8_t)(i + 1u), nuc);
        }
        return;
    }

    ack_status[cam_id - 1u] = icd_apply_ir_nuc_action_single(cam_id, nuc);
}

void mcu_on_icd_frame(McuContext *ctx,
                      uint16_t msg_id,
                      uint32_t seq,
                      const uint8_t time_tag[8],
                      const uint8_t *payload,
                      uint32_t payload_len)
{
    if (!ctx) return;

    icd_set_time_from_tag(time_tag);
    s_icd_enabled = true;
    if (s_icd_last_cbit_ms == 0u) {
        s_icd_last_cbit_ms = HAL_GetTick();
    }

    switch (msg_id) {
    case ICD_RITA_PIPA_MODE_CONTROL: {
        if (payload_len < 1u) return;
        s_icd_operation_mode = payload[0];
        icd_send_frame(ICD_PIPA_RITA_MODE_CONTROL_ACK, seq, payload, 1u);
        if (payload[0] == ICD_OPERATION_IBIT) {
            uint8_t ibit[28];
            icd_fill_ibit(ibit);
            icd_send_frame(ICD_PIPA_RITA_IBIT, seq, ibit, sizeof(ibit));
        }
        if (payload[0] == ICD_OPERATION_MAINTENANCE) {
            // Every request into Maintenance mode (not just a genuine
            // transition) gets a full 6-camera status burst, so the UI
            // always has each camera's own last-applied/default values
            // freshly re-sent as soon as it opens the maintenance screen.
            icd_send_maintenance_camera_burst(seq);
        }
        // PBIT is sent exactly once after boot (see s_icd_initial_pbit_sent at
        // ST_OPERATION entry) - it must not be re-sent on every mode change.
        icd_send_status(seq);
        break;
    }

    case ICD_RITA_PIPA_CAMERA_CONTROL: {
        if (payload_len < 5u) return;
        uint8_t ack[ICD_CAMERA_ACK_LEN];
        memset(ack, 0, sizeof(ack));
        memcpy(ack, payload, 5u);

        icd_apply_camera_power_action(NODE_DAY, payload[0], payload[1], &ack[5]);
        icd_apply_ir_nuc_action(payload[2], payload[3], &ack[11]);
        icd_apply_camera_power_action(NODE_IR, payload[2], payload[4], &ack[11]);

        /* ACK reports whether the requested action was applied. It is not a
         * camera-health verdict; health is reported by CBIT/status.
         */
        icd_send_frame(ICD_PIPA_RITA_CAMERA_CONTROL_ACK, seq, ack, sizeof(ack));
        icd_send_status(seq);
        break;
    }

    case ICD_RITA_PIPA_PANORAMA_CONTROL: {
        if (payload_len < ICD_PANORAMA_BASE_LEN) return;
        if (payload[0] != 0u) {
            uint8_t mode = icd_mode_from_video_select(payload[0]);
            if (mode != 0xFFu && ctx->state == ST_OPERATION) {
                (void)icd_apply_mode(mode);
            }
        }
        (void)icd_apply_panorama_camera_params(payload);
        uint8_t fpga_reset_ack = 0u;
        uint8_t ack_len = ICD_PANORAMA_BASE_LEN;
        if (payload_len > ICD_PANORAMA_FPGA_RESET_OFF && payload[ICD_PANORAMA_FPGA_RESET_OFF] != 0u) {
            fpga_reset_pulse_active_low();
            fpga_reset_ack = 1u;
            ack_len = ICD_PANORAMA_EXT_LEN;
        }
        uint8_t ack[ICD_PANORAMA_EXT_LEN];
        icd_fill_panorama_ack(ack, ack_len, fpga_reset_ack);
        icd_send_frame(ICD_PIPA_RITA_PANORAMA_ACK, seq, ack, ack_len);
        icd_send_status(seq);
        break;
    }

    case ICD_RITA_PIPA_MAINTENANCE: {
        if (payload_len < 37u) return;
        /* Apply the requested camera/maintenance changes first. 0x22A4 is
         * intentionally emitted only after all synchronous apply attempts have
         * returned, so the ACK/status payload represents the post-apply copy. */
        uint8_t apply_err = icd_apply_maintenance_payload(payload, (uint16_t)payload_len);
        uint8_t ack[ICD_MAINT_ACK_LEN];
        uint8_t status[ICD_MAINT_STATUS_LEN];
        icd_fill_maintenance_ack(ack);
        icd_fill_maintenance_status(status);
        icd_send_frame(ICD_PIPA_RITA_MAINTENANCE_ACK, seq, ack, sizeof(ack));
        icd_send_frame(ICD_PIPA_RITA_MAINTENANCE_STATUS, seq, status, sizeof(status));
        if (apply_err == ERR_OK) {
            icd_send_status(seq);
        }
        break;
    }

    case ICD_RITA_PIPA_UTC_INFO:
        icd_set_time_from_utc_payload(payload, payload_len);
        icd_send_status(seq);
        break;

    default:
        break;
    }
}
