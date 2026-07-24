/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Master firmware - MCU FSM + UI framing + IMU DRDY hook
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "mcu_fsm.h"
#include "proto_uart.h"
#include "fpga_i2c.h"
#include "fpga_regs.h"
#include "imu_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define RET_OK            0
#define RET_ERR_TX       -1
#define RET_ERR_TIMEOUT  -2
#define RET_ERR_RX       -3
#define RET_ERR_SPI      -4

#define IR_NUC_MODE_DISABLED          0u
#define IR_NUC_MODE_AUTO              1u
#define IR_NUC_DEFAULT_INTERVAL_MIN   3u
#define IR_NUC_MIN_INTERVAL_MIN       1u
#define IR_NUC_MAX_INTERVAL_MIN      10u
#define IR_NUC_STAGGER_MS            20u

/* IIM-46234 command packets are always exactly 20 bytes (from ATMEL ref SIZE_PACKET_CMD) */
#define IMU_CMD_PACKET_SIZE  20u


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART3_CMD_READ_TEMP   0xA1  // send this single byte from serial monitor

static uint8_t uart3_cmd_byte __attribute__((unused)) = 0;


volatile int drdy_interrupt = 0;

#include <stdbool.h>
bool g_debug_enabled = false;

extern UART_HandleTypeDef huart3;

// same IDs as your FSM
#define MSG_LOG   0x7F

#ifndef ICD_ONLY_UI_PROTOCOL
#define ICD_ONLY_UI_PROTOCOL 1
#endif

// Keep legacy MSG_LOG frames compiled out while ICD-only protocol is active.
#define TEMP_ENABLE_LEGACY_LOGS 0

// ------------------------------
// UI (UART3) framing buffers
// ------------------------------
#define UI_RX_DMA_MAX      128u
#define UI_ACC_MAX         512u
#define UI_PAYLOAD_MAX     256u
#define ICD_ACC_MAX        1200u
#define ICD_HEADER_SIZE    22u
#define ICD_TIME_TAG_OFF   14u
#define ICD_TAIL_SIZE      6u
#define ICD_MAX_PAYLOAD    1024u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;  // IR cameras
UART_HandleTypeDef huart2;  // Day cameras
UART_HandleTypeDef huart3;  // PC UI (binary COBS+CRC)

/* USER CODE BEGIN PV */

// ---- FSM context ----
static McuContext g_ctx;

// ---- IMU DRDY counters (updated in EXTI ISR) ----
volatile uint32_t g_imu_drdy_count = 0;
volatile uint32_t g_imu_drdy_edges_total = 0;
volatile uint8_t g_vsync_flag = 0;

// ---- UART3 UI RX ----
// Ring of DMA/Idle chunk buffers. A single buffer re-armed in place would let
// a second chunk arriving while the main loop is stuck in a blocking FSM step
// (VISCA waits, HAL_Delay()s) silently overwrite the first, unread one -
// dropping a command. Four slots give plenty of headroom for UI command
// bursts arriving during the worst-case ~6s "apply to all EO cameras" wait.
#define UI_RX_RING_DEPTH 4u
static uint8_t  ui_rx_ring[UI_RX_RING_DEPTH][UI_RX_DMA_MAX];
static volatile uint16_t ui_rx_ring_len[UI_RX_RING_DEPTH];
static volatile uint8_t  ui_rx_ring_head = 0u;   // next slot the ISR will fill
static volatile uint8_t  ui_rx_ring_tail = 0u;   // next slot the main loop will drain
static volatile uint8_t  ui_rx_ring_count = 0u;  // number of filled, unread slots

// Accumulator so we can parse frames split across idle events:
static uint8_t  ui_acc[UI_ACC_MAX];
static uint16_t ui_acc_len = 0;

static uint8_t  icd_acc[ICD_ACC_MAX];
static uint16_t icd_acc_len = 0;
static bool     icd_acc_active = false;


typedef struct {
  volatile int16_t roll_cd;
  volatile int16_t pitch_cd;
  volatile uint16_t rate_hz;
  volatile uint16_t seq;
  volatile bool pending;
} fpga_orient_shadow_t;

static fpga_orient_shadow_t g_fpga_orient = {0};

fpga_i2c_t g_fpga;

static uint32_t g_boot_rcc_csr = 0u;
static uint32_t g_boot_flash_optr = 0u;
static bool g_boot_reset_reported = false;

void fpga_init(void)
{
    fpga_i2c_init(&g_fpga, &hi2c3, FPGA_I2C_ADDR7);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
void Enable_All_Camera_Power(void);
void Disable_All_Camera_Power(void);
int Power_Cycle_All_Cameras(void);
void IMU_Reset(void);
void IMU_SoftReset(void);

/* IMU SPI helpers */
static void delay_us(uint32_t us);
static int imu_drdy_ready(void);
void build_cmd20(const uint8_t *cmd, uint32_t cmd_len, uint8_t out20[IMU_CMD_PACKET_SIZE]);

void Send_UART3_Debug(const char *format, ...);
static void Capture_Boot_Reset_State(void);
static void Report_Boot_Reset_State_Once(void);
const char *mcu_get_reset_cause_str(void);

int send_imu_command(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name);
int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name, uint8_t *rx_out);

// Day Camera (VISCA) Functions
void select_day_camera(uint8_t camera_num);
int send_visca_command(uint8_t* cmd, uint8_t cmd_len, int parse_version);
int wait_for_visca_ack_and_completion(uint32_t timeout_ms, int parse_version);
void parse_and_print_version_response(uint8_t* data, uint8_t len);
int test_day_camera_controls(uint8_t camera_num);

// IR Camera Functions
void select_ir_camera(uint8_t camera_num);
int Send_IR_Camera_Command(uint8_t message_id, uint8_t *params, uint32_t param_size);
void Process_IR_Camera_Received_Data(uint8_t* data, uint16_t length);
int Apply_IR_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
int Apply_EO_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
int Initialize_IR_Camera(uint8_t camera_num);
int ir_camera_handshake(uint8_t camera_num);
int IR_PresenceProbe_Start(uint8_t camera_num);
int IR_PresenceProbe_Poll(void);
bool Validate_IR_Parameter(uint16_t param_id, uint16_t value);
bool Validate_EO_Parameter(uint16_t param_id, uint16_t value);
uint8_t ComputeChecksum(uint8_t* message_buffer, uint32_t length);


/* USER CODE BEGIN PFP */
static void ui_process_bytes(const uint8_t *data, uint16_t len);
static void ui_acc_reset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Day camera (VISCA) variables
uint8_t visca_rx_buffer[64];
volatile uint8_t visca_rx_idx = 0;
volatile uint8_t visca_rx_ready = 0;
uint8_t visca_rx_byte;
uint8_t current_day_camera = 0;  // Currently selected day camera (0-5)

// IR camera variables
volatile bool ir_camera_data_received_flag = false;
volatile uint16_t ir_camera_received_bytes = 0;
uint8_t ir_camera_rx_buffer[128];
uint8_t current_ir_camera = 0;  // Currently selected IR camera (0-5)
static uint8_t s_ir_nuc_mode[6] = {
    IR_NUC_MODE_DISABLED, IR_NUC_MODE_DISABLED, IR_NUC_MODE_DISABLED,
    IR_NUC_MODE_DISABLED, IR_NUC_MODE_DISABLED, IR_NUC_MODE_DISABLED
};
static uint8_t s_ir_nuc_interval_min[6] = {
    IR_NUC_DEFAULT_INTERVAL_MIN, IR_NUC_DEFAULT_INTERVAL_MIN, IR_NUC_DEFAULT_INTERVAL_MIN,
    IR_NUC_DEFAULT_INTERVAL_MIN, IR_NUC_DEFAULT_INTERVAL_MIN, IR_NUC_DEFAULT_INTERVAL_MIN
};
static uint32_t s_ir_nuc_next_due_ms[6] = { 0u, 0u, 0u, 0u, 0u, 0u };
static bool s_ir_camera_present[6] = { false, false, false, false, false, false };


void Enable_All_Camera_Power(void)
{
    // Port C GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // Port A GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    // Port D GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
}

void Disable_All_Camera_Power(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
}

int Power_Cycle_All_Cameras(void)
{
    Disable_All_Camera_Power();
    HAL_Delay(500u);
    Enable_All_Camera_Power();
    HAL_Delay(1000u);
    return RET_OK;
}





#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// same IDs as your FSM
#define MSG_LOG   0x7F

static void ui_logf(const char *fmt, ...) {
#if ICD_ONLY_UI_PROTOCOL && !TEMP_ENABLE_LEGACY_LOGS
    (void)fmt;
    return;
#else
    char text[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    // Internal message format: [type][seq][lenL][lenH][...data...]
    // uart_frame_build() converts it to the SOM/LEN/MessageID/... wire packet.
    uint8_t payload[4 + 200];
    uint16_t n = (uint16_t)strlen(text);
    if (n > 200) n = 200;

    payload[0] = MSG_LOG;
    payload[1] = 0;              // seq unused for logs
    payload[2] = (uint8_t)(n & 0xFF);
    payload[3] = (uint8_t)((n >> 8) & 0xFF);
    memcpy(&payload[4], text, n);

    uint8_t frame[512];
    size_t flen = uart_frame_build(payload, (size_t)(4 + n), frame, sizeof(frame));
    if (flen) {
        HAL_UART_Transmit(&huart3, frame, (uint16_t)flen, 50);
    }
#endif
}

// Backward-compatible name used by older code paths:
void Send_UART3_Debug(const char *format, ...) {
#if ICD_ONLY_UI_PROTOCOL && !TEMP_ENABLE_LEGACY_LOGS
    (void)format;
    return;
#else
    char text[200];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    ui_logf("%s", text);
#endif
}

static void Capture_Boot_Reset_State(void)
{
    g_boot_rcc_csr = RCC->CSR;
    g_boot_flash_optr = FLASH->OPTR;
    g_boot_reset_reported = false;
}

static const char *nrst_mode_text(uint32_t optr)
{
#if defined(FLASH_OPTR_NRST_MODE)
    switch (optr & FLASH_OPTR_NRST_MODE) {
    case FLASH_OPTR_NRST_MODE_0:
        return "input-only";
    case FLASH_OPTR_NRST_MODE_1:
        return "GPIO-only";
    case FLASH_OPTR_NRST_MODE:
        return "input-output";
    default:
        return "reserved";
    }
#else
    (void)optr;
    return "not-supported";
#endif
}

static void Report_Boot_Reset_State_Once(void)
{
    if (g_boot_reset_reported) {
        return;
    }

    const uint32_t csr = g_boot_rcc_csr;
    const bool any_reset_flag =
        ((csr & (RCC_CSR_PINRSTF | RCC_CSR_PWRRSTF | RCC_CSR_SFTRSTF |
                 RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF |
                 RCC_CSR_OBLRSTF)) != 0u);

    Send_UART3_Debug("[BOOT] MCU reset cause: CSR=0x%08lX OPTR=0x%08lX NRST=%s flags:%s%s%s%s%s%s%s%s\r\n",
                     (unsigned long)csr,
                     (unsigned long)g_boot_flash_optr,
                     nrst_mode_text(g_boot_flash_optr),
                     any_reset_flag ? "" : " none",
                     (csr & RCC_CSR_PINRSTF)  ? " PINRST"  : "",
                     (csr & RCC_CSR_PWRRSTF)  ? " PWRRST(POR/BOR/PDR)" : "",
                     (csr & RCC_CSR_SFTRSTF)  ? " SFTRST"  : "",
                     (csr & RCC_CSR_IWDGRSTF) ? " IWDGRST" : "",
                     (csr & RCC_CSR_WWDGRSTF) ? " WWDGRST" : "",
                     (csr & RCC_CSR_LPWRRSTF) ? " LPWRRST" : "",
                     (csr & RCC_CSR_OBLRSTF)  ? " OBLRST"  : "");

    g_boot_reset_reported = true;
}

// Build a human-readable last-reset-cause string from the values captured at boot.
// The UART3 [BOOT] report happens before the UI is connected, so this is emitted to
// the UI log on INIT -> the reset cause is visible without a UART3 terminal.
const char *mcu_get_reset_cause_str(void)
{
    static char buf[160];
    const uint32_t csr = g_boot_rcc_csr;
    const bool any =
        (csr & (RCC_CSR_PINRSTF | RCC_CSR_PWRRSTF | RCC_CSR_SFTRSTF |
                RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF)) != 0u;

    snprintf(buf, sizeof(buf),
             "[BOOT] last reset:%s%s%s%s%s%s%s (CSR=0x%08lX NRST=%s)\r\n",
             any ? "" : " none",
             (csr & RCC_CSR_PINRSTF)  ? " PINRST"          : "",
             (csr & RCC_CSR_PWRRSTF)  ? " PWRRST(BOR/POR)" : "",
             (csr & RCC_CSR_SFTRSTF)  ? " SFTRST"          : "",
             (csr & RCC_CSR_IWDGRSTF) ? " IWDGRST"         : "",
             (csr & RCC_CSR_WWDGRSTF) ? " WWDGRST"         : "",
             (csr & RCC_CSR_LPWRRSTF) ? " LPWRRST"         : "",
             (unsigned long)csr,
             nrst_mode_text(g_boot_flash_optr));
    return buf;
}

/* -------------------------------------------------------------------------- */
/* Microsecond busy-wait (Cortex-M0+ has no DWT cycle counter).              */
/* Calibrated for 16 MHz SYSCLK: ~4 cycles per NOP iteration.               */
/* -------------------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 4u;   /* 16MHz / 4 cycles ? 4 iters/?s */
    while (count--) { __NOP(); }
}

/* -------------------------------------------------------------------------- */
/* spi2_flush ? drain stale bytes from the SPI2 RX FIFO and wait for BSY=0. */
/*                                                                            */
/* WHY: After each CS transaction the STM32 SPI peripheral may have bytes    */
/* sitting in the RX FIFO (received during TX or from a partial read).       */
/* If not drained before the next transaction, HAL_SPI_TransmitReceive will  */
/* return those old bytes as if they were the new response.                  */
/*                                                                            */
/* On STM32G0 SPI: DR read drains RXNE. We loop while RXNE is set.          */
/* -------------------------------------------------------------------------- */
static void spi2_flush(void)
{
    /* Re-assert FRXTH=1 every time: HAL_SPI_Init (and any HAL re-init path)
     * resets CR2, which clears FRXTH back to 0 (16-bit threshold).
     * With FRXTH=0 RXNE only fires after 2 bytes, so the last byte of any
     * odd-length transfer stalls forever.  Always force 8-bit threshold. */
    hspi2.Instance->CR2 |= SPI_CR2_FRXTH;

    /* Wait for BSY=0 (any in-progress transfer to finish) */
    uint32_t t0 = HAL_GetTick();
    while (hspi2.Instance->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - t0) >= 10u) break;
    }
    /* Drain RX FIFO: read DR while RXNE is set */
    while (hspi2.Instance->SR & SPI_SR_RXNE) {
        volatile uint8_t dummy = *(__IO uint8_t *)&hspi2.Instance->DR;
        (void)dummy;
    }
    if (hspi2.Instance->SR & SPI_SR_OVR) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    }
    /* Re-check BSY after drain */
    t0 = HAL_GetTick();
    while (hspi2.Instance->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - t0) >= 5u) break;
    }
}

#ifndef SIZE_RESP_READ_REGS_BASE
#define SIZE_RESP_READ_REGS_BASE  16u
#endif

#define IMU_DRDY_PORT   GPIOB
#define IMU_DRDY_PIN    GPIO_PIN_9

static inline int imu_drdy_is_idle(void)
{
    return (HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN) == GPIO_PIN_SET);
}

static const uint8_t s_imu_spi_zeros[64] = {0};

void IMU_Reset(void) {
    spi2_flush();
    __HAL_SPI_DISABLE(&hspi2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(300);

    {
        uint32_t t0 = HAL_GetTick();
        while (imu_drdy_is_idle()) {
            if ((HAL_GetTick() - t0) >= 200u) break;
        }
        HAL_Delay(50);
        drdy_interrupt = 0;
        HAL_Delay(10);
        drdy_interrupt = 0;
    }
    Send_UART3_Debug("IMU: reset\r\n");
}

void IMU_SoftReset(void) {
    static uint8_t stop_stream_pkt[] = {
        0x24, 0x24,
        0x08,
        0x28,
        0x00, 0x28,
        0x0D, 0x0A
    };

    send_imu_command_ex(stop_stream_pkt, sizeof(stop_stream_pkt),
                        IMU_SIZE_RESP_ACK, "STOP_STREAMING", NULL);

    drdy_interrupt = 0;
    Send_UART3_Debug("IMU: soft reset (STOP_STREAMING)\r\n");
}

void build_cmd20(const uint8_t *cmd, uint32_t cmd_len, uint8_t out20[IMU_CMD_PACKET_SIZE])
{
    memset(out20, 0, IMU_CMD_PACKET_SIZE);
    uint32_t copy_len = (cmd_len < IMU_CMD_PACKET_SIZE) ? cmd_len : IMU_CMD_PACKET_SIZE;
    memcpy(out20, cmd, copy_len);
}

static int imu_drdy_ready(void) __attribute__((unused));
static int imu_drdy_ready(void)
{
    if (drdy_interrupt) {
        drdy_interrupt = 0;
        return 1;
    }
    return 0;
}

static HAL_StatusTypeDef imu_rx_one(uint8_t *out) __attribute__((unused));
static HAL_StatusTypeDef imu_rx_one(uint8_t *out)
{
    uint8_t tx = 0x00u;
    return HAL_SPI_TransmitReceive(&hspi2, &tx, out, 1, 10);
}

static int imu_spi_wait_sr_set(uint32_t mask, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((hspi2.Instance->SR & mask) != mask) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return -1;
        }
    }
    return 0;
}

static int imu_spi_xfer_byte(uint8_t tx, uint8_t *rx, uint32_t timeout_ms)
{
    if (imu_spi_wait_sr_set(SPI_SR_TXE, timeout_ms) != 0) {
        return -1;
    }

    *(__IO uint8_t *)&hspi2.Instance->DR = tx;

    if (imu_spi_wait_sr_set(SPI_SR_RXNE, timeout_ms) != 0) {
        return -2;
    }

    if (rx) {
        *rx = *(__IO uint8_t *)&hspi2.Instance->DR;
    } else {
        volatile uint8_t discard = *(__IO uint8_t *)&hspi2.Instance->DR;
        (void)discard;
    }

    return 0;
}

static void imu_spi_begin_transaction(void)
{
    IMU_SPI_ResetSession();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    __HAL_SPI_ENABLE(&hspi2);
    delay_us(IMU_NSS_SETUP_US);
}

static void imu_spi_end_transaction(void)
{
    uint32_t t0 = HAL_GetTick();
    while (hspi2.Instance->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - t0) >= 10u) break;
    }
    delay_us(IMU_NSS_HOLD_US);
    __HAL_SPI_DISABLE(&hspi2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    delay_us(IMU_NSS_HIGH_US);
    IMU_SPI_ResetSession();
}

void IMU_SPI_ResetSession(void)
{
    spi2_flush();
    __HAL_SPI_DISABLE(&hspi2);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
    hspi2.Instance->CR2 |= SPI_CR2_FRXTH;
    spi2_flush();
}

int IMU_SPI_WriteSession(const uint8_t *tx, uint16_t len, uint32_t timeout_ms)
{
    if (tx == NULL || len == 0u) return -1;

    imu_spi_begin_transaction();
    for (uint16_t i = 0u; i < len; i++) {
        if (imu_spi_xfer_byte(tx[i], NULL, timeout_ms) != 0) {
            imu_spi_end_transaction();
            return -1;
        }
    }
    imu_spi_end_transaction();

    return 0;
}

int IMU_SPI_ReadSession(uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    if (rx == NULL || len == 0u) return -1;

    imu_spi_begin_transaction();
    for (uint16_t i = 0u; i < len; i++) {
        if (imu_spi_xfer_byte(0x00u, &rx[i], timeout_ms) != 0) {
            imu_spi_end_transaction();
            return -1;
        }
    }
    imu_spi_end_transaction();

    return 0;
}

static uint8_t *imu_find_complete_packet(uint8_t *buf, uint32_t buf_len,
                                         uint32_t pkt_len, uint32_t *offset_out)
{
    if (buf == NULL || pkt_len < 4u || buf_len < pkt_len) {
        return NULL;
    }

    for (uint32_t i = 0u; i + pkt_len <= buf_len; i++) {
        if (buf[i] == 0x23u && buf[i + 1u] == 0x23u &&
            buf[i + pkt_len - 2u] == 0x0Du &&
            buf[i + pkt_len - 1u] == 0x0Au) {
            if (offset_out) *offset_out = i;
            return &buf[i];
        }
    }

    return NULL;
}

static uint8_t *imu_find_packet_header(uint8_t *buf, uint32_t buf_len, uint32_t *offset_out)
{
    if (buf == NULL || buf_len < 2u) {
        return NULL;
    }

    for (uint32_t i = 0u; i + 1u < buf_len; i++) {
        if (buf[i] == 0x23u && buf[i + 1u] == 0x23u) {
            if (offset_out) *offset_out = i;
            return &buf[i];
        }
    }

    return NULL;
}

static int imu_spi_write_pulsed(const uint8_t *tx, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef st = HAL_OK;

    if (tx == NULL || len == 0u) return -1;

    IMU_SPI_ResetSession();
    for (uint16_t i = 0u; i < len; i++) {
        uint8_t discard = 0u;
        imu_spi_begin_transaction();
        st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx[i], &discard, 1u, timeout_ms);
        imu_spi_end_transaction();
        if (st != HAL_OK) {
            break;
        }
    }
    IMU_SPI_ResetSession();

    return (st == HAL_OK) ? 0 : -1;
}

static int imu_spi_read_pulsed(uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef st = HAL_OK;

    if (rx == NULL || len == 0u) return -1;

    IMU_SPI_ResetSession();
    for (uint16_t i = 0u; i < len; i++) {
        uint8_t tx_zero = 0x00u;
        imu_spi_begin_transaction();
        st = HAL_SPI_TransmitReceive(&hspi2, &tx_zero, &rx[i], 1u, timeout_ms);
        imu_spi_end_transaction();
        if (st != HAL_OK) {
            break;
        }
    }
    IMU_SPI_ResetSession();

    return (st == HAL_OK) ? 0 : -1;
}

static int imu_spi_read_command_packet(uint8_t *rx,
                                       uint16_t rx_cap,
                                       uint16_t pkt_len,
                                       uint32_t timeout_ms,
                                       uint16_t *actual_len,
                                       uint16_t *pkt_offset)
{
    uint16_t count = 0u;
    uint16_t offset = 0u;
    int header_found = 0;

    if (rx == NULL || rx_cap == 0u || pkt_len < 4u) return -1;
    if (actual_len) *actual_len = 0u;
    if (pkt_offset) *pkt_offset = 0u;

    imu_spi_begin_transaction();
    while (count < rx_cap) {
        if (imu_spi_xfer_byte(0x00u, &rx[count], timeout_ms) != 0) {
            imu_spi_end_transaction();
            if (actual_len) *actual_len = count;
            if (pkt_offset) *pkt_offset = offset;
            return -1;
        }

        count++;

        if (!header_found && count >= 2u) {
            if (rx[count - 2u] == 0x23u && rx[count - 1u] == 0x23u) {
                offset = (uint16_t)(count - 2u);
                header_found = 1;
            }
        }

        if (header_found && count >= (uint16_t)(offset + pkt_len)) {
            uint16_t footer = (uint16_t)(offset + pkt_len - 2u);
            if (rx[footer] == 0x0Du && rx[footer + 1u] == 0x0Au) {
                if (actual_len) *actual_len = count;
                if (pkt_offset) *pkt_offset = offset;
                imu_spi_end_transaction();
                return 0;
            }
        }
    }
    imu_spi_end_transaction();

    if (actual_len) *actual_len = count;
    if (pkt_offset) *pkt_offset = offset;
    return -2;
}

static uint32_t imu_cmd_noresp_gap_us(uint8_t cmd_type)
{
    if (cmd_type == CMD_TYPE_START_STREAMING) {
        return IMU_CMD_TWAIT_US_START;
    }
    return IMU_CMD_TNEXT_US;
}

/* -------------------------------------------------------------------------- */
/* send_imu_command_ex ? IIM-46234 SPI command/response                      */
/* Command mode is two separate SPI transactions:                            */
/*   1) Host sends a 20-byte command packet                                  */
/*   2) After DRDY, host starts a new read transaction for ACK/response      */
/* This matches the ATMEL reference and the datasheet half-duplex timing.    */
/* -------------------------------------------------------------------------- */
int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size,
                        uint32_t rx_size, const char *cmd_name,
                        uint8_t *rx_out)
{
    uint8_t tx20[IMU_CMD_PACKET_SIZE];
    if (tx_packet == NULL || tx_size == 0) return RET_ERR_SPI;

    build_cmd20(tx_packet, tx_size, tx20);

    {
        char dbg[120];
        int off = snprintf(dbg, sizeof(dbg), "IMU TX20 [%s]:", cmd_name);
        for (int i = 0; i < (int)IMU_CMD_PACKET_SIZE && off < (int)(sizeof(dbg) - 4); i++)
            off += snprintf(dbg + off, sizeof(dbg) - (size_t)off, " %02X", tx20[i]);
        snprintf(dbg + off, sizeof(dbg) - (size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }

    uint8_t cmd_type = tx20[3];
    int no_resp_cmd =
        (cmd_type == 0x27u) || /* START_STREAMING */
        (cmd_type == 0x28u) || /* STOP_STREAMING  */
        (cmd_type == 0x2Du);   /* SET_UTC_TIME    */

    uint32_t fixed_rx;
    switch (cmd_type) {
    case 0x11u:
        fixed_rx = (uint32_t)SIZE_RESP_READ_REGS_BASE + (uint32_t)tx_packet[5];
        break;
    case 0x20u:
        fixed_rx = 20u;
        break;
    case 0x2Bu:
        fixed_rx = 16u;
        break;
    default:
        fixed_rx = (uint32_t)IMU_SIZE_RESP_ACK;
        break;
    }
    if (rx_size > 0u && rx_size != fixed_rx) fixed_rx = rx_size;
    if (fixed_rx > 70u) fixed_rx = 70u;

    uint32_t edge_base = g_imu_drdy_edges_total;
    drdy_interrupt = 0;

    if (IMU_SPI_WriteSession(tx20, IMU_CMD_PACKET_SIZE, 10u) != 0) {
        uint32_t sr = hspi2.Instance->SR;
        Send_UART3_Debug("IMU: TX fail [%s] SR=0x%04lX\r\n",
                         cmd_name, (unsigned long)sr);
        return RET_ERR_SPI;
    }

    if (no_resp_cmd) {
        if (cmd_type == 0x27u) {
            Send_UART3_Debug("IMU: START_STREAM sent\r\n");
        }
        delay_us(imu_cmd_noresp_gap_us(cmd_type));
        return RET_OK;
    }

    {
        uint32_t resp_timeout_ms = (cmd_type == 0x2Bu) ? 6000u : 20u;
        uint32_t t_wait_start = HAL_GetTick();
        while ((g_imu_drdy_edges_total == edge_base) && (drdy_interrupt == 0)) {
            if ((HAL_GetTick() - t_wait_start) >= resp_timeout_ms) {
                Send_UART3_Debug("IMU: DRDY timeout [%s] edge=%lu pin=%d\r\n",
                                 cmd_name,
                                 (unsigned long)(g_imu_drdy_edges_total - edge_base),
                                 (int)HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN));
                return RET_ERR_TIMEOUT;
            }
        }

        delay_us(IMU_CMD_TWAIT_US_NORMAL);

        {
            uint8_t rx_buf[IMU_RX_TOTAL] = {0};
            uint8_t *pkt_start = NULL;
            uint32_t pkt_offset = 0u;
            uint32_t hdr_offset = 0u;
            uint16_t rx_total = 0u;
            uint16_t rx_pkt_offset = 0u;
            int rx_status = imu_spi_read_command_packet(rx_buf,
                                                        IMU_RX_TOTAL,
                                                        (uint16_t)fixed_rx,
                                                        10u,
                                                        &rx_total,
                                                        &rx_pkt_offset);

            {
                char dbg[120];
                int off = snprintf(dbg, sizeof(dbg), "IMU RX%u [%s]:",
                                   (unsigned)rx_total, cmd_name);
                for (uint32_t i = 0u; i < rx_total && i < 20u && off < (int)(sizeof(dbg) - 4); i++)
                    off += snprintf(dbg + off, sizeof(dbg) - (size_t)off, " %02X", rx_buf[i]);
                snprintf(dbg + off, sizeof(dbg) - (size_t)off, "\r\n");
                Send_UART3_Debug(dbg);
            }

            if (rx_status < 0) {
                uint32_t sr = hspi2.Instance->SR;
                Send_UART3_Debug("IMU: RX fail [%s] SR=0x%04lX status=%d\r\n",
                                 cmd_name, (unsigned long)sr, rx_status);
                return RET_ERR_SPI;
            }

            pkt_start = imu_find_complete_packet(rx_buf, rx_total, fixed_rx, &pkt_offset);
            if (pkt_start == NULL && rx_total >= (uint16_t)(rx_pkt_offset + fixed_rx)) {
                pkt_start = &rx_buf[rx_pkt_offset];
                pkt_offset = rx_pkt_offset;
            }

            if (pkt_start == NULL) {
                if (imu_find_packet_header(rx_buf, rx_total, &hdr_offset) != NULL) {
                    Send_UART3_Debug("IMU: incomplete pkt hdr=%lu need=%lu got=%lu [%s]\r\n",
                                     (unsigned long)hdr_offset,
                                     (unsigned long)fixed_rx,
                                     (unsigned long)(rx_total - hdr_offset),
                                     cmd_name);
                } else {
                    Send_UART3_Debug("IMU: no 0x2323 hdr [%s] edge=%lu pin=%d\r\n",
                                     cmd_name,
                                     (unsigned long)(g_imu_drdy_edges_total - edge_base),
                                     (int)HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN));
                }
                return RET_ERR_RX;
            }

            uint8_t *pkt = pkt_start;

            if (pkt_offset > 0u) {
                Send_UART3_Debug("IMU: WARN echo/shift %lu [%s]\r\n",
                                 (unsigned long)pkt_offset, cmd_name);
            }

            if (pkt[fixed_rx-2u] != 0x0Du || pkt[fixed_rx-1u] != 0x0Au) {
                Send_UART3_Debug("IMU: bad footer [%s] %02X %02X\r\n",
                                 cmd_name, pkt[fixed_rx-2u], pkt[fixed_rx-1u]);
            }

            if (fixed_rx >= 7u) {
                uint16_t stored = ((uint16_t)pkt[fixed_rx-4u] << 8) | pkt[fixed_rx-3u];
                uint16_t calc = 0u;
                for (uint32_t ci = 3u; ci < (fixed_rx - 4u); ci++) calc += pkt[ci];
                if (stored != calc) {
                    Send_UART3_Debug("IMU: chk FAIL [%s] stored=%04X calc=%04X\r\n",
                                     cmd_name, (unsigned)stored, (unsigned)calc);
                }
            }

            if (cmd_type == 0x12u && pkt[4] != 0x00u) {
                Send_UART3_Debug("IMU: WRITE_REG err=0x%02X [%s]\r\n", pkt[4], cmd_name);
                return RET_ERR_RX;
            }

            if (rx_out) memcpy(rx_out, pkt, fixed_rx);
        }

        {
            uint32_t tnext_start = HAL_GetTick();
            while (!imu_drdy_is_idle()) {
                if ((HAL_GetTick() - tnext_start) >= 20u) {
                    Send_UART3_Debug("IMU: DRDY stayed low after RX [%s]\r\n", cmd_name);
                    break;
                }
            }
        }

        drdy_interrupt = 0;
        delay_us(IMU_CMD_TNEXT_US);
        return RET_OK;
    }
}

/* Backward-compatible wrapper */
int send_imu_command(uint8_t *tx_packet, uint32_t tx_size,
                     uint32_t rx_size, const char *cmd_name)
{
    return send_imu_command_ex(tx_packet, tx_size, rx_size, cmd_name, NULL);
}



// ===================== DAY CAMERA (VISCA) FUNCTIONS =====================
void select_day_camera(uint8_t camera_num) {
    if (camera_num > 5) camera_num = 5;
    if (current_day_camera == camera_num) {
        return;
    }
    current_day_camera = camera_num;

    // Set day camera multiplexer control pins (A=PC3, B=PB1, C=PB2)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (camera_num & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (camera_num & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (camera_num & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // C

    HAL_Delay(10);
    Send_UART3_Debug("Selected day camera %d\r\n", camera_num);
}

void parse_and_print_version_response(uint8_t* data, uint8_t len) {
    uint8_t pkt[16];
    uint8_t pkt_len = 0;
    uint8_t start = 0;
    bool found = false;

    while (start < len) {
        uint8_t end = start;
        while (end < len && data[end] != 0xFF) {
            end++;
        }

        if (end < len) {
            uint8_t cur_len = (uint8_t)(end - start + 1u);
            if (cur_len >= 7u && data[start] == 0x90u && data[start + 1u] == 0x50u) {
                memcpy(pkt, &data[start], cur_len);
                pkt_len = cur_len;
                found = true;
                break;
            }
            start = (uint8_t)(end + 1u);
        } else {
            break;
        }
    }

    if (!found || pkt_len < 7u) {
        ui_logf("Day camera version response not found (len=%u)\r\n", len);
        return;
    }

    uint8_t address = (pkt[0] & 0x0F);
    (void)address;
    uint16_t model_code = ((uint16_t)pkt[2] << 8) | pkt[3];
    uint8_t fw_major = pkt[4];
    uint8_t fw_minor = pkt[5];

    ui_logf("Day Camera %d - Model: 0x%04X, FW: %d.%03d\r\n",
            current_day_camera, model_code, fw_major, fw_minor);
}

// was: void wait_for_visca_ack_and_completion(...)
int wait_for_visca_ack_and_completion(uint32_t timeout_ms, int parse_version)
{
    uint32_t start = HAL_GetTick();

    // Example: wait until visca_rx_ready set by RX ISR/parser
    while (!visca_rx_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) {
        	Send_UART3_Debug("RET Timeout");
        	return RET_ERR_TIMEOUT;

        }
    }

    if (parse_version) {
        uint32_t version_wait_start = HAL_GetTick();
        while ((HAL_GetTick() - version_wait_start) < 300u) {
            bool saw_version = false;
            uint8_t i = 0u;
            while (i + 6u < visca_rx_idx) {
                uint8_t end = i;
                while (end < visca_rx_idx && visca_rx_buffer[end] != 0xFFu) {
                    end++;
                }
                if (end < visca_rx_idx) {
                    if ((end - i + 1u) >= 7u &&
                        visca_rx_buffer[i] == 0x90u &&
                        visca_rx_buffer[i + 1u] == 0x50u) {
                        saw_version = true;
                        break;
                    }
                    i = (uint8_t)(end + 1u);
                } else {
                    break;
                }
            }
            if (saw_version) break;
            HAL_Delay(1);
        }
        parse_and_print_version_response(visca_rx_buffer, visca_rx_idx);
    }

//    Send_UART3_Debug("ACK Received! \r\n");
    return RET_OK;

}


int send_visca_command(uint8_t* cmd, uint8_t cmd_len, int parse_version)
{
    if (cmd == NULL || cmd_len == 0) return RET_ERR_TX;

    visca_rx_idx = 0;
    visca_rx_ready = 0;
    memset(visca_rx_buffer, 0, sizeof(visca_rx_buffer));

    // Start RX first (but RX might already be active => HAL_BUSY is OK)
    HAL_StatusTypeDef rxst = HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);

    if (rxst == HAL_BUSY) {
        // Option A: just accept BUSY (RX already active)
        // do nothing
    } else if (rxst != HAL_OK) {
        // Option B (more aggressive): abort and try once more
        (void)HAL_UART_AbortReceive_IT(&huart2);
        rxst = HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);
        if (rxst != HAL_OK && rxst != HAL_BUSY){

        	return RET_ERR_RX;
        }
    }

    // TX
    if (HAL_UART_Transmit(&huart2, cmd, cmd_len, 1000) != HAL_OK){
    	return RET_ERR_TX;
    	Send_UART3_Debug("Transmit Error! \r\n");
    }


    // Wait for ACK/completion (0 OK, <0 fail/timeout)
    return wait_for_visca_ack_and_completion(1000, parse_version);
}


// Return 0 on success, negative on failure
int test_day_camera_controls(uint8_t camera_num)
{
    select_day_camera(camera_num);
    HAL_Delay(10);

    Send_UART3_Debug("Testing Day Camera %d controls...\r\n", camera_num);

    static uint8_t visca_version_inquiry[] = { 0x81, 0x09, 0x00, 0x02, 0xFF };

    // If send_visca_command has a return value, use it.
    // If it returns void today, change it to return int (0 success / <0 fail).
    int r = send_visca_command(visca_version_inquiry,
                              (uint8_t)sizeof(visca_version_inquiry),
                              1);

    HAL_Delay(10);

    if (r != 0) {
        Send_UART3_Debug("Day Camera %d: VISCA version inquiry send FAIL (r=%d)\r\n", camera_num, r);
        return RET_ERR_TX;
    }

    // Future improvement: wait for VISCA ACK/response flag with timeout and return -2 on timeout.
    Send_UART3_Debug("CAMERA %d OK!! \r\n\n", camera_num);
    return RET_OK;

}



// ===================== IR CAMERA FUNCTIONS =====================
void select_ir_camera(uint8_t camera_num) {
    if (camera_num > 5) camera_num = 5;
    current_ir_camera = camera_num;

        // Set IR camera multiplexer control pins (A=PC0, B=PC1, C=PC2)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (camera_num & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // A
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (camera_num & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (camera_num & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // C

    HAL_Delay(10);
    Send_UART3_Debug("Selected IR camera %d\r\n", camera_num);
}

uint8_t ComputeChecksum(uint8_t* message_buffer, uint32_t length) {
    uint8_t sum = 0;
    for(uint32_t i = 0; i < length; i++) sum += message_buffer[i];
    return (uint8_t)((~sum) + 1);
}

// 2) IR camera UART: change to return int
int Send_IR_Camera_Command(uint8_t message_id, uint8_t *params, uint32_t param_size)
{
    uint8_t command_buffer[32];

    if (param_size > (sizeof(command_buffer) - 4u))
        return RET_ERR_TX;

    command_buffer[0] = 0x01;
    command_buffer[1] = message_id;
    command_buffer[2] = (uint8_t)param_size;

    if (param_size > 0 && params != NULL) {
        memcpy(&command_buffer[3], params, param_size);
    }

    uint32_t body_length = 3u + param_size;
    command_buffer[body_length] = ComputeChecksum(command_buffer, body_length);

    if (HAL_UART_Transmit(&huart1, command_buffer, (uint16_t)(body_length + 1u), 100) != HAL_OK)
        return RET_ERR_TX;

    return RET_OK;
}


bool Validate_IR_Parameter(uint16_t param_id, uint16_t value) {
    switch(param_id) {
        case PARAM_ID_IR_NUC_COMMAND:    return value <= IR_NUC_MODE_AUTO;
        case PARAM_ID_IR_NUC_INTERVAL:   return value >= IR_NUC_MIN_INTERVAL_MIN && value <= IR_NUC_MAX_INTERVAL_MIN;
        case PARAM_ID_IR_NUC_TRIGGER:    return value == 3u || value == 4u;
        case PARAM_ID_IR_IMAGE_FLIP:     return value <= 3u;
        case PARAM_ID_IR_FRAME_RATE:     return value <= 8u;
        // ICD document states RangeMax=4096 for both of these, but the field
        // is a 12-bit-scale camera register value where 4095 (0xFFF) is the
        // true maximum on hardware - kept at 4095 intentionally, not changed
        // to match the document. See the maintenance defaults/options
        // workbook for the documented discrepancy.
        case PARAM_ID_IR_ICE_GAIN_BIAS:  return value <= 4095u;
        case PARAM_ID_IR_ICE_LEVEL_BIAS: return value <= 4095u;
        case PARAM_ID_IR_ICE_STRENGTH:   return value <= 7u;
        case PARAM_ID_IR_PALETTE:        return value <= 6u;
        case PARAM_ID_IR_POLARITY:       return value <= 1u;
        case PARAM_ID_IR_D_ZOOM:         return value <= 12u;
        case PARAM_ID_IR_TNR_ENABLE:     return value <= 1u;
        case PARAM_ID_IR_TNR_THRESHOLD:  return value <= 32u;
        default: return false;
    }
}

bool Validate_EO_Parameter(uint16_t param_id, uint16_t value) {
    switch(param_id) {
        case PARAM_ID_EO_POWER_RESET:    return value <= 1u;
        // ICD RangeMax=50: 0-5 zoom-level presets, 6-11 digital-zoom motion
        // commands, and a documented (but illegibly encoded in the source ICD
        // spreadsheet) "Digital Zoom Direct" position sub-range up to 50 that
        // is not otherwise implemented - values 12-49 pass validation here but
        // still correctly fail at apply time (apply_eo_param_single's D-Zoom
        // switch has no case for them) until that sub-encoding is clarified.
        case PARAM_ID_EO_D_ZOOM:         return value <= 50u;
        case PARAM_ID_EO_WHITE_BALANCE:  return value <= 6u;
        case PARAM_ID_EO_RED_GAIN:       return value <= 14u;
        case PARAM_ID_EO_BLUE_GAIN:      return value <= 14u;
        case PARAM_ID_EO_CHROMA_GAIN:    return value <= 14u;
        case PARAM_ID_EO_AGC:            return value <= 10u;
        case PARAM_ID_EO_AE_SHUTTER:     return value <= 21u;
        case PARAM_ID_EO_BRIGHTNESS:     return value <= 20u;
        case PARAM_ID_EO_BLC:            return value <= 2u;
        case PARAM_ID_EO_HLC:            return (value <= 1u) || (value >= 100u && value <= 120u) || value == 210u;
        case PARAM_ID_EO_ACE:            return (value <= 1u) || (value >= 100u && value <= 111u);
        case PARAM_ID_EO_DEFOG:          return (value <= 1u) || (value >= 100u && value <= 111u);
        case PARAM_ID_EO_DNR:            return value <= 4u;
        case PARAM_ID_EO_GAMMA:          return value <= 6u;
        case PARAM_ID_EO_SHARPNESS:      return value <= 10u;
        case PARAM_ID_EO_MIRROR:         return value <= 2u;
        case PARAM_ID_EO_IMAGE_FLIP:     return value <= 2u;
        case PARAM_ID_EO_TRIGGER_MODE:   return value <= 1u;
        case PARAM_ID_EO_TRIG_IN_POLARITY: return value <= 1u;
        case PARAM_ID_EO_STROBE_POLARITY:  return value <= 1u;
        case PARAM_ID_EO_STROBE_MODE:      return value <= 1u;
        case PARAM_ID_EO_EXT_TRIG_DELAY:   return value <= 255u;
        case PARAM_ID_EO_STROBE_DELAY:     return value <= 255u;
        case PARAM_ID_EO_STROBE_WIDTH:     return value >= 1u && value <= 255u;
        default: return false;
    }
}

void Process_IR_Camera_Received_Data(uint8_t* data, uint16_t length) {
    uint16_t current_pos = 0;
    while (current_pos < length) {
        if (data[current_pos] == 0x01) {
            if (current_pos + 3 < length) {
                uint8_t message_id = data[current_pos + 1];
                uint8_t param_len = data[current_pos + 2];
                uint32_t total_msg_len = 4 + param_len;
                if (current_pos + total_msg_len <= length) {
                    uint8_t* msg_ptr = &data[current_pos];
                    if (msg_ptr[total_msg_len - 1] == ComputeChecksum(msg_ptr, total_msg_len - 1)) {
                        if (message_id == 0x00) {
                            Send_UART3_Debug("IR Camera %d Text: %.*s\r\n", current_ir_camera, param_len, (char*)&msg_ptr[3]);
                        } else if (message_id == 0x02) {
                            Send_UART3_Debug("IR Camera %d ACK received\r\n\n\n", current_ir_camera);
                        }
                        current_pos += total_msg_len; continue;
                    }
                }
            }
        }
        current_pos++;
    }
}

static bool IR_LogVersionResponse(uint8_t camera_num, uint8_t *data, uint16_t length)
{
    bool saw_valid_response = false;
    uint16_t current_pos = 0;

    while (current_pos < length) {
        if (data[current_pos] == 0x01) {
            if (current_pos + 3 < length) {
                uint8_t message_id = data[current_pos + 1];
                uint8_t param_len = data[current_pos + 2];
                uint32_t total_msg_len = 4u + param_len;

                if (current_pos + total_msg_len <= length) {
                    uint8_t* msg_ptr = &data[current_pos];
                    if (msg_ptr[total_msg_len - 1] == ComputeChecksum(msg_ptr, total_msg_len - 1)) {
                        if (message_id == 0x00) {
                            ui_logf("IR Camera %d Version: %.*s\r\n",
                                    camera_num, param_len, (char*)&msg_ptr[3]);
                            saw_valid_response = true;
                        } else if (message_id == 0x02) {
                            ui_logf("IR Camera %d Version ACK received\r\n",
                                    camera_num);
                            saw_valid_response = true;
                        }
                    }
                }
            }
        }
        current_pos++;
    }

    return saw_valid_response;
}


int IR_FactoryResetNV(void)
{
    uint8_t cmd[4];
    cmd[0] = 0x01;
    cmd[1] = 0xB3;
    cmd[2] = 0x00;
    cmd[3] = ComputeChecksum(cmd, 3);

    // Arm RX cleanly
    ir_camera_data_received_flag = false;
    ir_camera_received_bytes = 0;
    memset(ir_camera_rx_buffer, 0, sizeof ir_camera_rx_buffer);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);

    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), 200);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000) {
        if (!ir_camera_data_received_flag) continue;
        ir_camera_data_received_flag = false;

        for (uint16_t i = 0; i + 5 < ir_camera_received_bytes; i++) {
            if (ir_camera_rx_buffer[i]     == 0x01 &&
                ir_camera_rx_buffer[i + 1] == 0x02 &&   // ACK
                ir_camera_rx_buffer[i + 3] == 0x00 &&
                ir_camera_rx_buffer[i + 4] == 0xB3) {
                mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV OK\r\n", current_ir_camera);
                return 0; // SUCCESS
            }
        }
    }
    mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV FAILED (no ACK)\r\n", current_ir_camera);
    return -1; // FAILED
}


//IR Init Helpers

/* ir_flush_rx: explicit drain only ??called ONCE before verbose mode command.
 * Must NOT be called from IR_ArmRx ??doing so kills the live ReceiveToIdle_IT
 * that the callback re-arms, breaking the handshake.                        */
static void ir_flush_rx(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    __HAL_UART_CLEAR_OREFLAG(huart);
    uint8_t dummy;
    while (HAL_UART_Receive(huart, &dummy, 1, 0) == HAL_OK) {}
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* IR_ArmRx: matches original working code exactly.
 * Just resets the data-ready flag and re-arms ReceiveToIdle_IT.
 * The callback always re-arms after each received packet, so the UART
 * is always listening.  IR_ArmRx clears the flag so IR_WaitPacket waits
 * for the response to THIS specific command, not a stale one.           */
static void IR_ArmRx(void) {
    ir_camera_data_received_flag = false;
    ir_camera_received_bytes = 0;
    memset(ir_camera_rx_buffer, 0, sizeof ir_camera_rx_buffer);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
}

static int IR_WaitPacket(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (ir_camera_data_received_flag) return 0;
    }
    return -1;
}

/* Parse packets of form:
   [0]=0x01, [1]=msg_id, [2]=len, [3..3+len-1]=payload, [last]=checksum
*/
static int IR_FindPacket(uint8_t wanted_id, uint8_t wanted_len, uint16_t *out_payload_u16) {
    uint16_t n = ir_camera_received_bytes;
    for (uint16_t i = 0; i + 4u <= n; i++) {
        if (ir_camera_rx_buffer[i] != 0x01) continue;

        uint8_t id  = ir_camera_rx_buffer[i+1];
        uint8_t len = ir_camera_rx_buffer[i+2];
        uint16_t total = (uint16_t)(4u + len);
        if (i + total > n) continue;

        uint8_t *p = &ir_camera_rx_buffer[i];
        if (p[total-1] != ComputeChecksum(p, total-1)) continue;

        if (id == wanted_id && (wanted_len == 0xFF || len == wanted_len)) {
            if (out_payload_u16 && len >= 2) {
                *out_payload_u16 = ((uint16_t)p[3] << 8) | p[4];
            }
            return 0;
        }
    }
    return -1;
}

static int IR_SetNV(uint16_t id, uint16_t val) {
    uint8_t p[4] = { (uint8_t)(id>>8), (uint8_t)(id&0xFF), (uint8_t)(val>>8), (uint8_t)(val&0xFF) };

    IR_ArmRx();
    Send_IR_Camera_Command(0xB0, p, 4);

    if (IR_WaitPacket(500) != 0) {
        Send_UART3_Debug("SetNV#%u timeout\r\n", (unsigned)id);
        return -1;
    }

    /* ACK (0x02): setting accepted immediately.                          */
    if (IR_FindPacket(0x02, 0xFF, NULL) == 0) return 0;

    /* NACK (0x06): camera accepted the write but requires a power-cycle  *
     * to activate (e.g. NV17 Genlock Direction, NV18 Genlock Mode).      *
     * Treat as success ??the value IS stored; reboot will apply it.      */
    if (IR_FindPacket(0x06, 0xFF, NULL) == 0) {
        Send_UART3_Debug("SetNV#%u NACK(reboot needed) ??stored OK\r\n", (unsigned)id);
        return 0;
    }

    /* Unknown response ??log first 8 bytes for diagnostics */
    {
        char dbg[80];
        int off = snprintf(dbg, sizeof(dbg), "SetNV#%u unknown resp(%u):", (unsigned)id, ir_camera_received_bytes);
        for (int i = 0; i < 8 && i < (int)ir_camera_received_bytes && off < (int)sizeof(dbg)-4; i++)
            off += snprintf(dbg+off, sizeof(dbg)-(size_t)off, " %02X", ir_camera_rx_buffer[i]);
        snprintf(dbg+off, sizeof(dbg)-(size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }
    return -1;
}

static int IR_GetNV(int id, int *out) {
    uint8_t p[2] = { (uint8_t)(id>>8), (uint8_t)(id&0xFF) };

    IR_ArmRx();
    Send_IR_Camera_Command(0xB5, p, 2);

    if (IR_WaitPacket(500) != 0) {
        Send_UART3_Debug("GetNV#%d timeout\r\n", id);
        return -1;
    }

    uint16_t v = 0;
    if (IR_FindPacket(0x45, 0x02, &v) == 0) { *out = (int)v; return 0; }

    /* Log first 8 bytes so we can identify the actual response packet ID */
    {
        char dbg[80];
        int off = snprintf(dbg, sizeof(dbg), "GetNV#%d unknown resp(%u):", id, ir_camera_received_bytes);
        for (int i = 0; i < 8 && i < (int)ir_camera_received_bytes && off < (int)sizeof(dbg)-4; i++)
            off += snprintf(dbg+off, sizeof(dbg)-(size_t)off, " %02X", ir_camera_rx_buffer[i]);
        snprintf(dbg+off, sizeof(dbg)-(size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }
    return -1;
}

static int IR_SetMux(uint16_t sel) {
    uint8_t p[2] = { sel>>8, sel&0xFF };
    Send_IR_Camera_Command(0xD7, p, 2);               // Digital Video Mux Select
    // ACK to 0xD7 will arrive; no value to read back
    return 0;
}

static int IR_SendCommandAndWaitAck(uint8_t cmd_id, const uint8_t *params, uint8_t param_len)
{
    uint16_t ack_code = 0u;

    IR_ArmRx();
    if (Send_IR_Camera_Command(cmd_id, (uint8_t *)params, param_len) != RET_OK) {
        return RET_ERR_TX;
    }

    if (IR_WaitPacket(500u) != 0) {
        return RET_ERR_TIMEOUT;
    }

    if (IR_FindPacket(0x02, 0x02, &ack_code) == 0 && ack_code == (uint16_t)cmd_id) {
        return RET_OK;
    }

    return RET_ERR_RX;
}

static int IR_SetOrientation(uint16_t orientation)
{
    uint8_t p[2] = { (uint8_t)(orientation >> 8), (uint8_t)(orientation & 0xFF) };
    return IR_SendCommandAndWaitAck(0xCFu, p, 2u);
}

static int IR_SetIceStrength(uint16_t strength)
{
    uint8_t p[2] = { (uint8_t)(strength >> 8), (uint8_t)(strength & 0xFF) };
    return IR_SendCommandAndWaitAck(0x1Eu, p, 2u);
}

static int IR_SetIceGainBias(uint16_t value)
{
    uint8_t p[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return IR_SendCommandAndWaitAck(0x82u, p, 2u);
}

static int IR_SetIceLevelBias(uint16_t value)
{
    uint8_t p[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return IR_SendCommandAndWaitAck(0x83u, p, 2u);
}

static int IR_SetPalette(uint16_t palette)
{
    uint8_t p[2] = { (uint8_t)(palette >> 8), (uint8_t)(palette & 0xFF) };
    return IR_SendCommandAndWaitAck(0x58u, p, 2u);
}

static int IR_SetPolarity(uint16_t white_hot)
{
    return IR_SendCommandAndWaitAck((white_hot != 0u) ? 0x29u : 0x28u, NULL, 0u);
}

static int IR_SetZoom(uint16_t zoom_id)
{
    uint8_t p[2] = { (uint8_t)(zoom_id >> 8), (uint8_t)(zoom_id & 0xFF) };
    return IR_SendCommandAndWaitAck(0xA4u, p, 2u);
}

static void IR_NucScheduleCameraFromNow(uint8_t camera_num)
{
    if (camera_num >= 6u) return;
    s_ir_nuc_next_due_ms[camera_num] = HAL_GetTick() + ((uint32_t)s_ir_nuc_interval_min[camera_num] * 60000u);
}

static int IR_SetAutoCalibration(uint16_t enabled)
{
    uint8_t p[2] = { 0x00u, (uint8_t)(enabled != 0u ? 0x01u : 0x00u) };
    return IR_SendCommandAndWaitAck(0x26u, p, 2u);
}

static bool IR_TargetIsPresent(uint8_t camera_sel)
{
    if (camera_sel == 0u) {
        for (uint8_t cam = 0u; cam < 6u; ++cam) {
            if (s_ir_camera_present[cam]) return true;
        }
        return false;
    }

    if (camera_sel > 6u) return false;
    return s_ir_camera_present[camera_sel - 1u];
}

static int IR_RunNucTrigger(uint8_t camera_num, uint16_t trigger_value)
{
    uint8_t p[2] = { 0x00u, (uint8_t)trigger_value };
    uint16_t ack_code = 0u;

    if (camera_num >= 6u || !s_ir_camera_present[camera_num]) {
        return RET_ERR_RX;
    }

    IR_ArmRx();
    if (Send_IR_Camera_Command(0x27u, p, 2u) != RET_OK) {
        return RET_ERR_TX;
    }

    if (IR_WaitPacket(500u) != 0) {
        /* This command is observed to actuate the shutter on current cameras
         * even when the protocol ACK is not returned. Treat timeout as
         * accepted for known-present cameras so the operator-visible action
         * is not falsely reported as failure. */
        return RET_OK;
    }

    if (IR_FindPacket(0x02, 0x02, &ack_code) == 0 && ack_code == 0x27u) {
        return RET_OK;
    }

    return RET_OK;
}

static int IR_DisableAutoNucCamera(uint8_t camera_num)
{
    if (camera_num >= 6u) return RET_ERR_RX;

    select_ir_camera(camera_num);
    if (IR_SetAutoCalibration(0u) != RET_OK) {
        return RET_ERR_RX;
    }
    return RET_OK;
}

static int IR_DisableAutoNucTargets(uint8_t camera_sel)
{
    int first_error = RET_OK;
    uint8_t success_count = 0u;

    if (camera_sel == 0u) {
        for (uint8_t cam = 0u; cam < 6u; ++cam) {
            if (!s_ir_camera_present[cam]) {
                continue;
            }
            if (IR_DisableAutoNucCamera(cam) == RET_OK) {
                success_count++;
            } else if (first_error == RET_OK) {
                first_error = RET_ERR_RX;
            }
        }
        return (success_count > 0u) ? RET_OK : first_error;
    }

    if (camera_sel > 6u || !s_ir_camera_present[camera_sel - 1u]) return RET_ERR_RX;
    return IR_DisableAutoNucCamera((uint8_t)(camera_sel - 1u));
}

static int IR_RunSequentialNucSweep(uint8_t camera_sel, const char *reason, uint16_t trigger_value)
{
    int first_error = RET_OK;
    char msg[96];

    if (reason != NULL) {
        snprintf(msg, sizeof(msg), "IR NUC sweep start: %s\r\n", reason);
        mcu_ui_log(&g_ctx, "%s", msg);
    }

    uint8_t first_cam = (camera_sel == 0u) ? 0u : (uint8_t)(camera_sel - 1u);
    uint8_t last_cam = (camera_sel == 0u) ? 5u : (uint8_t)(camera_sel - 1u);
    uint8_t success_count = 0u;

    for (uint8_t cam = first_cam; cam <= last_cam; ++cam) {
        if (!s_ir_camera_present[cam]) {
            continue;
        }
        select_ir_camera(cam);
        int rc = IR_RunNucTrigger(cam, trigger_value);
        if (rc != RET_OK) {
            if (first_error == RET_OK) first_error = rc;
            snprintf(msg, sizeof(msg), "IR NUC cam %u trigger failed rc=%d\r\n",
                     (unsigned)cam, rc);
            mcu_ui_log(&g_ctx, "%s", msg);
        } else {
            success_count++;
        }
        HAL_Delay(IR_NUC_STAGGER_MS);
    }

    if (success_count > 0u) {
        mcu_ui_log(&g_ctx, "IR NUC sweep complete\r\n");
        return RET_OK;
    } else {
        mcu_ui_log(&g_ctx, "IR NUC sweep completed with errors\r\n");
        return first_error;
    }
}

static int apply_ir_param_single(uint8_t camera_num, uint16_t param_id, uint16_t value)
{
    if (camera_num >= 6u) return RET_ERR_RX;
    if (!Validate_IR_Parameter(param_id, value)) return RET_ERR_RX;

    select_ir_camera(camera_num);

    switch (param_id) {
        case PARAM_ID_IR_NUC_COMMAND:
        case PARAM_ID_IR_NUC_INTERVAL:
        case PARAM_ID_IR_NUC_TRIGGER:
            return RET_ERR_RX;
        case PARAM_ID_IR_IMAGE_FLIP:
            return IR_SetOrientation(value);
        case PARAM_ID_IR_FRAME_RATE:
            return IR_SetNV(16u, value);
        case PARAM_ID_IR_ICE_GAIN_BIAS:
            if (IR_SetNV(39u, value) != RET_OK) return RET_ERR_RX;
            return IR_SetIceGainBias(value);
        case PARAM_ID_IR_ICE_LEVEL_BIAS:
            if (IR_SetNV(40u, value) != RET_OK) return RET_ERR_RX;
            return IR_SetIceLevelBias(value);
        case PARAM_ID_IR_ICE_STRENGTH:
            return IR_SetIceStrength(value);
        case PARAM_ID_IR_PALETTE:
            return IR_SetPalette(value);
        case PARAM_ID_IR_POLARITY:
            return IR_SetPolarity(value);
        case PARAM_ID_IR_D_ZOOM:
            return IR_SetZoom(value);
        case PARAM_ID_IR_TNR_ENABLE:
            return IR_SetNV(89u, value);
        case PARAM_ID_IR_TNR_THRESHOLD:
            return IR_SetNV(90u, value);
        default:
            return RET_ERR_RX;
    }
}

static int day_send_visca_bytes(uint8_t camera_num, const uint8_t *cmd, uint8_t cmd_len)
{
    if (camera_num >= 6u || cmd == NULL || cmd_len == 0u) return RET_ERR_TX;
    select_day_camera(camera_num);
    HAL_Delay(10);
    return send_visca_command((uint8_t *)cmd, cmd_len, 0);
}

static int day_send_visca_simple(uint8_t camera_num, uint8_t b2, uint8_t b3, uint8_t b4)
{
    uint8_t cmd[] = { 0x81u, 0x01u, b2, b3, b4, 0xFFu };
    return day_send_visca_bytes(camera_num, cmd, (uint8_t)sizeof(cmd));
}

static int day_send_visca_u8(uint8_t camera_num, uint8_t b2, uint8_t b3, uint8_t value)
{
    uint8_t cmd[] = { 0x81u, 0x01u, b2, b3, (uint8_t)(value & 0x0Fu), 0xFFu };
    return day_send_visca_bytes(camera_num, cmd, (uint8_t)sizeof(cmd));
}

static int day_send_visca_sub_u8(uint8_t camera_num, uint8_t b2, uint8_t b3, uint8_t sub, uint8_t value)
{
    uint8_t cmd[] = { 0x81u, 0x01u, b2, b3, sub, (uint8_t)(value & 0x0Fu), 0xFFu };
    return day_send_visca_bytes(camera_num, cmd, (uint8_t)sizeof(cmd));
}

static int day_send_visca_u16_direct(uint8_t camera_num, uint8_t b2, uint8_t b3, uint16_t value)
{
    uint8_t cmd[] = { 0x81u, 0x01u, b2, b3, 0x00u, 0x00u, (uint8_t)((value >> 4) & 0x0Fu), (uint8_t)(value & 0x0Fu), 0xFFu };
    return day_send_visca_bytes(camera_num, cmd, (uint8_t)sizeof(cmd));
}

static int day_send_visca_sub_u16(uint8_t camera_num, uint8_t b2, uint8_t b3, uint8_t sub, uint16_t value)
{
    uint8_t cmd[] = { 0x81u, 0x01u, b2, b3, sub, 0x00u, (uint8_t)((value >> 4) & 0x0Fu), (uint8_t)(value & 0x0Fu), 0xFFu };
    return day_send_visca_bytes(camera_num, cmd, (uint8_t)sizeof(cmd));
}

enum {
    EO_MENUKEY_UP = 0x01u,
    EO_MENUKEY_DOWN = 0x02u,
    EO_MENUKEY_LEFT = 0x04u,
    EO_MENUKEY_RIGHT = 0x08u,
    EO_MENUKEY_MENU = 0x10u,
    EO_MENUKEY_ESC = 0x20u,
};

static int day_send_menu_key(uint8_t camera_num, uint8_t key)
{
    return day_send_visca_u8(camera_num, 0x04u, 0x16u, key);
}

static int day_menu_wait(void)
{
    HAL_Delay(50);
    return RET_OK;
}

static int day_menu_key_and_wait(uint8_t camera_num, uint8_t key)
{
    int rc = day_send_menu_key(camera_num, key);
    if (rc != RET_OK) return rc;
    return day_menu_wait();
}

static int day_menu_repeat(uint8_t camera_num, uint8_t key, uint16_t count)
{
    for (uint16_t i = 0u; i < count; ++i) {
        int rc = day_menu_key_and_wait(camera_num, key);
        if (rc != RET_OK) return rc;
    }
    return RET_OK;
}

static int day_menu_normalize_to_top(uint8_t camera_num, uint16_t max_rows)
{
    return day_menu_repeat(camera_num, EO_MENUKEY_UP, (uint16_t)(max_rows + 2u));
}

static int day_open_trigger_menu(uint8_t camera_num)
{
    int rc = RET_OK;
    rc = day_menu_repeat(camera_num, EO_MENUKEY_ESC, 3u);
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_MENU);
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_DOWN);
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_MENU);
    return rc;
}

static int day_close_menu(uint8_t camera_num)
{
    return day_menu_repeat(camera_num, EO_MENUKEY_ESC, 2u);
}

static int day_open_exposure_menu(uint8_t camera_num)
{
    int rc = RET_OK;
    rc = day_menu_repeat(camera_num, EO_MENUKEY_ESC, 3u);
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_MENU);
    if (rc != RET_OK) return rc;
    rc = day_menu_repeat(camera_num, EO_MENUKEY_DOWN, 2u);
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_MENU);
    return rc;
}

static int day_save_current_submenu(uint8_t camera_num, uint8_t row, uint8_t item_count)
{
    int rc = day_menu_repeat(camera_num, EO_MENUKEY_DOWN, (uint16_t)(item_count - row));
    if (rc != RET_OK) return rc;
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_RIGHT);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }
    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_MENU);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }
    HAL_Delay(100);
    (void)day_close_menu(camera_num);
    return rc;
}

static int day_set_trigger_row_absolute(uint8_t camera_num, uint8_t row, uint16_t value, uint16_t min_value, uint16_t max_value)
{
    int rc = day_open_trigger_menu(camera_num);
    if (rc != RET_OK) return rc;
    rc = day_menu_normalize_to_top(camera_num, 6u);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }
    rc = day_menu_repeat(camera_num, EO_MENUKEY_DOWN, row);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    rc = day_menu_repeat(camera_num, EO_MENUKEY_LEFT, (uint16_t)(max_value + 2u));
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    if (value > min_value) {
        rc = day_menu_repeat(camera_num, EO_MENUKEY_RIGHT, (uint16_t)(value - min_value));
        if (rc != RET_OK) {
            (void)day_close_menu(camera_num);
            return rc;
        }
    }

    return day_save_current_submenu(camera_num, row, 6u);
}

static int day_ensure_exposure_manual(uint8_t camera_num)
{
    int rc = day_open_exposure_menu(camera_num);
    if (rc != RET_OK) return rc;

    rc = day_menu_normalize_to_top(camera_num, 7u);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    rc = day_menu_key_and_wait(camera_num, EO_MENUKEY_DOWN); /* MODE row */
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    rc = day_menu_repeat(camera_num, EO_MENUKEY_LEFT, 3u);
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    rc = day_menu_repeat(camera_num, EO_MENUKEY_RIGHT, 2u); /* AUTO -> SHUT.P -> MANUAL */
    if (rc != RET_OK) {
        (void)day_close_menu(camera_num);
        return rc;
    }

    return day_save_current_submenu(camera_num, 1u, 7u);
}

static int day_apply_eo_trigger_param(uint8_t camera_num, uint16_t param_id, uint16_t value)
{
    int rc = day_ensure_exposure_manual(camera_num);
    if (rc != RET_OK) return rc;

    switch (param_id) {
        case PARAM_ID_EO_TRIGGER_MODE:
            return day_set_trigger_row_absolute(camera_num, 0u, value, 0u, 1u);
        case PARAM_ID_EO_TRIG_IN_POLARITY:
            return day_set_trigger_row_absolute(camera_num, 1u, value, 0u, 1u);
        case PARAM_ID_EO_STROBE_POLARITY:
            return day_set_trigger_row_absolute(camera_num, 2u, value, 0u, 1u);
        case PARAM_ID_EO_STROBE_MODE:
            /* Manual lists this feature, but the trigger OSD page does not expose a distinct row for it. */
            return RET_ERR_TX;
        case PARAM_ID_EO_EXT_TRIG_DELAY:
            return day_set_trigger_row_absolute(camera_num, 3u, value, 0u, 255u);
        case PARAM_ID_EO_STROBE_DELAY:
            return day_set_trigger_row_absolute(camera_num, 4u, value, 0u, 255u);
        case PARAM_ID_EO_STROBE_WIDTH:
            return day_set_trigger_row_absolute(camera_num, 5u, value, 1u, 255u);
        default:
            return RET_ERR_TX;
    }
}

static int apply_eo_param_single(uint8_t camera_num, uint16_t param_id, uint16_t value)
{
    static const uint8_t dzoom_positions[] = { 0x00u, 0x1Eu, 0x28u, 0x2Du, 0x30u, 0x32u };
    if (camera_num >= 6u) return RET_ERR_TX;
    if (!Validate_EO_Parameter(param_id, value)) return RET_ERR_TX;

    switch (param_id) {
        case PARAM_ID_EO_POWER_RESET:
            return day_send_visca_simple(camera_num, 0x04u, 0x00u, 0x03u);
        case PARAM_ID_EO_D_ZOOM:
            if (value <= 5u) {
                return day_send_visca_u16_direct(camera_num, 0x04u, 0x46u, dzoom_positions[value]);
            }
            switch (value) {
                case 6u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x02u);  // Digital Zoom On
                case 7u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x03u);  // Digital Zoom Off
                case 8u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x00u);  // Digital Zoom Stop
                // Tele/Wide (variable speed) need the full byte 0x27/0x37 sent as-is -
                // day_send_visca_u8() masks its value to the low nibble (for the
                // 0-0xF register-style writes elsewhere), which collapses both 0x27
                // and 0x37 to the same 0x07 and made Tele and Wide indistinguishable.
                case 9u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x27u);  // Digital Zoom Tele
                case 10u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x37u); // Digital Zoom Wide
                case 11u: return day_send_visca_simple(camera_num, 0x04u, 0x06u, 0x10u); // Digital Zoom x1/Max
                default: return RET_ERR_TX;
            }
        case PARAM_ID_EO_WHITE_BALANCE:
            // ICD: 0=Off, 1=Auto, 2=One Push AWB, 3=3700K Manual, 4=5100K
            // Manual, 5=8000K Manual, 6=One Push AWB Trigger. VISCA white
            // balance has no documented distinct "disabled" mode, so 0=Off is
            // treated as a no-op (leave as-is), matching the same N/A
            // convention this ICD already uses for Mirror/Flip/BLC.
            if (value == 0u) return RET_OK;
            switch (value) {
                case 1u: return day_send_visca_simple(camera_num, 0x04u, 0x35u, 0x00u); // Auto
                case 2u: return day_send_visca_simple(camera_num, 0x04u, 0x35u, 0x03u); // One Push AWB
                case 3u: return day_send_visca_simple(camera_num, 0x04u, 0x35u, 0x01u); // 3700K Manual
                case 4u: return day_send_visca_simple(camera_num, 0x04u, 0x35u, 0x02u); // 5100K Manual
                case 5u: return day_send_visca_simple(camera_num, 0x04u, 0x35u, 0x05u); // 8000K Manual
                case 6u: return day_send_visca_simple(camera_num, 0x04u, 0x10u, 0x05u); // One Push AWB Trigger
                default: return RET_ERR_TX;
            }
        case PARAM_ID_EO_RED_GAIN:
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x43u, value);
        case PARAM_ID_EO_BLUE_GAIN:
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x44u, value);
        case PARAM_ID_EO_CHROMA_GAIN:
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x13u, value);
        case PARAM_ID_EO_AGC:
            return day_send_visca_u8(camera_num, 0x04u, 0x2Cu, (uint8_t)value);
        case PARAM_ID_EO_AE_SHUTTER:
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x4Au, value);
        case PARAM_ID_EO_BRIGHTNESS: {
            // Datasheet CAM_ExpComp (the OSD "Brightness" control) ignores Direct
            // position writes (0x4E) unless compensation mode is first enabled
            // via 0x3E 0x02 ("On"). Best-effort: always still issue the Direct
            // write even if the "On" command's ACK wait fails/times out, so a
            // flaky/missing ACK on the enable command can't silently swallow
            // the actual brightness change.
            (void)day_send_visca_simple(camera_num, 0x04u, 0x3Eu, 0x02u);
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x4Eu, value);
        }
        case PARAM_ID_EO_BLC:
            // ICD encoding: 0=N/A (no-op, leave as-is), 1=On, 2=Off.
            if (value == 0u) return RET_OK;
            return day_send_visca_simple(camera_num, 0x04u, 0x33u, (value == 1u) ? 0x02u : 0x03u);
        case PARAM_ID_EO_HLC:
            // ICD: 0=On, 1=Off - opposite of this VISCA register's own 0/1
            // sense, so invert before forwarding to preserve the camera-level
            // command that was already being sent for each logical state.
            if (value == 0u || value == 1u) {
                return day_send_visca_u8(camera_num, 0x04u, 0x32u, (uint8_t)(value == 0u ? 1u : 0u));
            }
            if (value >= 100u && value <= 120u) {
                return day_send_visca_sub_u16(camera_num, 0x04u, 0x32u, 0x10u, (uint16_t)(value - 100u));
            }
            if (value == 210u) {
                return day_send_visca_u8(camera_num, 0x04u, 0x32u, 0x3Au);
            }
            return RET_ERR_TX;
        case PARAM_ID_EO_ACE:
            // ICD: 0=On, 1=Off - flip which wire value triggers VISCA's
            // 0x02 (On)/0x03 (Off) so the camera-level command is unchanged.
            if (value == 0u || value == 1u) {
                return day_send_visca_simple(camera_num, 0x04u, 0x1Au, (value == 0u) ? 0x02u : 0x03u);
            }
            if (value >= 100u && value <= 111u) {
                return day_send_visca_sub_u8(camera_num, 0x04u, 0x1Au, 0x10u, (uint8_t)(value - 100u));
            }
            return RET_ERR_TX;
        case PARAM_ID_EO_DEFOG:
            // ICD: 0=On, 1=Off - same polarity fix as ACE above.
            if (value == 0u || value == 1u) {
                return day_send_visca_simple(camera_num, 0x04u, 0x65u, (value == 0u) ? 0x02u : 0x03u);
            }
            if (value >= 100u && value <= 111u) {
                return day_send_visca_sub_u8(camera_num, 0x04u, 0x65u, 0x10u, (uint8_t)(value - 100u));
            }
            return RET_ERR_TX;
        case PARAM_ID_EO_DNR:
            return day_send_visca_u8(camera_num, 0x04u, 0x53u, (uint8_t)value);
        case PARAM_ID_EO_GAMMA:
            return day_send_visca_u8(camera_num, 0x04u, 0x5Bu, (uint8_t)value);
        case PARAM_ID_EO_SHARPNESS:
            return day_send_visca_u16_direct(camera_num, 0x04u, 0x42u, value);
        case PARAM_ID_EO_MIRROR:
            // ICD encoding: 0=N/A (no-op, leave as-is), 1=On, 2=Off.
            if (value == 0u) return RET_OK;
            return day_send_visca_simple(camera_num, 0x04u, 0x61u, (value == 1u) ? 0x02u : 0x03u);
        case PARAM_ID_EO_IMAGE_FLIP:
            // ICD encoding: 0=N/A (no-op, leave as-is), 1=On, 2=Off - same
            // 3-state pattern as Mirror above.
            if (value == 0u) return RET_OK;
            return day_send_visca_simple(camera_num, 0x04u, 0x66u, (value == 1u) ? 0x02u : 0x03u);
        case PARAM_ID_EO_TRIGGER_MODE:
        case PARAM_ID_EO_TRIG_IN_POLARITY:
        case PARAM_ID_EO_STROBE_POLARITY:
        case PARAM_ID_EO_STROBE_MODE:
        case PARAM_ID_EO_EXT_TRIG_DELAY:
        case PARAM_ID_EO_STROBE_DELAY:
        case PARAM_ID_EO_STROBE_WIDTH:
            return day_apply_eo_trigger_param(camera_num, param_id, value);
        default:
            return RET_ERR_TX;
    }
}

int Apply_IR_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value)
{
    int rc = RET_OK;

    if (!Validate_IR_Parameter(param_id, value)) return RET_ERR_RX;

    if (param_id == PARAM_ID_IR_NUC_INTERVAL) {
        if (!IR_TargetIsPresent(camera_sel)) return RET_ERR_RX;

        if (camera_sel == 0u) {
            for (uint8_t cam = 0u; cam < 6u; ++cam) {
                if (!s_ir_camera_present[cam]) continue;
                s_ir_nuc_interval_min[cam] = (uint8_t)value;
                if (s_ir_nuc_mode[cam] == IR_NUC_MODE_AUTO) {
                    IR_NucScheduleCameraFromNow(cam);
                }
            }
            return RET_OK;
        }

        if (camera_sel > 6u) return RET_ERR_RX;
        s_ir_nuc_interval_min[camera_sel - 1u] = (uint8_t)value;
        if (s_ir_nuc_mode[camera_sel - 1u] == IR_NUC_MODE_AUTO) {
            IR_NucScheduleCameraFromNow((uint8_t)(camera_sel - 1u));
        }
        return RET_OK;
    }

    if (param_id == PARAM_ID_IR_NUC_COMMAND) {
        if (!IR_TargetIsPresent(camera_sel)) return RET_ERR_RX;

        if (IR_DisableAutoNucTargets(camera_sel) != RET_OK) {
            return RET_ERR_RX;
        }

        if (value == IR_NUC_MODE_DISABLED) {
            if (camera_sel == 0u) {
                for (uint8_t cam = 0u; cam < 6u; ++cam) {
                    if (!s_ir_camera_present[cam]) continue;
                    s_ir_nuc_mode[cam] = IR_NUC_MODE_DISABLED;
                    s_ir_nuc_next_due_ms[cam] = 0u;
                }
                mcu_ui_log(&g_ctx, "IR NUC mode set: disabled (all)\r\n");
            } else {
                if (camera_sel > 6u) return RET_ERR_RX;
                s_ir_nuc_mode[camera_sel - 1u] = IR_NUC_MODE_DISABLED;
                s_ir_nuc_next_due_ms[camera_sel - 1u] = 0u;
                mcu_ui_log(&g_ctx, "IR NUC mode set: disabled cam %u\r\n",
                           (unsigned)(camera_sel - 1u));
            }
            return RET_OK;
        }

        if (camera_sel == 0u) {
            for (uint8_t cam = 0u; cam < 6u; ++cam) {
                if (!s_ir_camera_present[cam]) continue;
                s_ir_nuc_mode[cam] = IR_NUC_MODE_AUTO;
                IR_NucScheduleCameraFromNow(cam);
            }
            mcu_ui_log(&g_ctx, "IR NUC mode set: enabled all (%u min)\r\n",
                       (unsigned)s_ir_nuc_interval_min[0]);
        } else {
            if (camera_sel > 6u) return RET_ERR_RX;
            s_ir_nuc_mode[camera_sel - 1u] = IR_NUC_MODE_AUTO;
            IR_NucScheduleCameraFromNow((uint8_t)(camera_sel - 1u));
            mcu_ui_log(&g_ctx, "IR NUC mode set: enabled cam %u (%u min)\r\n",
                       (unsigned)(camera_sel - 1u),
                       (unsigned)s_ir_nuc_interval_min[camera_sel - 1u]);
        }
        return RET_OK;
    }

    if (param_id == PARAM_ID_IR_NUC_TRIGGER) {
        if (!IR_TargetIsPresent(camera_sel)) return RET_ERR_RX;

        if (IR_DisableAutoNucTargets(camera_sel) != RET_OK) {
            return RET_ERR_RX;
        }

        rc = IR_RunSequentialNucSweep(camera_sel, "manual", value);
        if (rc == RET_OK) {
            if (camera_sel == 0u) {
                for (uint8_t cam = 0u; cam < 6u; ++cam) {
                    if (s_ir_nuc_mode[cam] == IR_NUC_MODE_AUTO) {
                        IR_NucScheduleCameraFromNow(cam);
                    }
                }
            } else if (camera_sel <= 6u && s_ir_nuc_mode[camera_sel - 1u] == IR_NUC_MODE_AUTO) {
                IR_NucScheduleCameraFromNow((uint8_t)(camera_sel - 1u));
            }
        }
        return rc;
    }

    if (camera_sel == 0u) {
        // Apply to every camera regardless of individual failures - returning on
        // the first non-OK result (e.g. an absent/unresponsive camera slot) used
        // to abort the sweep, silently skipping every camera after it. That made
        // "All IR Cameras" broadcast a config only as far as the first missing
        // camera, while a single specific camera (which never hits this loop)
        // always worked.
        int worst_rc = RET_OK;
        for (uint8_t cam = 0u; cam < 6u; ++cam) {
            rc = apply_ir_param_single(cam, param_id, value);
            if (rc != RET_OK) worst_rc = rc;
        }
        return worst_rc;
    }

    if (camera_sel > 6u) return RET_ERR_RX;
    return apply_ir_param_single((uint8_t)(camera_sel - 1u), param_id, value);
}

void IR_Nuc_Service(McuContext *ctx)
{
    (void)ctx;

    uint32_t now = HAL_GetTick();
    uint8_t due_list[6];
    uint8_t due_count = 0u;

    for (uint8_t cam = 0u; cam < 6u; ++cam) {
        if (s_ir_nuc_mode[cam] != IR_NUC_MODE_AUTO || s_ir_nuc_next_due_ms[cam] == 0u) {
            continue;
        }

        if ((int32_t)(now - s_ir_nuc_next_due_ms[cam]) >= 0) {
            due_list[due_count++] = cam;
        }
    }

    for (uint8_t i = 0u; i < due_count; ++i) {
        uint8_t cam = due_list[i];
        (void)IR_DisableAutoNucCamera(cam);
        if (IR_RunSequentialNucSweep((uint8_t)(cam + 1u), "auto", 3u) != RET_OK) {
            mcu_ui_log(&g_ctx, "IR auto NUC sweep failed cam %u\r\n", (unsigned)cam);
        }
        if (s_ir_nuc_mode[cam] == IR_NUC_MODE_AUTO) {
            IR_NucScheduleCameraFromNow(cam);
        }
    }
}

void IR_Nuc_OnOperationEnter(void)
{
    for (uint8_t cam = 0u; cam < 6u; ++cam) {
        if (s_ir_nuc_mode[cam] == IR_NUC_MODE_AUTO) {
            IR_NucScheduleCameraFromNow(cam);
        } else {
            s_ir_nuc_next_due_ms[cam] = 0u;
        }
    }
}

int Apply_EO_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value)
{
    int rc = RET_OK;

    if (camera_sel == 0u) {
        // Same fix as Apply_IR_Camera_Param(): keep applying to every camera
        // even if one fails, instead of aborting the whole broadcast sweep on
        // the first non-OK result.
        int worst_rc = RET_OK;
        for (uint8_t cam = 0u; cam < 6u; ++cam) {
            rc = apply_eo_param_single(cam, param_id, value);
            if (rc != RET_OK) worst_rc = rc;
        }
        return worst_rc;
    }

    if (camera_sel > 6u) return RET_ERR_TX;
    return apply_eo_param_single((uint8_t)(camera_sel - 1u), param_id, value);
}




// Return 0 on success, negative on failure
int Initialize_IR_Camera(uint8_t camera_num) {
    select_ir_camera(camera_num);
    HAL_Delay(100);
    s_ir_camera_present[camera_num] = false;

    ui_logf("Initializing IR Camera %d...\r\n", camera_num);

    // Factory reset NV ??restores all NV parameters to defaults before writing ours
    {
        int r = IR_FactoryResetNV();
        if (r != 0) {
            mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV FAILED ??continuing anyway\r\n", camera_num);
        }
        HAL_Delay(10);
    }

    // Enable Verbose Mode
    uint8_t verbose_params[] = {0x00, 0x01};
    Send_IR_Camera_Command(0xFF, verbose_params, sizeof(verbose_params));
    HAL_Delay(100);
    if (ir_camera_data_received_flag) {
        Process_IR_Camera_Received_Data(ir_camera_rx_buffer, ir_camera_received_bytes);
        ir_camera_data_received_flag = false;
        // Do NOT return ??verbose mode response is expected; NV commands must still run
        //Return only when NV commands not required
//        return RET_OK;
    }

    ui_logf("\r\n--- STEP 2: Here Performing Handshake ---\r\n");
    IR_ArmRx();
    Send_IR_Camera_Command(0x07, NULL, 0);

    bool handshake_success = false;
    {
        uint32_t version_wait_start = HAL_GetTick();
        while ((HAL_GetTick() - version_wait_start) < 500u) {
            if (ir_camera_data_received_flag) {
                handshake_success |= IR_LogVersionResponse(camera_num,
                                                           ir_camera_rx_buffer,
                                                           ir_camera_received_bytes);
                ir_camera_data_received_flag = false;
                if (handshake_success) {
                    ui_logf("\r\n--- IR Camera %d version query successful ---\r\n\n",
                            camera_num);
                    break;
                }
            }
            HAL_Delay(1);
        }
    }

    if (!handshake_success) {
        ui_logf("\r\n--- ERROR: Camera %d handshake failed - No valid response ---\r\n\n\n", camera_num);
        s_ir_camera_present[camera_num] = false;
        return RET_ERR_TX; // Exit if handshake failed
    }

    // Enable NTSC Analog Video
//    uint8_t payload[] = {0x00, 0x04, 0x00, 0x01};
//    Send_IR_Camera_Command(0xB0, payload, 4);
//    HAL_Delay(100);
//    if (ir_camera_data_received_flag) {
//        Process_IR_Camera_Received_Data(ir_camera_rx_buffer, ir_camera_received_bytes);
//        ir_camera_data_received_flag = false;
//    }





   IR_SetMux(9);
   IR_SetNV(7, 9);
   IR_SetNV(6,0);
   IR_SetNV(14,1);


    // --- Parallel 8-bit post-AGC on PDVO ---
    IR_SetNV(4, 0);        // NV#4 RS-170 = disable
    IR_SetNV(5, 1);        // NV#5 PDVO = enable
    IR_SetNV(76, 0);






    // Verify PDVO settings
    int nv4=-1,nv5=-1,nv7=-1,nv76=-1;
    IR_GetNV(4,&nv4); IR_GetNV(5,&nv5); IR_GetNV(7,&nv7); IR_GetNV(76,&nv76);
    mcu_ui_log(&g_ctx, "Verify PDVO: NV4=%d NV5=%d NV7=%d NV76=%d\r\n", nv4,nv5,nv7,nv76);

    // --- 30 fps ---
    IR_SetNV(16,1);       // NV#16 Frame Rate = 1 => 30 Hz
    int nv16=-1; IR_GetNV(16,&nv16);
    mcu_ui_log(&g_ctx, "Verify FrameRate: NV16=%d\r\n", nv16);

    // --- Genlock slave / input ---
    IR_SetNV(17, 1);       // NV#17 Genlock Direction = input (slave mode)
    IR_SetNV(18, 0);       // NV#18 Genlock Mode = 0 (slave)

    int nv17=-1,nv18=-1;
    IR_GetNV(17,&nv17); IR_GetNV(18,&nv18);
    mcu_ui_log(&g_ctx, "Verify Genlock: NV17=%d NV18=%d\r\n", nv17, nv18);

    s_ir_nuc_mode[camera_num] = IR_NUC_MODE_DISABLED;
    s_ir_nuc_interval_min[camera_num] = IR_NUC_DEFAULT_INTERVAL_MIN;
    s_ir_nuc_next_due_ms[camera_num] = 0u;

    if (IR_SetAutoCalibration(0u) != RET_OK) {
        mcu_ui_log(&g_ctx, "IR Camera %d auto NUC disable failed\r\n", camera_num);
    } else {
        mcu_ui_log(&g_ctx, "IR Camera %d auto NUC disabled\r\n", camera_num);
    }

    s_ir_camera_present[camera_num] = true;
    mcu_ui_log(&g_ctx, "IR Camera %d initialization complete\r\n", camera_num);
    return RET_OK;
}

// Lightweight IR camera presence/handshake check. Selects the camera's control
// channel and sends only the handshake ping (msg 0x07), then waits briefly for a
// valid ACK (0x02) or text (0x00) reply. Performs NO FactoryResetNV and NO NV
// reconfiguration, so it is safe to call periodically (status checks) without
// disturbing a running camera.
// Parse the currently-buffered IR RX reply and decide whether it indicates a
// present, responsive camera (a valid 0x02 ACK or 0x00 text message with a
// correct checksum). Shared by both the blocking handshake and the
// non-blocking background probe so their acceptance criteria stay identical.
static bool ir_reply_indicates_present(void)
{
    uint16_t current_pos = 0;
    bool present = false;
    while (current_pos < ir_camera_received_bytes) {
        if (ir_camera_rx_buffer[current_pos] == 0x01) {
            if (current_pos + 3 < ir_camera_received_bytes) {
                uint8_t message_id = ir_camera_rx_buffer[current_pos + 1];
                uint8_t param_len  = ir_camera_rx_buffer[current_pos + 2];
                uint32_t total_msg_len = 4u + param_len;
                if (current_pos + total_msg_len <= ir_camera_received_bytes) {
                    uint8_t* msg_ptr = &ir_camera_rx_buffer[current_pos];
                    if (msg_ptr[total_msg_len - 1] == ComputeChecksum(msg_ptr, total_msg_len - 1)) {
                        if (message_id == 0x02) { present = true; break; }
                        else if (message_id == 0x00) { present = true; }
                    }
                }
            }
        }
        current_pos++;
    }
    return present;
}

int ir_camera_handshake(uint8_t camera_num)
{
    if (camera_num > 5u) return RET_ERR_TX;

    select_ir_camera(camera_num);
    HAL_Delay(50);

    IR_ArmRx();

    if (Send_IR_Camera_Command(0x07, NULL, 0) != RET_OK) {
        s_ir_camera_present[camera_num] = false;
        return RET_ERR_TX;
    }

    uint32_t t0 = HAL_GetTick();
    while (!ir_camera_data_received_flag && (HAL_GetTick() - t0) < 250u) {
        /* IR camera UART RX is interrupt-driven; wait briefly for the reply */
        HAL_Delay(1);
    }

    bool handshake_success = false;
    if (ir_camera_data_received_flag) {
        handshake_success = ir_reply_indicates_present();
        ir_camera_data_received_flag = false;
    }

    s_ir_camera_present[camera_num] = handshake_success;
    return handshake_success ? RET_OK : RET_ERR_TX;
}

// ---------------------------------------------------------------------------
// Non-blocking IR presence probe (background CBIT health sweep)
//
// The blocking ir_camera_handshake() above freezes the ENTIRE main loop for
// its mux settle (HAL_Delay(50)) plus up to a 250 ms reply wait. Running that
// every CBIT (1 Hz, round-robin one camera per second) via icd_service_periodic
// meant UI command handling and the 10 Hz PIPA_RITA_STATUS cadence stalled for
// up to ~300 ms once per second - the root cause of "command response time goes
// huge after the board has been running a while": a UI command arriving during
// that window sits unprocessed in the RX ring for the whole freeze.
//
// This state machine performs the IDENTICAL probe (same mux select, same 60 ms
// total settle, same 0x07 ping, same 250 ms reply window, same accept criteria)
// but spreads it across main-loop iterations so no single iteration blocks more
// than the ~1 ms ping transmit. Because it is non-blocking, the settle and wait
// windows cost no main-loop latency and are kept generous to preserve detection
// reliability exactly.
//
// It shares the IR mux + UART1 with normal camera commands, which also run in
// the main loop between polls. If an intervening command re-points the mux, the
// probe ABORTS (leaving presence unchanged) rather than risk misreading another
// camera's traffic - so a command can never cause a present camera to be
// reported absent.
// ---------------------------------------------------------------------------
typedef enum {
    IR_PROBE_IDLE = 0,
    IR_PROBE_SETTLE,   // mux pointed, waiting out the settle window before ping
    IR_PROBE_WAIT      // ping sent, waiting for a reply (or timeout)
} IrProbePhase;

#define IR_PROBE_SETTLE_MS  60u    // matches old select(10ms)+HAL_Delay(50ms)
#define IR_PROBE_WAIT_MS    250u   // matches old blocking reply window

static IrProbePhase s_ir_probe_phase = IR_PROBE_IDLE;
static uint8_t      s_ir_probe_cam   = 0u;
static uint32_t     s_ir_probe_deadline_ms = 0u;

// Begin a probe for one camera. Non-blocking: points the IR mux and starts the
// settle timer, then returns immediately. Returns 0 (RET_OK) if started.
int IR_PresenceProbe_Start(uint8_t camera_num)
{
    if (camera_num > 5u) return RET_ERR_TX;
    if (s_ir_probe_phase != IR_PROBE_IDLE) return RET_ERR_TX;

    // Point the mux directly (skipping select_ir_camera()'s own HAL_Delay(10)
    // and debug print); the settle window below covers the switch time.
    current_ir_camera = camera_num;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (camera_num & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (camera_num & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (camera_num & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    s_ir_probe_cam = camera_num;
    s_ir_probe_deadline_ms = HAL_GetTick() + IR_PROBE_SETTLE_MS;
    s_ir_probe_phase = IR_PROBE_SETTLE;
    return RET_OK;
}

// Advance the probe. Non-blocking; safe to call many times per main-loop
// iteration. Returns: -1 idle, 0 in progress, 1 completed-present,
// 2 completed-absent, 3 aborted (mux disturbed; presence left unchanged).
int IR_PresenceProbe_Poll(void)
{
    if (s_ir_probe_phase == IR_PROBE_IDLE) {
        return -1;
    }

    // If an intervening IR command re-pointed the mux, our probe context is no
    // longer valid - abort without touching presence.
    if (current_ir_camera != s_ir_probe_cam) {
        s_ir_probe_phase = IR_PROBE_IDLE;
        return 3;
    }

    if (s_ir_probe_phase == IR_PROBE_SETTLE) {
        if ((int32_t)(HAL_GetTick() - s_ir_probe_deadline_ms) < 0) {
            return 0;  // still settling
        }
        IR_ArmRx();
        if (Send_IR_Camera_Command(0x07, NULL, 0) != RET_OK) {
            s_ir_camera_present[s_ir_probe_cam] = false;
            s_ir_probe_phase = IR_PROBE_IDLE;
            return 2;  // ping TX failed -> treat as absent, same as blocking path
        }
        s_ir_probe_deadline_ms = HAL_GetTick() + IR_PROBE_WAIT_MS;
        s_ir_probe_phase = IR_PROBE_WAIT;
        return 0;
    }

    // IR_PROBE_WAIT
    if (ir_camera_data_received_flag) {
        bool present = ir_reply_indicates_present();
        ir_camera_data_received_flag = false;
        s_ir_camera_present[s_ir_probe_cam] = present;
        s_ir_probe_phase = IR_PROBE_IDLE;
        return present ? 1 : 2;
    }
    if ((int32_t)(HAL_GetTick() - s_ir_probe_deadline_ms) >= 0) {
        s_ir_camera_present[s_ir_probe_cam] = false;
        s_ir_probe_phase = IR_PROBE_IDLE;
        return 2;  // no reply within the window -> absent
    }
    return 0;  // still waiting
}



void Read_HDC3022_Sensor(float* temp, float* hum) {
    uint8_t sensor_data_buffer[6];
    uint8_t trigger_cmd[] = {0x24, 0x00};
    // This runs on the main loop's hot path (icd_service_periodic(), called
    // from mcu_on_imu_drdy() every DRDY event) once per CBIT interval (~1s),
    // via icd_update_env_sample(). The previous 1000ms HAL timeout on EACH of
    // these two calls meant a single bus glitch or unresponsive sensor could
    // block the whole main loop - IMU processing, UART3 command handling,
    // and PIPA_RITA_STATUS's 100ms cadence - for up to ~2 seconds, once per
    // CBIT interval. A healthy transaction of this size completes in well
    // under 1ms, so a short timeout still gives huge headroom on the happy
    // path while making a failure fail fast instead of stalling everything.
    if (HAL_I2C_Master_Transmit(&hi2c2, (0x44 << 1), trigger_cmd, 2, 20) != HAL_OK) {
        Send_UART3_Debug("I2C Transmit Error\r\n");
        *temp = -999.0f;
        *hum = -999.0f;
        return;
    }
    HAL_Delay(20);
    if (HAL_I2C_Master_Receive(&hi2c2, (0x44 << 1), sensor_data_buffer, 6, 20) != HAL_OK) {
        Send_UART3_Debug("I2C Receive Error\r\n");
        *temp = -999.0f;
        *hum = -999.0f;
        return;
    }
    uint16_t raw_temp = (sensor_data_buffer[0] << 8) | sensor_data_buffer[1];
    uint16_t raw_hum = (sensor_data_buffer[3] << 8) | sensor_data_buffer[4];
    *temp = 175.0f * ((float)raw_temp / 65535.0f) - 45.0f;
    *hum = 100.0f * ((float)raw_hum / 65535.0f);
}

void mcu_on_new_attitude(int16_t roll_cd, int16_t pitch_cd, uint16_t rate_hz) {
  g_fpga_orient.roll_cd = roll_cd;
  g_fpga_orient.pitch_cd = pitch_cd;
  g_fpga_orient.rate_hz = rate_hz;
  g_fpga_orient.seq++;
  g_fpga_orient.pending = true;
}

/* fpga_service_orient is implemented in mcu_fsm.c */



// ------------------------------------------------------------
// EXTI callback
// PB9: IMU DRDY
// PC4: FPGA VSYNC
// ------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_9) { // PB9 DRDY
        drdy_interrupt = 1;       /* used by send_imu_command_ex Phase 2 (command mode) */
        g_imu_drdy_count++;       /* consumed by main loop -> mcu_on_imu_drdy (streaming mode) */
        g_imu_drdy_edges_total++;
    } else if (GPIO_Pin == GPIO_PIN_4) { // PC4 VSYNC
        g_vsync_flag = 1;
    }
}

// ------------------------------------------------------------
// UART Rx Event callback (IDLE)
// UART3 is UI binary stream (COBS delimited by 0x00)
// ------------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Day camera (VISCA)
        if (visca_rx_idx < sizeof(visca_rx_buffer)) {
            visca_rx_buffer[visca_rx_idx++] = visca_rx_byte;
            if (visca_rx_byte == 0xFF) visca_rx_ready = 1;
        }
        HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);
//        Send_UART3_Debug("day cam call triggered\r\n");
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {  // IR camera
        ir_camera_data_received_flag = true;
        ir_camera_received_bytes = Size;
        /* Always re-arm ??keeps the UART listening between NV commands and
         * during the handshake / verbose-mode response window.             */
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
    }
    else if (huart->Instance == USART3) { // UI protocol (COBS + CRC)
        if (ui_rx_ring_count >= UI_RX_RING_DEPTH) {
            // Ring full: drop the oldest unread chunk to make room for the
            // newest rather than overwriting the slot the main loop is about
            // to read. Extremely unlikely in practice (4 slots), but a live
            // control channel should favor the freshest command if it ever
            // happens.
            ui_rx_ring_tail = (uint8_t)((ui_rx_ring_tail + 1u) % UI_RX_RING_DEPTH);
            ui_rx_ring_count--;
        }
        ui_rx_ring_len[ui_rx_ring_head] = Size;
        ui_rx_ring_head = (uint8_t)((ui_rx_ring_head + 1u) % UI_RX_RING_DEPTH);
        ui_rx_ring_count++;

        // re-arm into the next free slot
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_ring[ui_rx_ring_head], UI_RX_DMA_MAX);
    }
}

// ------------------------------------------------------------
// UART error recovery.
//
// On any RX line error (overrun/ORE, framing, noise, parity) the HAL IRQ
// handler aborts the in-progress interrupt reception (RxState -> READY) and
// calls this weak callback. Without re-arming here, that UART goes deaf
// permanently after the FIRST such error. Overruns become increasingly likely
// the longer the board runs because the main loop is busy draining IMU DRDY at
// high ODR and the FSM performs blocking HAL_Delay()s during which incoming
// bytes can overrun. That is the root cause of "commands become unresponsive
// after a large time elapses". Clear the sticky error flags and re-arm the
// affected UART so a transient error self-heals.
// ------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    if (huart->Instance == USART3) {          // UI / ICD command channel
        // Whatever partial bytes were mid-flight into the current head slot
        // are incomplete/corrupt - re-arm into that same slot rather than
        // advancing head (no chunk was actually completed here).
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_ring[ui_rx_ring_head], UI_RX_DMA_MAX);
    } else if (huart->Instance == USART1) {   // IR camera
        (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
    } else if (huart->Instance == USART2) {   // Day camera (VISCA)
        (void)HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);
    }
}


// ------------------------------------------------------------
// UI accumulator helpers
// ------------------------------------------------------------
static void ui_acc_reset(void)
{
    ui_acc_len = 0;
    memset(ui_acc, 0, sizeof(ui_acc));
}

static void icd_acc_reset(void)
{
    icd_acc_len = 0u;
    icd_acc_active = false;
    memset(icd_acc, 0, sizeof(icd_acc));
}

static uint16_t icd_read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t icd_read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
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

static bool icd_process_byte(uint8_t b)
{
    static const uint8_t som_bytes[4] = { 0x83u, 0x78u, 0x71u, 0x70u };
    static const uint8_t eom_bytes[4] = { 0x70u, 0x71u, 0x78u, 0x83u };

    if (!icd_acc_active) {
        if (b != som_bytes[0]) {
            return false;
        }
        icd_acc_reset();
        icd_acc_active = true;
    }

    if (icd_acc_len >= ICD_ACC_MAX) {
        icd_acc_reset();
        return true;
    }

    icd_acc[icd_acc_len++] = b;

    if (icd_acc_len <= 4u) {
        if (icd_acc[icd_acc_len - 1u] != som_bytes[icd_acc_len - 1u]) {
            icd_acc_reset();
        }
        return true;
    }

    if (icd_acc_len < ICD_HEADER_SIZE) {
        return true;
    }

    const uint32_t payload_len = icd_read_le32(&icd_acc[10]);
    if (payload_len > ICD_MAX_PAYLOAD) {
        icd_acc_reset();
        return true;
    }

    const uint32_t total_len = ICD_HEADER_SIZE + payload_len + ICD_TAIL_SIZE;
    if (icd_acc_len < total_len) {
        return true;
    }

    if (icd_acc_len == total_len) {
        const uint32_t crc_off = ICD_HEADER_SIZE + payload_len;
        const uint8_t *payload = &icd_acc[ICD_HEADER_SIZE];
        const uint16_t rx_crc = icd_read_le16(&icd_acc[crc_off]);
        const uint16_t calc_crc = icd_crc16_ccitt_payload(payload, payload_len);

        if (memcmp(&icd_acc[crc_off + 2u], eom_bytes, sizeof(eom_bytes)) == 0 &&
            rx_crc == calc_crc) {
            Report_Boot_Reset_State_Once();
            mcu_on_icd_frame(&g_ctx,
                             icd_read_le16(&icd_acc[4]),
                             icd_read_le32(&icd_acc[6]),
                             &icd_acc[ICD_TIME_TAG_OFF],
                             payload,
                             payload_len);
        }
        icd_acc_reset();
        return true;
    }

    icd_acc_reset();
    return true;
}

// Feed bytes into the ICD parser. Legacy COBS frames are ignored when the
// customer ICD protocol is the only allowed PC-facing protocol.
static void ui_process_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (icd_acc_active || b == 0x83u) {
            if (icd_process_byte(b)) {
                continue;
            }
        }

#if !ICD_ONLY_UI_PROTOCOL
        if (b == 0x00) {
            // delimiter -> parse current frame if any
            if (ui_acc_len > 0) {
                uint8_t payload[UI_PAYLOAD_MAX];
                size_t pay_len = uart_frame_parse(ui_acc, ui_acc_len, payload, sizeof(payload));
                if (pay_len > 0) {
                    Report_Boot_Reset_State_Once();
                    mcu_on_ui_payload(&g_ctx, payload, (uint16_t)pay_len);
                }
            }
            ui_acc_reset();
            continue;
        }

        // normal byte
        if (ui_acc_len < UI_ACC_MAX) {
            ui_acc[ui_acc_len++] = b;
        } else {
            // overflow -> drop
            ui_acc_reset();
        }
#else
        ui_acc_reset();
#endif
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  Capture_Boot_Reset_State();
  HAL_Init();
  __HAL_RCC_CLEAR_RESET_FLAGS();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof(ir_camera_rx_buffer));
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);

  MX_USART3_UART_Init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_ring[ui_rx_ring_head], UI_RX_DMA_MAX);

  MX_I2C2_Init();
  MX_I2C3_Init();
  fpga_i2c_init(&g_fpga, &hi2c3, FPGA_I2C_ADDR7);


  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  // Init FSM context
  mcu_fsm_init(&g_ctx);

  // Arm UART3 UI receiver
  ui_acc_reset();
  icd_acc_reset();
  (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_ring[ui_rx_ring_head], UI_RX_DMA_MAX);


//  Send_UART3_Debug("waiting for INIT");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Consume pending IMU DRDY events (counter prevents edge collapse at high ODR)
    uint32_t pending_drdy = 0;
    __disable_irq();
    pending_drdy = g_imu_drdy_count;
    g_imu_drdy_count = 0;
    __enable_irq();

    while (pending_drdy--) {
        mcu_on_imu_drdy(&g_ctx);
    }

    if (g_vsync_flag) {
        g_vsync_flag = 0;
        mcu_on_vsync(&g_ctx);
    }

    // Consume UI RX only after latency-sensitive IMU/VSYNC servicing.
    // Drain every chunk queued while we were busy (e.g. inside a blocking FSM
    // step) rather than just the newest one, so a burst of commands received
    // during that window is all processed instead of silently lost.
    __disable_irq();
    uint8_t ui_pending = ui_rx_ring_count;
    __enable_irq();

    while (ui_pending > 0u) {
        uint8_t slot = ui_rx_ring_tail;
        uint16_t len = ui_rx_ring_len[slot];

        if (len > 0u) {
            ui_process_bytes(ui_rx_ring[slot], len);
        }

        __disable_irq();
        ui_rx_ring_tail = (uint8_t)((ui_rx_ring_tail + 1u) % UI_RX_RING_DEPTH);
        ui_rx_ring_count--;
        __enable_irq();

        ui_pending--;
    }

    // Progress FSM (contains blocking delays by design)
    mcu_fsm_step(&g_ctx);

    // Reception-health watchdog for the UI/ICD command channel
    // (belt-and-suspenders on top of HAL_UART_ErrorCallback). If UART3 RX is
    // ever found not armed - a missed re-arm, or a HAL edge case after an
    // error - restart it so the command channel can never stay permanently
    // deaf. Rate-limited so it costs nothing at the IMU DRDY loop rate. Scoped
    // to UART3 only: the IR camera UART has its own arm/re-arm handshake
    // discipline and is recovered by HAL_UART_ErrorCallback instead.
    {
        static uint32_t s_rx_health_last_ms = 0u;
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - s_rx_health_last_ms) >= 200u) {
            s_rx_health_last_ms = now_ms;
            // Always clear a sticky overrun flag (no-op when not set): defends
            // against a HAL edge case that leaves ORE set while RxState stays
            // BUSY_RX, which would silently wedge reception.
            __HAL_UART_CLEAR_OREFLAG(&huart3);
            if (huart3.RxState == HAL_UART_STATE_READY) {
                (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_ring[ui_rx_ring_head], UI_RX_DMA_MAX);
            }
        }
    }

    // NOTE: No HAL_Delay() here. The main loop must be free-running so IMU
    // DRDY events at 416 Hz (every 2.4 ms) are processed with minimum latency.
    // Any HAL_Delay() here would add that many ms of lag to EVERY IMU sample.
  }
  /* USER CODE END WHILE */
}

/* ===================== CLOCK / PERIPH INIT ===================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}



static void MX_I2C3_Init(void)
{
  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x0010061A;  // Fast-mode 400 kHz for FPGA register transactions
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* CPOL=1 ??clock idles HIGH */
  hspi2.Init.CLKPhase    = SPI_PHASE_2EDGE;    /* CPHA=1 ??SPI Mode 3 (datasheet confirmed; Atmel NCPHA=0 = standard CPHA=1) */
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  /* ~8 MHz, inside datasheet 4-12 MHz window */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* Force RXFIFO threshold to 8-bit (1 byte).
   * STM32G0 SPI FIFO default threshold is 16-bit ??RXNE only fires after
   * 2 bytes are received.  For 8-bit transfers this causes HAL_SPI_TransmitReceive
   * to stall on the last byte of any odd-length transfer (10-byte ACK,
  * 18-byte READ_REG response, 22-byte drain, etc.).
   * Setting FRXTH=1 makes RXNE fire after every single byte.            */
  hspi2.Instance->CR2 |= SPI_CR2_FRXTH;
  __HAL_SPI_DISABLE(&hspi2);

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC11 PC12 PC13 PC0
                           PC1 PC2 PC3 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB9 interrupt pin (DRDY)
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      // PC4 interrupt pin (FPGA VSYNC)
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      // Enable EXTI4_15 interrupt for PB9
      HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


      // PA0 reset pin (output)
          GPIO_InitStruct.Pin = GPIO_PIN_0;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_PULLUP;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

          // PB12 NSS pin (software-controlled output)
          GPIO_InitStruct.Pin = GPIO_PIN_12;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_PULLUP;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

          // SPI2 pins (PB13 SCK, PB14 MISO, PB15 MOSI)
          GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
          GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
          GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PC6 FPGA_RESET_N, active-low idle high */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
