#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- CRC16 (CCITT-FALSE) ----
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

// ---- COBS ----
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);
size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);

// ---- Framing helpers ----
// Internal caller format remains:
//   [msg_type][seq][len_lo][len_hi][payload...]
//
// On-wire decoded packet is:
//   [SOM][LEN_lo][LEN_hi][MessageID][SEQ][payload...][CRC_lo][CRC_hi][EOM]
//
// The full decoded packet is COBS-encoded and terminated with 0x00 on UART.
// returns total bytes written (including delimiter), or 0 on error.
size_t uart_frame_build(const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_max);

// Returns internal caller format length on success, 0 on failure.
size_t uart_frame_parse(const uint8_t *cobs_data, size_t cobs_len,
                        uint8_t *payload_out, size_t payload_out_max);

#ifdef __cplusplus
}
#endif
