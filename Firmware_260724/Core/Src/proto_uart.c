#include "proto_uart.h"
#include <string.h>

#define UART_PKT_SOM  0x02u
#define UART_PKT_EOM  0x03u

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Standard COBS
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
    if (!in || !out || out_max == 0) return 0;

    size_t read_index = 0, write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < len) {
        if (write_index >= out_max) return 0;

        if (in[read_index] == 0) {
            out[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            out[write_index++] = in[read_index++];
            code++;
            if (code == 0xFF) {
                out[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    if (code_index >= out_max) return 0;
    out[code_index] = code;

    return write_index;
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
    if (!in || !out) return 0;
    size_t read_index = 0, write_index = 0;

    while (read_index < len) {
        uint8_t code = in[read_index];
        if (code == 0) return 0;
        read_index++;

        for (uint8_t i = 1; i < code; i++) {
            if (read_index >= len) return 0;
            if (write_index >= out_max) return 0;
            out[write_index++] = in[read_index++];
        }
        if (code != 0xFF && read_index < len) {
            if (write_index >= out_max) return 0;
            out[write_index++] = 0;
        }
    }
    return write_index;
}

size_t uart_frame_build(const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_max) {
    if (!payload || !out) return 0;
    if (payload_len < 4) return 0;

    uint8_t tmp[512];
    uint16_t app_len = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    if ((size_t)(app_len + 4u) != payload_len) return 0;

    uint16_t wire_len = (uint16_t)(app_len + 2u); // msg_id + seq + app payload
    size_t tmp_len = (size_t)wire_len + 6u;       // som + len(2) + body + crc(2) + eom
    if (tmp_len > sizeof(tmp)) return 0;

    tmp[0] = UART_PKT_SOM;
    tmp[1] = (uint8_t)(wire_len & 0xFF);
    tmp[2] = (uint8_t)((wire_len >> 8) & 0xFF);
    tmp[3] = payload[0]; // msg type -> Message ID
    tmp[4] = payload[1]; // keep seq in payload data section
    if (app_len > 0u) {
        memcpy(&tmp[5], &payload[4], app_len);
    }

    uint16_t crc = crc16_ccitt_false(tmp, (size_t)wire_len + 3u);
    tmp[5u + app_len] = (uint8_t)(crc & 0xFF);
    tmp[6u + app_len] = (uint8_t)((crc >> 8) & 0xFF);
    tmp[7u + app_len] = UART_PKT_EOM;

    size_t enc_len = cobs_encode(tmp, tmp_len, out, out_max);
    if (enc_len == 0) return 0;
    if (enc_len + 1 > out_max) return 0;
    out[enc_len] = 0x00; // delimiter
    return enc_len + 1;
}

size_t uart_frame_parse(const uint8_t *cobs_data, size_t cobs_len,
                        uint8_t *payload_out, size_t payload_out_max) {
    if (!cobs_data || !payload_out) return 0;

    uint8_t tmp[512];
    if (cobs_len > sizeof(tmp)) return 0;

    size_t dec_len = cobs_decode(cobs_data, cobs_len, tmp, sizeof(tmp));
    if (dec_len < 8u) return 0;
    if (tmp[0] != UART_PKT_SOM) return 0;
    if (tmp[dec_len - 1u] != UART_PKT_EOM) return 0;

    uint16_t wire_len = (uint16_t)tmp[1] | ((uint16_t)tmp[2] << 8);
    size_t expected_len = (size_t)wire_len + 6u;
    if (expected_len != dec_len) return 0;
    if (wire_len < 2u) return 0;

    uint16_t crc_rx = (uint16_t)tmp[dec_len - 3u] | ((uint16_t)tmp[dec_len - 2u] << 8);
    uint16_t crc_calc = crc16_ccitt_false(tmp, (size_t)wire_len + 3u);
    if (crc_rx != crc_calc) return 0;

    uint16_t app_len = (uint16_t)(wire_len - 2u);
    size_t internal_len = (size_t)app_len + 4u;
    if (internal_len > payload_out_max) return 0;

    payload_out[0] = tmp[3]; // msg type
    payload_out[1] = tmp[4]; // seq
    payload_out[2] = (uint8_t)(app_len & 0xFF);
    payload_out[3] = (uint8_t)((app_len >> 8) & 0xFF);
    if (app_len > 0u) {
        memcpy(&payload_out[4], &tmp[5], app_len);
    }
    return internal_len;
}
