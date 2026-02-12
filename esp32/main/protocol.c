#include "protocol.h"

#include <string.h>

uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool parse_guidance_stream(uint8_t byte, guidance_packet_t *out_packet) {
    static uint8_t buf[sizeof(guidance_packet_t)] = {0};
    static uint16_t idx = 0;

    if (idx == 0) {
        if (byte != 0x55) {
            return false;
        }
    } else if (idx == 1) {
        if (byte != 0xAA) {
            idx = 0;
            return false;
        }
    }

    buf[idx++] = byte;
    if (idx < sizeof(guidance_packet_t)) {
        return false;
    }

    idx = 0;
    memcpy(out_packet, buf, sizeof(guidance_packet_t));

    if (out_packet->header != GUIDANCE_HEADER || out_packet->msg_type != GUIDANCE_MSG_TYPE) {
        return false;
    }

    uint16_t crc = crc16_ccitt_false((const uint8_t *)out_packet, (uint16_t)(sizeof(guidance_packet_t) - 2));
    if (crc != out_packet->crc16) {
        return false;
    }

    return true;
}

void build_status_packet(
    status_packet_t *packet,
    uint16_t seq_echo,
    uint8_t mode,
    uint8_t control_state,
    float battery_v,
    uint8_t ack_code
) {
    packet->header = STATUS_HEADER;
    packet->msg_type = STATUS_MSG_TYPE;
    packet->seq_echo = seq_echo;
    packet->mode = mode;
    packet->control_state = control_state;
    packet->battery_v = battery_v;
    packet->ack_code = ack_code;
    packet->crc16 = crc16_ccitt_false((const uint8_t *)packet, (uint16_t)(sizeof(status_packet_t) - 2));
}
