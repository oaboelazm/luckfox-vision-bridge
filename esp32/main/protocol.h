#pragma once

#include <stdbool.h>
#include <stdint.h>

#define GUIDANCE_HEADER 0xAA55
#define STATUS_HEADER 0x55AA
#define GUIDANCE_MSG_TYPE 0x01
#define STATUS_MSG_TYPE 0x81

#pragma pack(push, 1)
typedef struct {
    uint16_t header;
    uint8_t msg_type;
    uint16_t seq;
    uint32_t timestamp_ms;
    uint8_t target_type;
    float dx_norm;
    float dy_norm;
    float z_est_m;
    float yaw_err_deg;
    float confidence;
    uint8_t flags;
    uint16_t crc16;
} guidance_packet_t;

typedef struct {
    uint16_t header;
    uint8_t msg_type;
    uint16_t seq_echo;
    uint8_t mode;
    uint8_t control_state;
    float battery_v;
    uint8_t ack_code;
    uint16_t crc16;
} status_packet_t;
#pragma pack(pop)

uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len);
bool parse_guidance_stream(uint8_t byte, guidance_packet_t *out_packet);
void build_status_packet(
    status_packet_t *packet,
    uint16_t seq_echo,
    uint8_t mode,
    uint8_t control_state,
    float battery_v,
    uint8_t ack_code
);
