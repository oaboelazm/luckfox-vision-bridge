#pragma once

#include <stdbool.h>

#include "protocol.h"

typedef struct {
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_rate_setpoint;
    float throttle_adjust;
    bool target_valid;
    bool emergency_hover;
} guidance_command_t;

void map_guidance_to_command(const guidance_packet_t *pkt, guidance_command_t *out);
