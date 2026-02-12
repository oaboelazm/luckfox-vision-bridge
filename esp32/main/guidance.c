#include "guidance.h"

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void map_guidance_to_command(const guidance_packet_t *pkt, guidance_command_t *out) {
    const float k_roll = 18.0f;
    const float k_pitch = 18.0f;
    const float k_yaw = 0.8f;
    const float k_throttle = 0.15f;

    out->target_valid = (pkt->flags & 0x01) != 0;
    out->emergency_hover = (pkt->flags & 0x02) != 0;

    if (!out->target_valid || out->emergency_hover) {
        out->roll_setpoint = 0.0f;
        out->pitch_setpoint = 0.0f;
        out->yaw_rate_setpoint = 0.0f;
        out->throttle_adjust = 0.0f;
        return;
    }

    out->roll_setpoint = clampf(pkt->dx_norm * k_roll, -20.0f, 20.0f);
    out->pitch_setpoint = clampf(-pkt->dy_norm * k_pitch, -20.0f, 20.0f);
    out->yaw_rate_setpoint = clampf(pkt->yaw_err_deg * k_yaw, -60.0f, 60.0f);

    float z_error = pkt->z_est_m - 1.2f;
    out->throttle_adjust = clampf(-z_error * k_throttle, -0.25f, 0.25f);
}
