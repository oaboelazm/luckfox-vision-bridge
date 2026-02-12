# UART Protocol

## Guidance packet (Luckfox -> ESP32)

Little-endian packed bytes:

- `header` `uint16` = `0xAA55`
- `msg_type` `uint8` = `0x01`
- `seq` `uint16`
- `timestamp_ms` `uint32`
- `target_type` `uint8` (`0=none,1=apriltag,2=plant`)
- `dx_norm` `float32`
- `dy_norm` `float32`
- `z_est_m` `float32`
- `yaw_err_deg` `float32`
- `confidence` `float32`
- `flags` `uint8` (`bit0=target_valid`, `bit1=emergency_hover`)
- `crc16` `uint16` CRC-CCITT-FALSE over all preceding bytes

Format: `<H B H I B f f f f f B H`

## Status packet (ESP32 -> Luckfox)

- `header` `uint16` = `0x55AA`
- `msg_type` `uint8` = `0x81`
- `seq_echo` `uint16`
- `mode` `uint8`
- `control_state` `uint8`
- `battery_v` `float32`
- `ack_code` `uint8`
- `crc16` `uint16`

Format: `<H B H B B f B H`

## Timeout policy

- ESP32 enters hover/safe if no valid guidance for `>200ms`.
- Luckfox reopens UART session if no status for `>500ms`.
