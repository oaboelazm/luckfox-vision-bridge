#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "guidance.h"
#include "protocol.h"

#define UART_PORT UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD 921600
#define RX_BUF_SIZE 1024
#define STATUS_PERIOD_MS 50
#define GUIDANCE_TIMEOUT_MS 200

static const char *TAG = "vision_bridge";

typedef struct {
    guidance_packet_t pkt;
    int64_t last_update_ms;
    bool has_data;
} shared_guidance_t;

static shared_guidance_t g_shared = {0};
static SemaphoreHandle_t g_lock;

static int64_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

static void uart_task(void *arg) {
    (void)arg;
    uint8_t byte = 0;
    int64_t last_status = 0;

    while (1) {
        int n = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(5));
        if (n == 1) {
            guidance_packet_t pkt;
            if (parse_guidance_stream(byte, &pkt)) {
                xSemaphoreTake(g_lock, portMAX_DELAY);
                g_shared.pkt = pkt;
                g_shared.last_update_ms = now_ms();
                g_shared.has_data = true;
                xSemaphoreGive(g_lock);
            }
        }

        int64_t t = now_ms();
        if ((t - last_status) >= STATUS_PERIOD_MS) {
            guidance_packet_t last_pkt = {0};
            bool has_data = false;

            xSemaphoreTake(g_lock, portMAX_DELAY);
            has_data = g_shared.has_data;
            if (has_data) {
                last_pkt = g_shared.pkt;
            }
            xSemaphoreGive(g_lock);

            status_packet_t status = {0};
            uint16_t seq_echo = has_data ? last_pkt.seq : 0;
            uint8_t mode = 1;
            uint8_t control_state = 1;
            float battery_v = 12.0f;
            uint8_t ack_code = has_data ? 0 : 1;

            build_status_packet(&status, seq_echo, mode, control_state, battery_v, ack_code);
            uart_write_bytes(UART_PORT, (const char *)&status, sizeof(status));
            last_status = t;
        }
    }
}

static void control_task(void *arg) {
    (void)arg;
    guidance_command_t cmd = {0};

    while (1) {
        guidance_packet_t pkt = {0};
        int64_t last_update = 0;
        bool has_data = false;

        xSemaphoreTake(g_lock, portMAX_DELAY);
        has_data = g_shared.has_data;
        if (has_data) {
            pkt = g_shared.pkt;
            last_update = g_shared.last_update_ms;
        }
        xSemaphoreGive(g_lock);

        bool timed_out = !has_data || ((now_ms() - last_update) > GUIDANCE_TIMEOUT_MS);
        if (timed_out) {
            cmd.roll_setpoint = 0.0f;
            cmd.pitch_setpoint = 0.0f;
            cmd.yaw_rate_setpoint = 0.0f;
            cmd.throttle_adjust = 0.0f;
            cmd.target_valid = false;
            cmd.emergency_hover = true;
        } else {
            map_guidance_to_command(&pkt, &cmd);
        }

        // TODO: Integrate these setpoints with your PID + motor mixer.
        ESP_LOGD(
            TAG,
            "cmd valid=%d hover=%d roll=%.2f pitch=%.2f yaw=%.2f thr=%.2f",
            cmd.target_valid,
            cmd.emergency_hover,
            cmd.roll_setpoint,
            cmd.pitch_setpoint,
            cmd.yaw_rate_setpoint,
            cmd.throttle_adjust
        );

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void) {
    g_lock = xSemaphoreCreateMutex();

    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, RX_BUF_SIZE, RX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 9, NULL);
}
