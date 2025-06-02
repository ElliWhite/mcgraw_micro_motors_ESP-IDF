#ifndef _CHIP_UTILS_H_
#define _CHIP_UTILS_H_



#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"
#include "esp_log.h"

static const char *TAG_INFO = "info";
static const char *TAG_UART_TX = "uart_tx";
static const char *TAG_UART_RX = "uart_rx";
static const char *TAG_LVGL = "lvgl";
static const char *TAG_I2C = "i2c";
static const char *TAG_MOTOR = "bdc_motor";
static const char *TAG_PID = "pid";
static const char *TAG_PCNT = "pcnt";

extern bool g_enable_info_logs;     // global flag so should only be defined in one place as this header file is included in multiple places
extern bool g_enable_uart_tx_logs;  // global flag
extern bool g_enable_uart_rx_logs;  // global flag
extern bool g_enable_i2c_logs;      // global flag
extern bool g_enable_lvgl_logs;     // global flag
extern bool g_enable_motor_logs;    // global flag
extern bool g_enable_pid_logs;    // global flag
extern bool g_enable_pcnt_logs;    // global flag

// Conditional logging based on flags above
#define COND_ESP_LOGI(tag, fmt, ...)   \
    do {                                                           \
        if (                                                       \
            (strcmp(tag, TAG_INFO) == 0   && g_enable_info_logs)   ||           \
            (strcmp(tag, TAG_UART_TX) == 0  && g_enable_uart_tx_logs)  ||       \
            (strcmp(tag, TAG_I2C) == 0  && g_enable_i2c_logs)  ||               \
            (strcmp(tag, TAG_LVGL) == 0  && g_enable_lvgl_logs)  ||             \
            (strcmp(tag, TAG_MOTOR) == 0  && g_enable_motor_logs)  ||           \
            (strcmp(tag, TAG_PID) == 0  && g_enable_pid_logs)  ||           \
            (strcmp(tag, TAG_PCNT) == 0  && g_enable_pcnt_logs)  ||           \
            (strcmp(tag, TAG_UART_RX) == 0 && g_enable_uart_rx_logs)) {        \
            ESP_LOGI(tag, fmt, ##__VA_ARGS__);                     \
        }                                                          \
    } while (0)

void printESPInfo();
void restartESP(uint16_t seconds);

#ifdef __cplusplus
}
#endif 
#endif // _CHIP_UTILS_H_