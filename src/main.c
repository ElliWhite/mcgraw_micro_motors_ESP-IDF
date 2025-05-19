#include "stdio.h"
#include "stdint.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "chip_utils.h"
#include "motor_control.h"

#define PIN_LED_BUILTIN 2

static const char *TAG_INFO = "info";


void app_main(void)
{
    printf("Hello world!\n");

    //printESPInfo();

    static motor_control_context_t motor_ctrl_ctx_left = {
        .pcnt_encoder = NULL,
    };

    static motor_control_context_t motor_ctrl_ctx_right = {
        .pcnt_encoder = NULL,
    };

    bdc_motor_handle_t motor_left = NULL;
    bdc_motor_handle_t motor_right = NULL;
    bdc_motor_handle_t motor_left_configured = NULL;
    bdc_motor_handle_t motor_right_configured = NULL;
    ESP_LOGI(TAG_MOTOR, "Setting up motor controllers");
    ESP_ERROR_CHECK(setupMotorControllers(motor_left, motor_right, &motor_left_configured, &motor_right_configured));      // Returned configured motors

    motor_ctrl_ctx_left.motor = motor_left_configured;
    motor_ctrl_ctx_right.motor = motor_right_configured;
    ESP_LOGI(TAG_PCNT, "Setting up pulse counters");
    setupPCNT(&motor_ctrl_ctx_left, &motor_ctrl_ctx_right);


    const esp_timer_create_args_t periodic_timer_args_left = {
        .callback = pid_loop_cb_left,
        .arg = &motor_ctrl_ctx_left,
        .name = "pid_loop_left"
    };

    const esp_timer_create_args_t periodic_timer_args_right = {
        .callback = pid_loop_cb_right,
        .arg = &motor_ctrl_ctx_right,
        .name = "pid_loop_right"
    };

    ESP_LOGI(TAG_PID, "Setting up PID loops");
    setupPIDLoops(&motor_ctrl_ctx_left, &motor_ctrl_ctx_right, periodic_timer_args_left, periodic_timer_args_right);

    ESP_LOGI(TAG_INFO, "ESP initialised motor, PCNT & PID okay");

    
    //BDC_PID_EXPECT_SPEED_LEFT = 0.0;
    //BDC_PID_EXPECT_SPEED_RIGHT = 0.0;
    BDC_PID_EXPECT_SPEED_LEFT_MS = 0.0;
    BDC_PID_EXPECT_SPEED_RIGHT_MS = 0.0;
    

    while(1){
        vTaskDelay(pdMS_TO_TICKS(2500));        // 2.5 second pause
       
        if(BDC_PID_EXPECT_SPEED_LEFT_MS == 0.0){
            BDC_PID_EXPECT_SPEED_LEFT_MS = 0.1;
        }else{
            BDC_PID_EXPECT_SPEED_LEFT_MS = 0.0;
        }
        if(BDC_PID_EXPECT_SPEED_RIGHT_MS == 0.0){
            BDC_PID_EXPECT_SPEED_RIGHT_MS = 0.1;
        }else{
            BDC_PID_EXPECT_SPEED_RIGHT_MS = 0.0;
        }

        ESP_LOGI(TAG_MOTOR, "Setting left speed to %3.1f m/s", BDC_PID_EXPECT_SPEED_LEFT_MS);
        ESP_LOGI(TAG_MOTOR, "Setting right speed to %3.1f m/s", BDC_PID_EXPECT_SPEED_RIGHT_MS);
        
    }



    // Reset the pin
    gpio_reset_pin(PIN_LED_BUILTIN);

    // Set input/output so state can be read with get_level
    gpio_set_direction(PIN_LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    for(int i = 0; i < 10; i++) {
        // Toggle the pin (set to the opposite level)
        gpio_set_level(PIN_LED_BUILTIN, (gpio_get_level(PIN_LED_BUILTIN) == 0) ? 1 : 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    restartESP(2);
    
}

