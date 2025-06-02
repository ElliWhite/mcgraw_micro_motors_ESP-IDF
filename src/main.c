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


/* Calculate wheel target velocities given the robot's target linear velocity (m/s) and angular velocity (rads/s) */
esp_err_t calculateWheelVelocities(float robot_target_linear_velocity, float robot_target_angular_velocity, float *bdc_pid_target_velocity_left_ms, float *bdc_pid_target_velocity_right_ms) {

    *bdc_pid_target_velocity_left_ms = robot_target_linear_velocity - ((robot_target_angular_velocity * WHEEL_BASE_M) / 2);
    *bdc_pid_target_velocity_right_ms = robot_target_linear_velocity + ((robot_target_angular_velocity * WHEEL_BASE_M) / 2);
    
    return ESP_OK;

}

/* Configure motor direction based on robot target linear/angular velocities */
/* atm, motor direction is based on motors sat on the bench and not in the robot, so the motor going on the right-hand side will need its directions inversed. 
This could be done in motor_control.h by swapping BDC_MCPWM_RIGHT_GPIO_A/B pin numbers round so the motor directions below make sense  */
esp_err_t configureMotorDirection(bdc_motor_handle_t motor_left, bdc_motor_handle_t motor_right, float robot_target_linear_velocity, float robot_target_angular_velocity){

    // robot stationary
    if(robot_target_linear_velocity == 0.0) {
        // turning left
        if(robot_target_angular_velocity > 0) {
            COND_ESP_LOGI(TAG_MOTOR, "Motor left backwards - Motor right forwards");
            ESP_ERROR_CHECK(bdc_motor_reverse(motor_left));
            ESP_ERROR_CHECK(bdc_motor_forward(motor_right));
        } else {    // turning right
            COND_ESP_LOGI(TAG_MOTOR, "Motor left forwards - Motor right backwards");
            ESP_ERROR_CHECK(bdc_motor_forward(motor_left));
            ESP_ERROR_CHECK(bdc_motor_reverse(motor_right));
        }
    } else if (robot_target_linear_velocity > 0.0) {     // forwards
        COND_ESP_LOGI(TAG_MOTOR, "Motors forwards");
        ESP_ERROR_CHECK(bdc_motor_forward(motor_left));
        ESP_ERROR_CHECK(bdc_motor_forward(motor_right));
    } else if (robot_target_linear_velocity < 0.0) {    // backwards
        COND_ESP_LOGI(TAG_MOTOR, "Motors backwards");
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_left));
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_right));
    }

    return ESP_OK;

}

void app_main(void)
{
    
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
    COND_ESP_LOGI(TAG_MOTOR, "Setting up motor controllers");
    ESP_ERROR_CHECK(setupMotorControllers(motor_left, motor_right, &motor_left_configured, &motor_right_configured));      // Returned configured motors

    motor_ctrl_ctx_left.motor = motor_left_configured;
    motor_ctrl_ctx_right.motor = motor_right_configured;
    COND_ESP_LOGI(TAG_PCNT, "Setting up pulse counters");
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

    COND_ESP_LOGI(TAG_PID, "Setting up PID loops");
    setupPIDLoops(&motor_ctrl_ctx_left, &motor_ctrl_ctx_right, periodic_timer_args_left, periodic_timer_args_right);

    COND_ESP_LOGI(TAG_INFO, "ESP initialised motor, PCNT & PID okay");

    
    BDC_PID_TARGET_SPEED_LEFT_MS = 0.0;
    BDC_PID_TARGET_SPEED_RIGHT_MS = 0.0;

    float robot_target_linear_velocity = 0.2;   // m/s
    float robot_target_angular_velocity = 0.0;  // rads/s

    while(1){
        vTaskDelay(pdMS_TO_TICKS(3500));        // 3.5 second pause

        // Forwards, no turning
        robot_target_linear_velocity = 0.2;
        robot_target_angular_velocity = 0.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);
        //COND_ESP_LOGI(TAG_MOTOR, "Setting left speed to %3.1f m/s. ", BDC_PID_TARGET_SPEED_LEFT_MS);
        //COND_ESP_LOGI(TAG_MOTOR, "Setting right speed to %3.1f m/s", BDC_PID_TARGET_SPEED_RIGHT_MS);

        vTaskDelay(pdMS_TO_TICKS(3500));        // 3.5 second pause

        // Stationary, turning
        robot_target_linear_velocity = 0.0;
        robot_target_angular_velocity = 3.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);
        
        // Stationary, turning
        robot_target_linear_velocity = 0.0;
        robot_target_angular_velocity = -3.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);
        
        vTaskDelay(pdMS_TO_TICKS(3500));        // 3.5 second pause

        // Forwards, turning
        robot_target_linear_velocity = 0.3;
        robot_target_angular_velocity = 2.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);

        // Forwards, turning
        robot_target_linear_velocity = 0.3;
        robot_target_angular_velocity = -6.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);

        vTaskDelay(pdMS_TO_TICKS(3500));        // 3.5 second pause

        // Backwards, no turning
        robot_target_linear_velocity = -0.2;
        robot_target_angular_velocity = 0.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);

        vTaskDelay(pdMS_TO_TICKS(3500));        // 3.5 second pause

        // Backwards, turning
        robot_target_linear_velocity = -0.3;
        robot_target_angular_velocity = -2.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);
        
        // Backwards, turning
        robot_target_linear_velocity = -0.3;
        robot_target_angular_velocity = 6.0;
        configureMotorDirection(motor_left_configured, motor_right_configured, robot_target_linear_velocity, robot_target_angular_velocity);
        calculateWheelVelocities(robot_target_linear_velocity, robot_target_angular_velocity, &BDC_PID_TARGET_SPEED_LEFT_MS, &BDC_PID_TARGET_SPEED_RIGHT_MS);
       
        COND_ESP_LOGI(TAG_MOTOR, "Target LV: %3.1fm/s. Target AV: %3.1frads/s. Target LMV: %6.3fm/s. Target RMV: %6.3fm/s", robot_target_linear_velocity, robot_target_angular_velocity, BDC_PID_TARGET_SPEED_LEFT_MS, BDC_PID_TARGET_SPEED_RIGHT_MS);
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

