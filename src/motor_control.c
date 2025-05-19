#include <stdio.h>
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "motor_control.h"

esp_timer_handle_t pid_loop_timer_left;      
esp_timer_handle_t pid_loop_timer_right;

float BDC_PID_EXPECT_SPEED_LEFT;
float BDC_PID_EXPECT_SPEED_RIGHT;
float BDC_PID_EXPECT_SPEED_LEFT_MS;     // expected motor speed, in m/s
float BDC_PID_EXPECT_SPEED_RIGHT_MS;     // expected motor speed, in m/s


// Takes blank motor configs, then updates them and returns pointers to the configs
esp_err_t setupMotorControllers(bdc_motor_handle_t motor_left, bdc_motor_handle_t motor_right, bdc_motor_handle_t *ret_motor_left, bdc_motor_handle_t *ret_motor_right){
    
    // Create new motor config structs
    bdc_motor_config_t motor_config_left = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_LEFT_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_LEFT_GPIO_B,
    };
    bdc_motor_config_t motor_config_right = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_RIGHT_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_RIGHT_GPIO_B,
    };
    // Create new MCPWM config struct
    bdc_motor_mcpwm_config_t mcpwm_config_left = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_config_right = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    //bdc_motor_handle_t motor_left = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_left, &mcpwm_config_left, &motor_left));      // If this fails, the ESP will reset. Returns configured motors
    //bdc_motor_handle_t motor_right = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_right, &mcpwm_config_right, &motor_right));      // If this fails, the ESP will reset. Returns configured motors
    ESP_LOGI(TAG_MOTOR, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_left));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_right));
    ESP_LOGI(TAG_MOTOR, "Forward motors");
    ESP_ERROR_CHECK(bdc_motor_forward(motor_left));
    ESP_ERROR_CHECK(bdc_motor_forward(motor_right));
    *ret_motor_left = motor_left;
    *ret_motor_right = motor_right;
    
    return ESP_OK;
}


void pid_loop_cb_left(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    float rpm = ((real_pulses/(float)BDC_ENCODER_PCNT_HIGH_LIMIT) / ((float)BDC_PID_LOOP_PERIOD_MS/1000)) * 60;
    float wheel_speed = (rpm / 60) * WHEEL_CIRCUMFERENCE;

    // calculate the speed error
    //float error = BDC_PID_EXPECT_SPEED_LEFT - real_pulses;
    float error = BDC_PID_EXPECT_SPEED_LEFT_MS - wheel_speed;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    //ESP_LOGI(TAG_PID, "Setting to speed of %f", new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

void pid_loop_cb_right(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    float rpm = ((real_pulses/(float)BDC_ENCODER_PCNT_HIGH_LIMIT) / ((float)BDC_PID_LOOP_PERIOD_MS/1000)) * 60;
    float wheel_speed = (rpm / 60) * WHEEL_CIRCUMFERENCE;

    // calculate the speed error
    //float error = BDC_PID_EXPECT_SPEED_LEFT - real_pulses;
    float error = BDC_PID_EXPECT_SPEED_RIGHT_MS - wheel_speed;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    //ESP_LOGI(TAG_PID, "Setting to speed of %f", new_speed);
    //ESP_LOGI(TAG_PID, "rpm %f, wheel_speed %f, real_pulses %i, error %f, new_speed %f", rpm, wheel_speed, real_pulses, error, new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}


esp_err_t setupPCNT(motor_control_context_t *motor_ctrl_ctx_left, motor_control_context_t *motor_ctrl_ctx_right){
    ESP_LOGI(TAG_PCNT, "Init PCNT driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit_left = NULL;
    pcnt_unit_handle_t pcnt_unit_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_right));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_left, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_right, &filter_config));

    pcnt_chan_config_t chan_a_config_left = {
        .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
        .level_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
    };
    pcnt_chan_config_t chan_a_config_right = {
        .edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
        .level_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a_left = NULL;
    pcnt_channel_handle_t pcnt_chan_a_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_a_config_left, &pcnt_chan_a_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_a_config_right, &pcnt_chan_a_right));

    pcnt_chan_config_t chan_b_config_left = {
        .edge_gpio_num = BDC_ENCODER_LEFT_GPIO_B,
        .level_gpio_num = BDC_ENCODER_LEFT_GPIO_A,
    };
    pcnt_chan_config_t chan_b_config_right = {
        .edge_gpio_num = BDC_ENCODER_RIGHT_GPIO_B,
        .level_gpio_num = BDC_ENCODER_RIGHT_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b_left = NULL;
    pcnt_channel_handle_t pcnt_chan_b_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_b_config_left, &pcnt_chan_b_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_b_config_right, &pcnt_chan_b_right));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_left, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_left, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_right, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_right, BDC_ENCODER_PCNT_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));
    motor_ctrl_ctx_left->pcnt_encoder = pcnt_unit_left;

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));
    motor_ctrl_ctx_right->pcnt_encoder = pcnt_unit_right;
    
    return ESP_OK;
}

esp_err_t setupPIDLoops(motor_control_context_t *motor_ctrl_ctx_left, motor_control_context_t *motor_ctrl_ctx_right, esp_timer_create_args_t periodic_timer_args_left, esp_timer_create_args_t periodic_timer_args_right){
    ESP_LOGI(TAG_PID, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 100,   //0.6, (old values for using pulses/BDC_PID_LOOP_PERIOD_MS)
        .ki = 80,   //0.4,
        .kd = 40,   //0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl_left = NULL;
    pid_ctrl_config_t pid_config_left = {
        .init_param = pid_runtime_param,
    };
    pid_ctrl_block_handle_t pid_ctrl_right = NULL;
    pid_ctrl_config_t pid_config_right = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config_left, &pid_ctrl_left));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config_right, &pid_ctrl_right));
    motor_ctrl_ctx_left->pid_ctrl = pid_ctrl_left;
    motor_ctrl_ctx_right->pid_ctrl = pid_ctrl_right;

 
    pid_loop_timer_left = NULL;
    pid_loop_timer_right = NULL;
    ESP_LOGI(TAG_PID, "Create a timer to do PID calculation periodically - left motor");
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_left, &pid_loop_timer_left));
    ESP_LOGI(TAG_PID, "Create a timer to do PID calculation periodically - right motor");
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_right, &pid_loop_timer_right));
    ESP_LOGI(TAG_PID, "Start motor speed loops");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer_left, BDC_PID_LOOP_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer_right, BDC_PID_LOOP_PERIOD_MS * 1000));

    return ESP_OK;
}

