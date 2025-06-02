#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ               2500    // 2.5KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX         (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_LEFT_GPIO_A           22
#define BDC_MCPWM_LEFT_GPIO_B           23
#define BDC_MCPWM_RIGHT_GPIO_A          19
#define BDC_MCPWM_RIGHT_GPIO_B          21

#define BDC_ENCODER_LEFT_GPIO_A         4
#define BDC_ENCODER_LEFT_GPIO_B         16
#define BDC_ENCODER_RIGHT_GPIO_A        5
#define BDC_ENCODER_RIGHT_GPIO_B        18
#define BDC_ENCODER_PCNT_HIGH_LIMIT     2940
#define BDC_ENCODER_PCNT_LOW_LIMIT      -2940

#define BDC_PID_LOOP_PERIOD_MS          10   // calculate the motor speed every 10ms


extern float BDC_PID_TARGET_SPEED_LEFT_MS;     // target motor speed, in m/s
extern float BDC_PID_TARGET_SPEED_RIGHT_MS;     // target motor speed, in m/s

#define WHEEL_RADIUS                    0.02    // m
#define WHEEL_CIRCUMFERENCE             (2 * M_PI * WHEEL_RADIUS);
#define WHEEL_BASE_M                    0.1     // distance between wheels in m


typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

extern esp_timer_handle_t pid_loop_timer_left;      // Used in setup PID loops (esp_timer_create())
extern esp_timer_handle_t pid_loop_timer_right;




esp_err_t setupMotorControllers(bdc_motor_handle_t motor_left, bdc_motor_handle_t motor_right, bdc_motor_handle_t *ret_motor_left, bdc_motor_handle_t *ret_motor_right);
void pid_loop_cb_left(void *args);
void pid_loop_cb_right(void *args);
esp_err_t setupPCNT(motor_control_context_t *motor_ctrl_ctx_left, motor_control_context_t *motor_ctrl_ctx_right);
esp_err_t setupPIDLoops(motor_control_context_t *motor_ctrl_ctx_left, motor_control_context_t *motor_ctrl_ctx_right, esp_timer_create_args_t periodic_timer_args_left, esp_timer_create_args_t periodic_timer_args_right);

#ifdef __cplusplus
}
#endif 
#endif // _MOTOR_CONTROL_H_