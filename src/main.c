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





#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"


#define PIN_LED_BUILTIN 2

// Custom UUIDs
#define JOYSTICK_SERVICE_UUID               0xFFF0
#define JOYSTICK_CHARACTERISTIC_WRITE_UUID  0xFFF1
#define JOYSTICK_CHARACTERISTIC_READ_UUID   0xFFF2

static const char *TAG = "BLE_JOYSTICK";
static uint8_t ble_addr_type;
static uint8_t joystick_data[4] = {128, 128, 0, 0};  // X=128, Y=128 (centered), no buttons pressed

static int joystick_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    char buf[64] = {0};
    int len = min(ctxt->om->om_len, sizeof(buf) - 1);
    memcpy(buf, ctxt->om->om_data, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "Joystick data: %s", buf);

    // TODO: Parse X/Y values and drive motors here

    return 0;
}

static int read_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // Provide data to the client when it reads this characteristic
    // For example, return a predefined value
    static const char response[] = "Hello from ESP32!";
    os_mbuf_append(ctxt->om, response, sizeof(response));
    return 0;
}

static int joystick_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return os_mbuf_append(ctxt->om, joystick_data, sizeof(joystick_data));
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_chr_def joystick_chr[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0xFFE1),
        .access_cb = joystick_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {0}
};

static const struct ble_gatt_svc_def joystick_svc[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFE0),
        .characteristics = joystick_chr,
    },
    {0}
};

static void ble_app_on_reset(int reason) {
    ESP_LOGI("BLE", "Reset callback, reason: %d", reason);
}

static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = "ESP32-JOYSTICK";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE("BLE_ADV", "Failed to set advertisement data; rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE("BLE_ADV", "Failed to start advertising; rc=%d", rc);
    }
}

static void
bleprph_on_sync(void)
{
    int rc;

#if CONFIG_EXAMPLE_RANDOM_ADDR
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
#endif

    /* Make sure we have proper identity address set (public preferred) */
#if CONFIG_EXAMPLE_RANDOM_ADDR
    rc = ble_hs_util_ensure_addr(1);
#else
    rc = ble_hs_util_ensure_addr(0);
#endif
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(ble_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    ext_bleprph_advertise();
#else
    ble_app_advertise();
#endif
}

void ble_host_task(void *param) {
    nimble_port_run();                  // This runs the BLE host task
    nimble_port_freertos_deinit();     // Clean up if it ever exits
    vTaskDelete(NULL);                 // Delete this task
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(JOYSTICK_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = BLE_UUID16_DECLARE(JOYSTICK_CHARACTERISTIC_WRITE_UUID),
            .access_cb = joystick_write_cb,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        },
        {
            .uuid = BLE_UUID16_DECLARE(JOYSTICK_CHARACTERISTIC_READ_UUID), // Read characteristic
            .access_cb = read_access_cb,
            .flags = BLE_GATT_CHR_F_READ,
        },
        {0}},
    },
    {0}
};



static void ble_app_on_sync_1(void) {
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE("BLE_SYNC", "Failed to infer address type; rc=%d", rc);
    }
    ble_app_advertise();
}

void ble_app_init(void) {

    ble_hs_cfg.sync_cb = bleprph_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_gatts_count_cfg(joystick_svc);
    ble_gatts_add_svcs(joystick_svc);

}

void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void ble_store_config_init(void);

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

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or incompatible version — erase and retry
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Disable bluetooth is already running 
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bt_controller_disable());
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        ESP_ERROR_CHECK(esp_bt_controller_deinit());
    }

    /* BLUETOOTH */

    // Start NimBLE stack
    nimble_port_init();
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    /* Set the default device name. */
    int rc = ble_svc_gap_device_name_set("nimble-bleprph");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    nimble_port_freertos_init(ble_host_task);
/*
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err) {
        ESP_LOGE("BLE_INIT", "Controller init failed: %d", err);
        return;
    }
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_nimble_hci_init());
*/  
    


    //ble_app_init();



    

    /* MOTOR CONTROL */
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

