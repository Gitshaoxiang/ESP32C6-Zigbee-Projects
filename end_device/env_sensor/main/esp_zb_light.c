/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif
 * Systems integrated circuit in a product or a software update for such
 * product, must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_light.h"

#include "hal/i2c_hal.h"
#include "hal/gpio_hal.h"
#include "soc/i2c_periph.h"
#include "driver/i2c.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_ENV_SENSOR";
/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p                  = signal_struct->p_app_signal;
    esp_err_t err_status              = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                /* commissioning failed */
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)",
                         esp_err_to_name(err_status));
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG,
                         "Joined network successfully (Extended PAN ID: "
                         "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: "
                         "0x%04hx, Channel:%d)",
                         extended_pan_id[7], extended_pan_id[6],
                         extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2],
                         extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel());
            } else {
                ESP_LOGI(TAG,
                         "Network steering was not successful (status: %s)",
                         esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                     esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

// int temperature = 0;

// void report_attr_cb(esp_zb_zcl_addr_t *addr, uint8_t endpoint,
//                     uint16_t cluster_id, uint16_t attr_id,
//                     esp_zb_zcl_attr_type_t attr_type, void *value) {
//     ESP_LOGI(TAG, "Report attribute callback");
//     if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
//         if (attr_id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
//             // TODO: 读取DH11的温度

//             if (temperature > 2000)
//                 temperature += 100;
//             else
//                 temperature -= 100;
//             *(int16_t *)value = temperature;
//             /* implemented some actions when temperature changed */
//             ESP_LOGI(TAG, "Temperature changed to %d", *(int16_t *)value);
//         }
//     } else {
//         /* Implement some actions if needed when other cluster changed */
//         ESP_LOGI(TAG, "cluster:0x%x, attribute:0x%x changed ", cluster_id,
//                  attr_id);
//     }
// }

static esp_err_t zb_read_attr_resp_handler(
    const esp_zb_zcl_cmd_read_attr_resp_message_t *message) {
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(
        message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG,
        TAG, "Received message: error status(%d)", message->info.status);
    ESP_LOGI(TAG,
             "Read attribute response: status(%d), cluster(0x%x), "
             "attribute(0x%x), type(0x%x), value(%d)",
             message->info.status, message->info.cluster, message->attribute.id,
             message->attribute.data.type,
             message->attribute.data.value
                 ? *(uint8_t *)message->attribute.data.value
                 : 0);
    if (message->info.dst_endpoint == HA_ESP_TEMP_ENDPOINT) {
        switch (message->info.cluster) {
            case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:

                // ESP_LOGI(TAG, "Server time recieved %lu",
                //          *(uint32_t *)message->attribute.data.value);
                // struct timeval tv;
                // tv.tv_sec = *(uint32_t *)message->attribute.data.value +
                //             946684800 -
                //             1080;  // after adding OTA cluster time shifted
                //             to
                //                    // 1080 sec... strange issue ...
                // settimeofday(&tv, NULL);
                // time_updated = true;
                break;
            default:
                ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ",
                         message->info.cluster, message->attribute.id);
        }
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message) {
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
        // case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        //     ret = zb_attribute_handler(
        //         (esp_zb_zcl_set_attr_value_message_t *)message);
        //     break;
        case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
            ret = zb_read_attr_resp_handler(
                (esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
            break;
        default:
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ret;
}

uint16_t temperature_value;
uint16_t humidity_value;

uint16_t undefined_value;

static void esp_zb_task(void *pvParameters) {
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // esp_zb_on_off_light_cfg_t light_cfg =
    // ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG(); esp_zb_ep_list_t
    // *esp_zb_on_off_light_ep =
    // esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    // esp_zb_device_register(esp_zb_on_off_light_ep);

    // esp_zb_temperature_sensor_cfg_t temperature_sensor_cfg =
    //     ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    // esp_zb_ep_list_t *esp_zb_temperature_sensor_ep =
    //     esp_zb_temperature_sensor_ep_create(HA_ESP_TEMP_ENDPOINT,
    //                                         &temperature_sensor_cfg);

    // esp_zb_device_register(esp_zb_temperature_sensor_ep);

    // esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster =
    //     esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    // esp_zb_temperature_meas_cluster_add_attr(
    //     esp_zb_temperature_meas_cluster,
    //     ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature_value);

    // esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster =
    //     esp_zb_zcl_attr_list_create(
    //         ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);

    // esp_zb_humidity_meas_cluster_add_attr(
    //     esp_zb_humidity_meas_cluster,
    //     ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity_value);

    /* Temperature cluster */
    esp_zb_attribute_list_t *esp_zb_temperature_meas_cluster =
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(
        esp_zb_temperature_meas_cluster,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(
        esp_zb_temperature_meas_cluster,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_temperature_meas_cluster_add_attr(
        esp_zb_temperature_meas_cluster,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    /* Humidity cluster */
    esp_zb_attribute_list_t *esp_zb_humidity_meas_cluster =
        esp_zb_zcl_attr_list_create(
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(
        esp_zb_humidity_meas_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(
        esp_zb_humidity_meas_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID,
        &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(
        esp_zb_humidity_meas_cluster,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID,
        &undefined_value);

    esp_zb_cluster_list_t *esp_zb_cluster_list =
        esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_temperature_meas_cluster(
        esp_zb_cluster_list, esp_zb_temperature_meas_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(
        esp_zb_cluster_list, esp_zb_humidity_meas_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list,
                          HA_ESP_TEMP_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID,
                          ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);
    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

#define I2C_MASTER_SCL_IO 1 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 2 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                                                         \
    0 /*!< I2C master i2c port number, the number of i2c peripheral interfaces \
         available will depend on the chip */
#define I2C_MASTER_FREQ_HZ        100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS     1000

#define SHT30_ADDR 0x44

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t sht30_read_data(float *temperature, float *humidity) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x2C, true);
    i2c_master_write_byte(cmd, 0x06, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for measurement to complete

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    *temperature = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    *humidity    = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);
    return ESP_OK;
}

uint16_t temperature = 0;
uint16_t humidity    = 0;

void sht30_task(void *pvParameters) {
    float temp, hum;
    while (1) {
        esp_err_t ret = sht30_read_data(&temp, &hum);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", temp, hum);

            temperature = (uint16_t)(temp * 100);
            humidity    = (uint16_t)(hum * 100);

        } else {
            ESP_LOGE(TAG, "Failed to read data from SHT30 sensor");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    light_driver_init(LIGHT_DEFAULT_OFF);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    i2c_master_init();
    xTaskCreate(sht30_task, "sht30_task", 4096, NULL, 10, NULL);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {
        // uint8_t cmd[] = {0x2C, 0x06};

        // int data[6];

        // i2c_master_write_read_device(
        //     I2C_MASTER_NUM, SHT3X_I2C_ADDR, cmd, 2, data, 6,
        //     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

        // float cTemp    = 0;
        // float fTemp    = 0;
        // float humidity = 0;

        // cTemp    = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
        // fTemp    = (cTemp * 1.8) + 32;
        // humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

        // ESP_LOGI(TAG, "cTemp: %.2f", cTemp);
        // ESP_LOGI(TAG, "fTemp: %.2f", fTemp);
        // ESP_LOGI(TAG, "humidity: %.2f", humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(
            HA_ESP_TEMP_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, false);

        esp_zb_zcl_status_t state_hum = esp_zb_zcl_set_attribute_val(
            HA_ESP_TEMP_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity,
            false);
    }
}
