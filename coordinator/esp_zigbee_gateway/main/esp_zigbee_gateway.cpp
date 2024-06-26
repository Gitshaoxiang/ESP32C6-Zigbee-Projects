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

// #define LGFX_AUTODETECT  // 自動認識 (D-duino-32 XS, WT32-SC01, PyBadge
//                          //
//                          はパネルID読取りが出来ないため自動認識の対象から外れています)

// //
// 複数機種の定義を行うか、LGFX_AUTODETECTを定義することで、実行時にボードを自動認識します。

// // ヘッダをincludeします。

// #define LGFX_AUTODETECT  // 自動認識 (D-duino-32 XS, WT32-SC01, PyBadge
// はパネルID読取りが出来ないため自動認識の対象から外れています)

#define LGFX_M5STACK  // M5Stack M5Stack Basic / Gray / Go / Fire

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>  // クラス"LGFX"を準備します
#include "gw_title.h"
#include "temp_icon.h"
#include "btn.h"

static LGFX lcd;  // LGFXのインスタンスを作成。
static LGFX_Sprite sprite(
    &lcd);  // スプライトを使う場合はLGFX_Spriteのインスタンスを作成。

#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_vfs_eventfd.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_gateway.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "switch_driver.h"

#if defined ZB_ED_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile light switch source code.
#endif
typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};

static const char *TAG = "ESP_ZB_ON_OFF_SWITCH";

static void esp_zb_buttons_handler(switch_func_pair_t *button_func_pair) {
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* implemented light switch toggle functionality */
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode  = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
        ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command");
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bound successfully!");
        if (user_ctx) {
            light_bulb_device_params_t *light =
                (light_bulb_device_params_t *)user_ctx;
            ESP_LOGI(TAG,
                     "The light originating from address(0x%x) on endpoint(%d)",
                     light->short_addr, light->endpoint);
            free(light);
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr,
                         uint8_t endpoint, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Found light");
        esp_zb_zdo_bind_req_param_t bind_req;
        light_bulb_device_params_t *light =
            (light_bulb_device_params_t *)malloc(
                sizeof(light_bulb_device_params_t));
        light->endpoint   = endpoint;
        light->short_addr = addr;
        esp_zb_ieee_address_by_short(light->short_addr, light->ieee_addr);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp      = HA_ONOFF_SWITCH_ENDPOINT;
        bind_req.cluster_id    = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, light->ieee_addr,
               sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp     = endpoint;
        bind_req.req_dst_addr = esp_zb_get_short_address();
        ESP_LOGI(TAG, "Try to bind On/Off");
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)light);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p                  = signal_struct->p_app_signal;
    esp_err_t err_status              = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)",
                         esp_err_to_name(err_status));
            }
            break;
        case ESP_ZB_BDB_SIGNAL_FORMATION:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG,
                         "Formed network successfully (Extended PAN ID: "
                         "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: "
                         "0x%04hx, Channel:%d)",
                         extended_pan_id[7], extended_pan_id[6],
                         extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2],
                         extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel());
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Restart network formation (status: %s)",
                         esp_err_to_name(err_status));
                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Network steering started");
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)
                esp_zb_app_signal_get_params(p_sg_p);
            ESP_LOGI(TAG,
                     "New device commissioned or rejoined (short: 0x%04hx)",
                     dev_annce_params->device_short_addr);
            esp_zb_zdo_match_desc_req_param_t cmd_req;
            cmd_req.dst_nwk_addr     = dev_annce_params->device_short_addr;
            cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
            esp_zb_zdo_find_on_off_light(&cmd_req, user_find_cb, NULL);
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                     esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

static esp_err_t zb_read_attr_resp_handler(
    const esp_zb_zcl_cmd_read_attr_resp_message_t *message) {
    // ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    // ESP_RETURN_ON_FALSE(
    //     message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
    //     ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
    //     message->info.status);
    ESP_LOGI(TAG,
             "Read attribute response: status(%d), cluster(0x%x), "
             "attribute(0x%x), type(0x%x), value(%d)",
             message->info.status, message->info.cluster, message->attribute.id,
             message->attribute.data.type,
             message->attribute.data.value
                 ? *(uint8_t *)message->attribute.data.value
                 : 0);
    // if (message->info.dst_endpoint == HA_ESP_TEMP_ENDPOINT) {
    switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT: {
            ESP_LOGI(TAG, "ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT");

            float temp =
                (float)(*(uint16_t *)message->attribute.data.value / 100.0);
            ESP_LOGI(TAG, "TEMP %.2f", temp);
            // lcd.printf("TEMP %.2f\r\n", temp);
            char info[50] = {0};

            sprintf(info, "%.2fC\r\n", temp);
            sprite.fillRect(0, 20, 320, 30, TFT_BLACK);
            sprite.drawString(info, 160, 25);
            sprite.pushSprite(0, 66);

        }

        break;
        case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT: {
            ESP_LOGI(TAG, "ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT");
            float HUM =
                (float)(*(uint16_t *)message->attribute.data.value / 100.0);
            ESP_LOGI(TAG, "HUM %.2f", HUM);
            // lcd.printf("HUM %.2f\r\n", HUM);
            char info[50] = {0};
            sprintf(info, "%.2f%%\r\n", HUM);
            sprite.fillRect(0, 80, 320, 30, TFT_BLACK);
            sprite.drawString(info, 160, 85);
            sprite.pushSprite(0, 66);

        }

        break;
            // default:
            // ESP_LOGI(TAG,
            //          "Message data: cluster(0x%x), attribute(0x%x)
            //          ",
            //          message->info.cluster,
            //          message->attribute.id);
    };
    // }
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

static void esp_zb_task(void *pvParameters) {
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_on_off_switch_cfg_t switch_cfg =
        ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_switch_ep =
        esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);
    esp_zb_device_register(esp_zb_on_off_switch_ep);

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

extern "C" {

void app_main(void) {
    lcd.init();
    lcd.setBrightness(255);
    lcd.setRotation(1);
    lcd.pushImage(0, 0, 320, 60, image_data_gw_title);
    lcd.pushImage(91, 193, 138, 37, image_data_btn);

    sprite.createSprite(320, 120);
    sprite.setTextDatum(middle_center);
    sprite.setFont(&fonts::FreeSansOblique24pt7b);

    gpio_reset_pin(gpio_num_t(21));
    gpio_reset_pin(gpio_num_t(22));

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    /* load Zigbee gateway platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
    // #if CONFIG_EXAMPLE_CONNECT_WIFI
    //     ESP_ERROR_CHECK(example_connect());
    // #if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
    //     ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    //     esp_coex_wifi_i154_enable();
    // #else
    //     ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    // #endif
    // #endif

    switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair),
                       esp_zb_buttons_handler);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    gpio_num_t btnA = gpio_num_t(39);
    gpio_reset_pin(btnA);
    gpio_set_direction(btnA, GPIO_MODE_INPUT);

    // vTaskDelay(8000);

    while (1) {
        // if (gpio_get_level(btnA) == false) {
        //     vTaskDelay(100);
        //     if (gpio_get_level(btnA) == false) {
        //         ESP_LOGI(TAG, "Reset Device");
        //         esp_zb_factory_reset();
        //         esp_restart();
        //     }
        //     vTaskDelay(1000);
        // }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        esp_zb_zcl_read_attr_cmd_t temp_cmd_req;
        temp_cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0xffff;
        temp_cmd_req.zcl_basic_cmd.dst_endpoint          = 0xff;
        temp_cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        temp_cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        temp_cmd_req.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        temp_cmd_req.attributeID  = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        ESP_EARLY_LOGI(
            TAG, "send 'ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT' read command");
        esp_zb_zcl_read_attr_cmd_req(&temp_cmd_req);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_zb_zcl_read_attr_cmd_t hum_cmd_req;
        hum_cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0xffff;
        hum_cmd_req.zcl_basic_cmd.dst_endpoint          = 0xff;
        hum_cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        hum_cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        hum_cmd_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
        hum_cmd_req.attributeID =
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
        ESP_EARLY_LOGI(
            TAG, "send 'ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT' read command");
        esp_zb_zcl_read_attr_cmd_req(&hum_cmd_req);
    }
}
}
