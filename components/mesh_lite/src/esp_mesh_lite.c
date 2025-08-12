/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "mesh_lite.pb-c.h"

static const char *TAG = "Mesh-Lite";

static void esp_mesh_lite_event_sta_lost_ip_handler(void *arg, esp_event_base_t event_base,
                                                    int32_t event_id, void *event_data)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGW(TAG, "STA lost IP, reconnecting");
        esp_mesh_lite_connect();
    }
}

static void esp_mesh_lite_event_ip_changed_handler(void *arg, esp_event_base_t event_base,
                                                   int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ESP_MESH_LITE_EVENT_CORE_STARTED:
        ESP_LOGI(TAG, "Mesh-Lite connecting");
        esp_mesh_lite_connect();
        break;
    case ESP_MESH_LITE_EVENT_CORE_INHERITED_NET_SEGMENT_CHANGED:
        ESP_LOGI(TAG, "netif network segment conflict check");
        if (esp_mesh_lite_get_level() > CONFIG_MESH_LITE_MAXIMUM_LEVEL_ALLOWED) {
            esp_wifi_deauth_sta(0);
            ESP_LOGW(TAG, "The Mesh connection for the current node has exceeded the maximum limit, deauthenticating child node and disconnecting Wi-Fi to search for a new parent node.");
            esp_mesh_lite_connect();
        }
        esp_bridge_netif_network_segment_conflict_update(NULL);
        break;
    case ESP_MESH_LITE_EVENT_CORE_ROUTER_INFO_CHANGED:
        break;
    case ESP_MESH_LITE_EVENT_OTA_START:
        ESP_LOGI(TAG, "OTA Start");
        break;
    case ESP_MESH_LITE_EVENT_OTA_FINISH: {
        esp_mesh_lite_event_ota_finish_t *event = (esp_mesh_lite_event_ota_finish_t*)event_data;
        if (event->reason == ESP_MESH_LITE_EVENT_OTA_SUCCESS) {
            ESP_LOGI(TAG, "LAN OTA Success!");
#ifdef CONFIG_OTA_AUTO_RESTART
            esp_restart();
#endif
        } else if (event->reason == ESP_MESH_LITE_EVENT_OTA_REJECTED) {
            ESP_LOGE(TAG, "LAN OTA Rejected\r\n");
        } else {
            ESP_LOGE(TAG, "LAN OTA Fail! Reason: %d\r\n", event->reason);
        }
        break;
    }
    case ESP_MESH_LITE_EVENT_OTA_PROGRESS: {
        esp_mesh_lite_event_ota_progress_t *event = (esp_mesh_lite_event_ota_progress_t*)event_data;
        ESP_LOGI(TAG, "LAN OTA Percentage: %d%%", event->percentage);
        break;
    }
    }
}

void esp_mesh_lite_init(esp_mesh_lite_config_t* config)
{
    ESP_LOGI(TAG, "esp-mesh-lite component version: %d.%d.%d", MESH_LITE_VER_MAJOR, MESH_LITE_VER_MINOR, MESH_LITE_VER_PATCH);

    esp_bridge_network_segment_check_register(esp_mesh_lite_network_segment_is_used);
    esp_event_handler_instance_register(ESP_MESH_LITE_EVENT, ESP_EVENT_ANY_ID, &esp_mesh_lite_event_ip_changed_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &esp_mesh_lite_event_sta_lost_ip_handler, NULL, NULL);

    esp_mesh_lite_espnow_init();

    esp_mesh_lite_core_init(config);

#if CONFIG_MESH_LITE_WIRELESS_DEBUG
    esp_mesh_lite_wireless_debug_init();
#endif

#if CONFIG_OTA_AUTO_CANCEL_ROLLBACK
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
#endif

    esp_wifi_set_inactive_time(WIFI_IF_AP, CONFIG_MESH_LITE_SOFTAP_INACTIVE_TIME);
}
