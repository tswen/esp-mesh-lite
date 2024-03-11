/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inttypes.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "app_wifi.h"
#include "app_bridge.h"
#include "app_espnow.h"
#include "app_rainmaker.h"

static const char *TAG = "app_main";

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_mac.h"

static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary                 = 0;
    uint8_t sta_mac[6]              = {0};
    wifi_ap_record_t ap_info        = {0};
    wifi_second_chan_t second       = 0;
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_sta_get_ap_info(&ap_info);
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, free heap: %"PRIu32"", primary,
             esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());
#if CONFIG_MESH_LITE_MAXIMUM_NODE_NUMBER
    ESP_LOGI(TAG, "child node number: %d", esp_mesh_lite_get_child_node_number());
#endif /* MESH_LITE_NODE_INFO_REPORT */
    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

void session_cost_information(const char *TAG, const char *func, int line, const char *desc)
{
    static uint32_t free_heap = 0;

    ESP_LOGW(TAG, "%s %d %s const heap %"PRIu32"", func, line, desc ? desc : "NULL", esp_get_free_heap_size() - free_heap);
    free_heap = esp_get_free_heap_size();
    ESP_LOGW(TAG, "free heap %"PRIu32", minimum %"PRIu32"", free_heap, esp_get_minimum_free_heap_size());
}

static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    static bool tcp_task = false;

    if (!tcp_task) {
        app_udp_server_create();
        app_tcp_client_create();
        tcp_task = true;
    }
}

void app_main(void)
{
    session_cost_information(TAG, __func__, __LINE__, "app_main");

    esp_storage_init();

    app_rmaker_enable_bridge();

    app_rainmaker_start();

    group_control_init();

    /* Start wifi provisioning */
    app_wifi_start(POP_TYPE_MAC);

    app_rmaker_mesh_lite_service_create();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

    TimerHandle_t timer = xTimerCreate("print_system_info", 20000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
