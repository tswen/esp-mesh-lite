/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/socket.h>

#include "esp_mac.h"
#include "esp_bridge.h"
#include "esp_mesh_lite.h"

static const char *TAG = "no_router";

typedef enum {
    MESH_LITE_MSG_ID_BROADCAST_INFO_TO_CHILD = 10,
    MESH_LITE_MSG_ID_BROADCAST_INFO_TO_SIBLING,
    MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT,
    MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT_ACK,
    MESH_LITE_MSG_ID_REPORT_INFO_TO_PARENT,
    MESH_LITE_MSG_ID_REPORT_INFO_TO_PARENT_ACK,
} app_msg_id_t;

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary                 = 0;
    uint8_t sta_mac[6]              = {0};
    wifi_ap_record_t ap_info        = {0};
    wifi_second_chan_t second       = 0;
    wifi_sta_list_t wifi_sta_list   = {0x0};

    if (esp_mesh_lite_get_level() > 1) {
        esp_wifi_sta_get_ap_info(&ap_info);
    }
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, free heap: %"PRIu32"", primary,
             esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

    uint32_t size = 0;
    const node_info_list_t *node = esp_mesh_lite_get_nodes_list(&size);
    printf("MeshLite nodes %ld:\r\n", size);
    for (uint32_t loop = 0; (loop < size) && (node != NULL); loop++) {
        struct in_addr ip_struct;
        ip_struct.s_addr = node->node->ip_addr;
        printf("%ld: %d, "MACSTR", %s\r\n" , loop + 1, node->node->level, MAC2STR(node->node->mac_addr), inet_ntoa(ip_struct));
        node = node->next;
    }
}

#include "iot_button.h"
#define BUTTON_NUM            1
#define BUTTON_SW1            9
static button_handle_t g_btns[BUTTON_NUM] = { 0 };
static TimerHandle_t test_timer;

static uint8_t report_raw_msg_to_root_data[12] = {0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21};

static esp_err_t report_raw_msg_to_root_process(uint8_t *data, uint32_t len, uint8_t **out_data, uint32_t* out_len, uint32_t seq)
{
    *out_len = 0;
    printf("[recv report from child | seq %"PRIu32"]\r\n", seq);
    ESP_LOG_BUFFER_HEXDUMP("recv report from child", data, len, ESP_LOG_WARN);
    return ESP_OK;
}

static esp_err_t report_raw_msg_to_root_ack_process(uint8_t *data, uint32_t len, uint8_t **out_data, uint32_t* out_len, uint32_t seq)
{
    return ESP_OK;
}

esp_err_t esp_mesh_lite_report_raw_msg_to_root(void)
{
    ESP_LOG_BUFFER_HEXDUMP("send report to root", report_raw_msg_to_root_data, sizeof(report_raw_msg_to_root_data), ESP_LOG_WARN);
    esp_mesh_lite_msg_config_t config = {
        .raw_msg = {
            .msg_id = MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT,
            .expect_resp_msg_id = MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT_ACK,
            .max_retry = 3,
            .data = report_raw_msg_to_root_data,
            .size = sizeof(report_raw_msg_to_root_data),
            .raw_resend = esp_mesh_lite_send_raw_msg_to_root,
        },
    };
    esp_mesh_lite_send_msg(ESP_MESH_LITE_RAW_MSG, &config);
    return ESP_OK;
}

static const esp_mesh_lite_raw_msg_action_t raw_msgs_test_action[] = {
    // {MESH_LITE_MSG_ID_BROADCAST_INFO_TO_CHILD, 0, broadcast_raw_msg_to_child_process},

    /* broadcast raw msg to the sibling node */
    // {MESH_LITE_MSG_ID_BROADCAST_INFO_TO_SIBLING, 0, broadcast_raw_msg_to_sibling_process},

    /* report raw msg to the root node */
    {MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT, MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT_ACK, report_raw_msg_to_root_process},
    {MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT_ACK, 0, report_raw_msg_to_root_ack_process},

    /* report raw msg to the parent node */
    // {MESH_LITE_MSG_ID_REPORT_INFO_TO_PARENT, MESH_LITE_MSG_ID_REPORT_INFO_TO_PARENT_ACK, report_raw_msg_to_parent_process},
    // {MESH_LITE_MSG_ID_REPORT_INFO_TO_PARENT_ACK, 0, report_raw_msg_to_parent_ack_process},

    {0, 0, NULL}
};

void esp_mesh_lite_comm_init(void)
{
    esp_mesh_lite_raw_msg_action_list_register(raw_msgs_test_action);
}

static void test_timer_cb(TimerHandle_t timer)
{
    ESP_LOGI(TAG, "communication test running...");
    char* outdata = "Hello from no_router!";
    uint32_t out_len = sizeof(outdata);
    esp_mesh_lite_msg_config_t config = {
        .raw_msg = {
            .msg_id = MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT,
            .expect_resp_msg_id = MESH_LITE_MSG_ID_REPORT_INFO_TO_ROOT_ACK,
            .max_retry = 3,
            .data = (uint8_t*)outdata,
            .size = out_len,
            .raw_resend = esp_mesh_lite_send_raw_msg_to_root,
        },
    };
    esp_mesh_lite_send_msg(ESP_MESH_LITE_RAW_MSG, &config);
    ESP_LOGI(TAG, "Free heap: %" PRIu32 "", esp_get_free_heap_size());
}

static void button_press_up_cb(void *hardware_data, void *usr_data)
{
    ESP_LOGI(TAG, "BTN: BUTTON_PRESS_UP");

    xTimerStart(test_timer, 0);
}

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

static void wifi_init(void)
{
    // Station
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x0, sizeof(wifi_config_t));
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    wifi_config_t wifi_softap_config = {
                                           .ap = {
                                                     .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
                                                     .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
                                                     .channel = CONFIG_MESH_CHANNEL,
                                                 },
                                       };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_softap_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[33];
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t ssid_size = sizeof(softap_ssid);
    size_t psw_size = sizeof(softap_psw);
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));
    memset(softap_psw, 0x0, sizeof(softap_psw));

    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &ssid_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get ssid from nvs: %s", softap_ssid);
    } else {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
        snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
        ESP_LOGI(TAG, "Get ssid from nvs failed, set ssid: %s", softap_ssid);
    }

    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &psw_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get psw from nvs: [HIDDEN]");
    } else {
        strlcpy(softap_psw, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(softap_psw));
        ESP_LOGI(TAG, "Get psw from nvs failed, set psw: [HIDDEN]");
    }

    esp_mesh_lite_set_softap_info(softap_ssid, softap_psw);
}

void app_main()
{
    // Set the log level for serial port printing.
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    mesh_lite_config.join_mesh_ignore_router_status = true;
#if CONFIG_MESH_ROOT
    mesh_lite_config.join_mesh_without_configured_wifi = false;
#else
    mesh_lite_config.join_mesh_without_configured_wifi = true;
#endif
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();

#if CONFIG_MESH_ROOT
    ESP_LOGI(TAG, "Root node");
    esp_mesh_lite_set_allowed_level(1);
#else
    ESP_LOGI(TAG, "Child node");
    esp_mesh_lite_set_disallowed_level(1);
#endif

    esp_mesh_lite_start();

    esp_mesh_lite_comm_init();

    // TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
    //                                    true, NULL, print_system_info_timercb);
    // xTimerStart(timer, 0);

    test_timer = xTimerCreate("test_timer", 500 / portTICK_PERIOD_MS, true, NULL, test_timer_cb);

    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = BUTTON_SW1,
            .active_level = 0,
        },
    };
    g_btns[0] = iot_button_create(&cfg);
    iot_button_register_cb(g_btns[0], BUTTON_PRESS_UP, button_press_up_cb, 0);
}
