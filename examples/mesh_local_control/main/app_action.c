/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_wifi_types.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/*
 * Definitions for sparse validation
 * (http://kernel.org/pub/linux/kernel/people/josh/sparse/)
 */
#ifdef __CHECKER__
#define __force __attribute__((force))
#define __bitwise __attribute__((bitwise))
#else
#define __force
#define __bitwise
#endif

#define WLAN_GET_SEQ_SEQ(seq) \
	(((seq) & (~(BIT(3) | BIT(2) | BIT(1) | BIT(0)))) >> 4)

typedef uint16_t __bitwise le16;

struct ieee80211_hdr {
	le16 frame_control;
	le16 duration_id;
	uint8_t addr1[6];
	uint8_t addr2[6];
	uint8_t addr3[6];
	le16 seq_ctrl;
	/* followed by 'u8 addr4[6];' if ToDS and FromDS is set in data frame
	 */
} STRUCT_PACKED;

static const char *TAG = "wifi action";
static const char *frame_data = "This is a test data";

extern esp_err_t esp_wifi_action_tx_req(uint8_t type, uint8_t channel,
                                        uint32_t wait_time_ms, const wifi_action_tx_req_t *req);
extern esp_err_t esp_wifi_remain_on_channel(uint8_t ifx, uint8_t type, uint8_t channel,
                                        uint32_t wait_time_ms, wifi_action_rx_cb_t rx_cb);

int dummy_rx_action(uint8_t *hdr, uint8_t *payload, size_t len, uint8_t channel)
{
    printf("recv the action frame\r\n");
    return ESP_OK;
}

static int rx_action_handle(uint8_t *hdr, uint8_t *payload, size_t len, uint8_t channel)
{
    struct ieee80211_hdr *rx_hdr = (struct ieee80211_hdr *)hdr;

    ESP_LOGI(TAG, "Rxd Action Frame from " MACSTR " (Seq-%lu)", MAC2STR(rx_hdr->addr2),
             WLAN_GET_SEQ_SEQ(rx_hdr->seq_ctrl));

    printf("the payload is %s\r\n", payload);

    return ESP_OK;
}

static void action_rx_task(void *pvParameters)
{
    // uint8_t channel = 3;
    wifi_action_rx_cb_t rx_cb = rx_action_handle;

    do {
        wifi_config_t wifi_cfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK) {
            continue;
        }
        printf("channel: %d\r\n", wifi_cfg.sta.channel);
        esp_wifi_remain_on_channel(WIFI_IF_STA, WIFI_ROC_REQ, wifi_cfg.sta.channel, 5000, rx_cb);
        // channel = channel % 11 + 1;
        vTaskDelay(pdMS_TO_TICKS(5000));
    } while(1);
}

static void action_tx_task(void *pvParameters)
{
    uint8_t channel = 1;
    wifi_ap_record_t ap_info;

    wifi_action_tx_req_t *req = calloc(sizeof(wifi_action_tx_req_t) + strlen(frame_data), 1);
    uint8_t dest_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint32_t wait_time_ms = 10;

    req->ifx = WIFI_IF_STA;
    memcpy(req->dest_mac, dest_mac, 6);
    req->no_ack = true;
    req->data_len = strlen(frame_data);
    req->rx_cb = dummy_rx_action;
    memcpy(req->data, (const uint8_t *)frame_data, req->data_len);

    do {
        // esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
        // channel = ap_info.primary ? ap_info.primary : channel;
        ESP_LOGI(TAG, "Action Tx - MAC:" MACSTR ", Channel-%d, WaitT-%" PRId32 "", MAC2STR(dest_mac), channel, wait_time_ms);
        // printf("payload: %s\r\n", frame_data);
        esp_wifi_action_tx_req(WIFI_OFFCHAN_TX_REQ, channel, wait_time_ms, req);
        // channel = channel % 11 + 1;
        vTaskDelay(pdMS_TO_TICKS(200));
    } while(1);
}

esp_err_t esp_wifi_action_rx_start(bool status)
{
    esp_err_t ret = pdPASS;
    static bool action_rx = false;
    static TaskHandle_t action_rx_task_handle = NULL;

    if (!action_rx && status) {
        ret = xTaskCreate(action_rx_task, "action_rx", 2048, NULL, 5, &action_rx_task_handle);
        action_rx = true;
    }

    if (action_rx && !status) {
        vTaskDelete(action_rx_task_handle);
        action_rx_task_handle = NULL;
        action_rx = false;
    }
    return ret;
}

esp_err_t esp_wifi_action_tx_start(bool status)
{
    esp_err_t ret = pdPASS;
    static bool action_tx = false;
    static TaskHandle_t action_tx_task_handle = NULL;

    if (!action_tx && status) {
        ret = xTaskCreate(action_tx_task, "action_tx", 1024*3, NULL, 5, &action_tx_task_handle);
        action_tx = true;
    }

    if (action_tx && !status) {
        vTaskDelete(action_tx_task_handle);
        action_tx_task_handle = NULL;
        action_tx = false;
    }
    return ret;
}
