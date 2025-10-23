/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <wifi_provisioning/manager.h>
#include <cJSON.h>

#include "zero_provisioning.h"
#include "esp_mesh_lite.h"
#include "wifi_prov_mgr.h"

#define WIFI_MAC_ADDR_LEN       (6)
#define MAX_PASSWORD_LEN        (64)

#define RSP_TX_DONE             BIT0
#define POLL_CHANNEL_TX_DONE    BIT1

#define BROADCAST_TX_WAIT_TIME_MS         (3)
#define RSP_TX_WAIT_TIMEOUT               (30)
#define MAX_POLL_CHANNEL_SEND_INTERVAL    (1000)

static const char *TAG = "zero";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct {
    zero_prov_event_id_t event;
    void (*eventfun)(void *);
} zero_prov_table_t;

typedef struct {
    int curState;
    zero_prov_table_t * p_act_table;
    int size;
} zero_prov_act_t;

static uint8_t resend_channel;
static uint8_t resend_mac_addr[ESP_NOW_ETH_ALEN];

static bool zero_prov_done = false;
static bool is_use_zero_prov = false;
static bool zero_provisioner = false;
static bool flg_is_wifi_provisioning = true;
static bool zero_prov_esp_now_init_done = false;

static TimerHandle_t resend_timer = NULL;
static TaskHandle_t zero_prov_handle = NULL;
static TaskHandle_t broadcast_handle = NULL;
static QueueHandle_t s_zero_prov_queue = NULL;
static esp_timer_handle_t g_listen_timer = NULL;
static EventGroupHandle_t zero_provision_event_group = NULL;
static zero_prov_esp_now_data_t *esp_now_data = NULL;
static zero_prov_idle_node_data_t *idle_br_data = NULL;
static wifi_config_t *router_cfg = NULL;

static uint8_t home_channel = 1;
static uint8_t broadcast_channel = 1;
static bool broadcast_tx = false;
static bool channel_increment = true;
static bool wifi_is_scanning = false;
esp_now_switch_channel_t *switch_channel_config = NULL;

static void zero_prov_deinit(void);
esp_err_t zero_prov_br_start(void);
static esp_err_t zero_prov_esp_now_init(void);
static void zero_prov_recieve_handle(void *arg);
static int zero_prov_data_parse(const uint8_t *data, uint16_t data_len);

zero_prov_table_t zero_prov_table[] = {
    {ZERO_PROV_RECIVE_DATA, zero_prov_recieve_handle},
};

bool is_zero_prov_be_used()
{
    return is_use_zero_prov;
}

esp_err_t zero_prov_is_wifi_config(bool *provisioned)
{
    if (!provisioned) {
        return ESP_ERR_INVALID_ARG;
    }

    *provisioned = false;
    wifi_config_t wifi_cfg;
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK) {
        return ESP_FAIL;
    }

    if (strlen((const char *) wifi_cfg.sta.ssid)) {
        *provisioned = true;
    }
    return ESP_OK;
}

esp_err_t zero_prov_connect_ap(wifi_config_t *wifi_cfg)
{
    ESP_LOGI(TAG, "ssid:%s, password:%s", wifi_cfg->sta.ssid, wifi_cfg->sta.password);
#if CONFIG_MESH_LITE_ENABLE
    mesh_lite_sta_config_t config;
    memset(&config, 0x0, sizeof(config));
    memcpy((char*)config.ssid, (char*)wifi_cfg->sta.ssid, sizeof(config.ssid));
    memcpy((char*)config.password, (char*)wifi_cfg->sta.password, sizeof(config.password));
    config.bssid_set = wifi_cfg->sta.bssid_set;
    if (config.bssid_set) {
        memcpy((char*)config.bssid, (char*)wifi_cfg->sta.bssid, sizeof(config.bssid));
    }
    esp_mesh_lite_set_router_config(&config);
    esp_mesh_lite_connect();
#else
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) {
        ESP_LOGE(TAG, "Failed set storage");
        return ESP_FAIL;
    }

    if (esp_wifi_set_config(WIFI_IF_STA, wifi_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed set config");
        return ESP_FAIL;
    }
    if (esp_wifi_connect() != ESP_OK) {
        ESP_LOGE(TAG, "Failed wifi connect");
        return ESP_FAIL;
    }
#endif /* CONFIG_MESH_LITE_ENABLE */

    return ESP_OK;
}

static void zero_prov_del_peer(void)
{
    esp_now_peer_num_t current_peer_num;
    esp_now_peer_info_t *current_peer = NULL;
    ZERO_PROV_CHECK_RETURN_VAIL(esp_now_get_peer_num(&current_peer_num));
    ESP_LOGW(TAG, "now peer number is %d", current_peer_num.total_num);
    ZERO_PROV_CHECK_RETURN_VAIL(esp_now_fetch_peer(true, current_peer));
    if (current_peer) {
        ZERO_PROV_CHECK_RETURN_VAIL(esp_now_del_peer(current_peer->peer_addr));
    }
}

static esp_err_t zero_prov_espnow_create_peer(uint8_t *dst_mac, uint8_t channel)
{
    esp_err_t ret = ESP_FAIL;
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        return ESP_ERR_NO_MEM;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));

    peer->channel = channel;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    // memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(peer->peer_addr, dst_mac, ESP_NOW_ETH_ALEN);

    if (esp_now_is_peer_exist(dst_mac) == false) {
        ret = esp_now_add_peer(peer);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_add_peer failed: %d, mac "MACSTR"", ret, MAC2STR(dst_mac));
        }
    } else {
        ret = esp_now_mod_peer(peer);
    }
    free(peer);

    return ret;
}

esp_err_t zero_prov_br_stop(void)
{
    broadcast_tx = false;

    ESP_LOGI(TAG, "Stop broadcast");

    if (esp_now_data) {
        free(esp_now_data);
        esp_now_data = NULL;
    }
    return ESP_OK;
}

static void resend_timer_timercb(TimerHandle_t timer)
{
    static uint8_t err_count = 0;
    uint16_t length = sizeof(zero_prov_esp_now_data_t);
    zero_prov_esp_now_data_t *pbuf = (zero_prov_esp_now_data_t *)malloc(length);
    if (pbuf == NULL) {
        ESP_LOGE(TAG, "Malloc unicast buff fail");
        return;
    }
    memset(pbuf, 0, length);
    pbuf->type = ESPNOW_DATA_UNICAST_CONFIRM;
    pbuf->len = length;
    pbuf->crc = 0;
    pbuf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)pbuf, pbuf->len);

    zero_prov_espnow_create_peer(resend_mac_addr, 0);

    esp_err_t ret = esp_mesh_lite_espnow_send(ESPNOW_DATA_TYPE_ZERO_PROV, resend_mac_addr, (const uint8_t *)pbuf, pbuf->len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
        if (ret == ESP_ERR_ESPNOW_FULL) {
            zero_prov_del_peer();

            err_count++;
            if (err_count > 5) {
                err_count = 0;
                xTimerStop(resend_timer, 10);
                zero_prov_br_start();
            }
        }
    }
    free(pbuf);

    uint8_t g_channel;
    wifi_second_chan_t g_channel2;
    esp_wifi_get_channel(&g_channel, &g_channel2);
#if ZERO_PROV_DEBUG
    ESP_LOGI(TAG, "%s %d send unicast data to "MACSTR", channel:%d", __func__, __LINE__, MAC2STR(resend_mac_addr), g_channel);
#endif
}

esp_err_t __attribute__((weak)) zero_prov_cust_data_validation(uint8_t *cust_data, size_t cust_data_len)
{
#if ZERO_PROV_DEBUG
    ESP_LOG_BUFFER_HEXDUMP("Cust data Recv", cust_data, cust_data_len, ESP_LOG_DEBUG);
#endif
    return ESP_OK;
}

static void zero_prov_recieve_handle(void *arg)
{
    zero_prov_event_t *evt = (zero_prov_event_t *)arg;
    espnow_recv_cb_t *recv_cb = &evt->info.recv_cb;
    int type = recv_cb->type;
    zero_prov_esp_now_data_t *recvbuf = (zero_prov_esp_now_data_t *)recv_cb->data;

    if (type == ESPNOW_DATA_BROADCAST) {
        if (!zero_provisioner || !flg_is_wifi_provisioning) {
            goto exit;
        }
#if ZERO_PROV_DEBUG
        ESP_LOGW(TAG, "1 Receive ESPNOW_DATA_BROADCAST broadcast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
#endif
        zero_prov_idle_node_data_t *date_br = (zero_prov_idle_node_data_t *)recvbuf->payload;

        if (zero_prov_cust_data_validation(date_br->cust_data, date_br->cust_data_len) != ESP_OK) {
            goto exit;
        }

        uint16_t length = sizeof(zero_prov_esp_now_data_t) + sizeof(zero_prov_unicast_data_t);
        zero_prov_esp_now_data_t *pbuf = (zero_prov_esp_now_data_t *)malloc(length);
        if (pbuf == NULL) {
            ESP_LOGE(TAG, "Malloc unicast buff fail");
            goto exit;
        }
        memset(pbuf, 0, length);
        pbuf->type = ESPNOW_DATA_UNICAST_INFO;
        pbuf->len = length;

        mesh_lite_sta_config_t router_config;
        zero_prov_unicast_data_t unicast_data;
        memset(&router_config, 0x0, sizeof(router_config));
        memset(&unicast_data, 0x0, sizeof(unicast_data));
        esp_mesh_lite_get_router_config(&router_config);
        snprintf((char *)unicast_data.router_ssid, strlen((char *)router_config.ssid) + 1, "%s", router_config.ssid);
        snprintf((char *)unicast_data.router_password, strlen((char *)router_config.password) + 1, "%s", router_config.password);
#if ZERO_PROV_DEBUG
        // ESP_LOGI(TAG, "unicast_data.router_ssid:%s, unicast_data.router_password:%s", unicast_data.router_ssid, unicast_data.router_password);
#endif

        unicast_data.channel = home_channel;
        unicast_data.mesh_id = esp_mesh_lite_get_mesh_id();
        unicast_data.random = esp_mesh_lite_get_argot();

        wifi_config_t wifi_cfg;
        esp_wifi_get_config(WIFI_IF_AP, &wifi_cfg);
        uint8_t softap_mac[WIFI_MAC_ADDR_LEN];
        esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
        snprintf((char *)unicast_data.softap_ssid, MAX_SSID_LEN, CONFIG_DEFAULT_SSID_PREFIX "_%02x%02x%02x", softap_mac[3], softap_mac[4], softap_mac[5]);
        snprintf((char *)unicast_data.softap_password, MAX_PASSWORD_LEN, (char *)wifi_cfg.sta.password);
#if ZERO_PROV_DEBUG
        // ESP_LOGI(TAG, "unicast_data.softap_ssid:%s, unicast_data.softap_password:%s", unicast_data.softap_ssid, unicast_data.softap_password);
#endif

        memcpy(pbuf->payload, &unicast_data, sizeof(zero_prov_unicast_data_t));
        pbuf->crc = 0;
        pbuf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)pbuf, pbuf->len);

        zero_prov_espnow_create_peer(recv_cb->mac_addr, date_br->channel);

        xEventGroupClearBits(zero_provision_event_group, RSP_TX_DONE);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
        size_t config_size = sizeof(esp_now_switch_channel_t) + pbuf->len;
        esp_now_switch_channel_t *switch_channel_config = malloc(config_size);
        if (switch_channel_config == NULL) {
            ESP_LOGE(TAG, "Malloc switch channel config fail");
            goto exit;
        }
        memset(switch_channel_config, 0, config_size);
        switch_channel_config->type = WIFI_OFFCHAN_TX_REQ;
        switch_channel_config->channel = date_br->channel;
        switch_channel_config->wait_time_ms = RSP_TX_WAIT_TIMEOUT;
        switch_channel_config->data_len = pbuf->len;
        memcpy(switch_channel_config->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
        memcpy(switch_channel_config->data, pbuf, pbuf->len);

        esp_err_t ret = esp_mesh_lite_espnow_switch_channel_send(ESPNOW_DATA_TYPE_ZERO_PROV, switch_channel_config, false);
#else
        esp_err_t ret = esp_mesh_lite_espnow_send(ESPNOW_DATA_TYPE_ZERO_PROV, recv_cb->mac_addr, (const uint8_t *)pbuf, pbuf->len);
#endif
        if (ret != ESP_OK) {
            switch (ret) {
            case ESP_ERR_ESPNOW_FULL:
                zero_prov_del_peer();
                break;

            case ESP_ERR_ESPNOW_NO_MEM:
                // ESP_LOGI(TAG, "free heap: %"PRIu32"", esp_get_free_heap_size());
                break;

            default:
                ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
                break;
            }
        }
        free(pbuf);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
        free(switch_channel_config);
#endif

        xEventGroupWaitBits(zero_provision_event_group,
                            RSP_TX_DONE,
                            pdTRUE,
                            pdFALSE,
                            pdMS_TO_TICKS(RSP_TX_WAIT_TIMEOUT));
#if ZERO_PROV_DEBUG
        ESP_LOGW(TAG, "1 Receive ESPNOW_DATA_BROADCAST END**********");
#endif
    } else if (type == ESPNOW_DATA_UNICAST_INFO) {
        if (!resend_timer) {
            goto exit;
        }
        zero_prov_br_stop();
#if ZERO_PROV_DEBUG
        ESP_LOGW(TAG, "2 Receive ESPNOW_DATA_UNICAST_INFO unicast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
#endif
        zero_prov_unicast_data_t *date_unicast = (zero_prov_unicast_data_t *)recvbuf->payload;

        uint16_t length = sizeof(zero_prov_esp_now_data_t);
        zero_prov_esp_now_data_t *pbuf = (zero_prov_esp_now_data_t *)malloc(length);
        if (pbuf == NULL) {
            ESP_LOGE(TAG, "Malloc unicast buff fail");
            goto exit;
        }
        memset(pbuf, 0, length);
        pbuf->type = ESPNOW_DATA_UNICAST_CONFIRM;
        pbuf->len = length;
        pbuf->crc = 0;
        pbuf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)pbuf, pbuf->len);

        esp_wifi_set_channel(date_unicast->channel, 0);
        ESP_LOGI(TAG, "set wifi channel:%d", date_unicast->channel);

        zero_prov_espnow_create_peer(recv_cb->mac_addr, 0);

        esp_err_t ret = esp_mesh_lite_espnow_send(ESPNOW_DATA_TYPE_ZERO_PROV, recv_cb->mac_addr, (const uint8_t *)pbuf, pbuf->len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
            if (ret == ESP_ERR_ESPNOW_FULL) {
                zero_prov_del_peer();
            }
        }
        free(pbuf);

        uint8_t g_channel;
        wifi_second_chan_t g_channel2;
        esp_wifi_get_channel(&g_channel, &g_channel2);
#if ZERO_PROV_DEBUG
        ESP_LOGI(TAG, "%s %d send unicast data to "MACSTR", channel:%d", __func__, __LINE__, MAC2STR(recv_cb->mac_addr), g_channel);
#endif
        resend_channel = date_unicast->channel;
        memcpy(resend_mac_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
        xTimerStart(resend_timer, 0);

        /*****************************************/

        is_use_zero_prov = true;
        flg_is_wifi_provisioning = true;

        esp_mesh_lite_set_mesh_id(date_unicast->mesh_id, true);
        ESP_LOGI(TAG, "[MeshID]: %d", date_unicast->mesh_id);

        esp_mesh_lite_set_argot(date_unicast->random);
        ESP_LOGI(TAG, "[random]: %"PRIu32"", date_unicast->random);

        char softap_ssid[MAX_SSID_LEN + 1];
        uint8_t softap_mac[WIFI_MAC_ADDR_LEN];
        esp_wifi_get_mac(WIFI_IF_AP, softap_mac);

        wifi_ap_config_t config;
        memset(&config, 0x0, sizeof(config));
        strlcpy((char *)config.password, (char *)date_unicast->softap_password, sizeof(config.password));
        esp_mesh_lite_set_softap_psw_to_nvs((char *)config.password);
        ESP_LOGI(TAG, "[SoftAP psw]: %s", config.password);

        esp_mesh_lite_set_softap_ssid_to_nvs((char *)date_unicast->softap_ssid);
        esp_mesh_lite_set_softap_info((char *)date_unicast->softap_ssid, (char*)config.password);
        snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", (char *)date_unicast->softap_ssid, softap_mac[3], softap_mac[4], softap_mac[5]);
        memcpy((char *)config.ssid, softap_ssid, sizeof(config.ssid));
        ESP_LOGI(TAG, "[SoftAP ssid]: %s", (char *)config.ssid);

        config.max_connection = CONFIG_BRIDGE_SOFTAP_MAX_CONNECT_NUMBER;
        config.channel = date_unicast->channel;
        config.authmode = strlen((char*)config.password) < 8 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
        esp_wifi_set_config(ESP_IF_WIFI_AP, (wifi_config_t*)&config);

        memcpy(router_cfg->sta.ssid, date_unicast->router_ssid, sizeof(router_cfg->sta.ssid));
        memcpy(router_cfg->sta.password, date_unicast->router_password, sizeof(router_cfg->sta.password));
        ESP_LOGW(TAG,"router_ssid:%s router_pwd:%s", router_cfg->sta.ssid, router_cfg->sta.password);

#if ZERO_PROV_DEBUG
        ESP_LOGW(TAG, "2 Receive ESPNOW_DATA_UNICAST_INFO END**********");
#endif
    } else if (type == ESPNOW_DATA_UNICAST_CONFIRM) {
#if ZERO_PROV_DEBUG
        ESP_LOGE(TAG, "3 Receive ESPNOW_DATA_UNICAST_CONFIRM unicast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
#endif

        uint16_t length = sizeof(zero_prov_esp_now_data_t) + sizeof(struct tm);
        zero_prov_esp_now_data_t *pbuf = (zero_prov_esp_now_data_t *)malloc(length);
        if (pbuf == NULL) {
            ESP_LOGE(TAG, "Malloc unicast buff fail");
            goto exit;
        }
        memset(pbuf, 0, length);
        pbuf->type = ESPNOW_DATA_UNICAST_ACK;
        pbuf->len = length;

        pbuf->crc = 0;
        pbuf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)pbuf, pbuf->len);

        zero_prov_espnow_create_peer(recv_cb->mac_addr, 0);

        esp_err_t ret = esp_mesh_lite_espnow_send(ESPNOW_DATA_TYPE_ZERO_PROV, recv_cb->mac_addr, (const uint8_t *)pbuf, pbuf->len);
        if (ret != ESP_OK) {
            switch (ret) {
            case ESP_ERR_ESPNOW_FULL:
                zero_prov_del_peer();
                break;

            case ESP_ERR_ESPNOW_NO_MEM:
                // ESP_LOGI(TAG, "free heap: %"PRIu32"", esp_get_free_heap_size());
                break;

            default:
                ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
                break;
            }
        }
        free(pbuf);

#if ZERO_PROV_DEBUG
        ESP_LOGE(TAG, "3 Receive ESPNOW_DATA_UNICAST_CONFIRM END**********");
#endif
    } else if (type == ESPNOW_DATA_UNICAST_ACK) {
        if (resend_timer != NULL) {
            if (xTimerIsTimerActive(resend_timer) != pdFALSE) {
                // xTimer is active, stop it
                xTimerStop(resend_timer, 10);
            }
            xTimerDelete(resend_timer, 10);
            resend_timer = NULL;
        }
#if ZERO_PROV_DEBUG
        ESP_LOGI(TAG, "4 Receive ESPNOW_DATA_UNICAST_ACK unicast data from: "MACSTR", len: %d", MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
#endif

        if (s_zero_prov_queue) {
            xQueueReset(s_zero_prov_queue);
        }

        if (!zero_prov_done) {
            zero_prov_connect_ap(router_cfg);
        }

        zero_prov_done = true;

#if ZERO_PROV_DEBUG
        ESP_LOGE(TAG, "4 Receive ESPNOW_DATA_UNICAST_ACK END**********");
#endif
    } else {
        ESP_LOGW(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
    }

exit:
    free(recv_cb->data);
}

static void zero_prov_regist(zero_prov_act_t* pact, zero_prov_table_t* ptable)
{
    pact->size = sizeof(zero_prov_table)/sizeof(zero_prov_table_t);
    pact->p_act_table = ptable;
}

static void zero_prov_event_handle(zero_prov_act_t* pact, zero_prov_event_id_t event, void *arg)
{
    zero_prov_table_t* p_table = pact->p_act_table;
    void (*p_event_fun)() = NULL;
    int g_max_num = pact->size;

    for (int i = 0; i < g_max_num; i++) {
        if (event == p_table[i].event) {
            p_event_fun = p_table[i].eventfun;
            if (p_event_fun) {
                p_event_fun(arg);
            }
        }
    }
}

void before_wifi_provisioning_scan(void)
{
    ESP_LOGE(TAG, "before_wifi_provisioning_scan");
    wifi_is_scanning = true;
}

void wifi_provisioning_scan_done(void)
{
    ESP_LOGE(TAG, "wifi_provisioning_scan_done");
    wifi_is_scanning = false;
}

#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 4, 2)
static void zero_prov_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
#else
static void zero_prov_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
#endif
{
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 4, 2)
    const uint8_t *mac_addr = tx_info->des_addr;
    if (tx_info == NULL) {
#else
    if (mac_addr == NULL) {
#endif
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(mac_addr)) {
        if (wifi_is_scanning == false && broadcast_tx == true) {
            if (status == ESP_NOW_SEND_SUCCESS) {
                if (switch_channel_config == NULL) {
                    return;
                }

                if (channel_increment) {
                    if (broadcast_channel < 13) {
                        broadcast_channel++;
                        switch_channel_config->channel = broadcast_channel;
                        esp_mesh_lite_espnow_switch_channel_send(ESPNOW_DATA_TYPE_ZERO_PROV, switch_channel_config, false);
                    } else {
                        xEventGroupSetBits(zero_provision_event_group, POLL_CHANNEL_TX_DONE);
                        broadcast_channel = 1;
                    }
                } else {
                    if (broadcast_channel > 1) {
                        broadcast_channel--;
                        switch_channel_config->channel = broadcast_channel;
                        esp_mesh_lite_espnow_switch_channel_send(ESPNOW_DATA_TYPE_ZERO_PROV, switch_channel_config, false);
                    } else {
                        xEventGroupSetBits(zero_provision_event_group, POLL_CHANNEL_TX_DONE);
                        broadcast_channel = 13;
                    }
                }
            } else {
                ESP_LOGW(TAG, "BROADCAST_TX_FAILED");
            }
        }
    } else {
        esp_now_del_peer(mac_addr);
#if ZERO_PROV_DEBUG
        ESP_LOGW(TAG, "send done | del peer "MACSTR"", MAC2STR(mac_addr));
#endif
        xEventGroupSetBits(zero_provision_event_group, RSP_TX_DONE);
    }
}

static int zero_prov_data_parse(const uint8_t *data, uint16_t data_len)
{
    zero_prov_esp_now_data_t *buf = (zero_prov_esp_now_data_t *)data;
    uint16_t crc_cal = 0;

    if (data_len < sizeof(zero_prov_esp_now_data_t)) {
        ESP_LOGD(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    uint8_t crc_zero[2];
    memset(crc_zero, 0, 2);

    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, ZERO_PROV_OFFSETOF(crc));
    crc_cal = esp_crc16_le(crc_cal, (uint8_t const *)crc_zero, 2);
    crc_cal = esp_crc16_le(crc_cal, (uint8_t const *)buf->payload, data_len - ZERO_PROV_ESP_NOW_HEADER_SIZE);

    if (crc_cal == buf->crc) {
        return buf->type;
    }
    return -1;
}

static esp_err_t zero_prov_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    zero_prov_event_t evt;
    espnow_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = (uint8_t *)recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return ESP_FAIL;
    }

    int ret = zero_prov_data_parse(data, len);
    if (ret == -1) {
        return ESP_FAIL;
    }

    evt.id = ZERO_PROV_RECIVE_DATA;
    evt.info.recv_cb.type = ret;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail. free heap: %"PRIu32"", esp_get_free_heap_size());
        return ESP_FAIL;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

    if (!s_zero_prov_queue) {
        ESP_LOGE(TAG, "Receive queue is NULL");
        goto err;
    }

    if (xQueueSend(s_zero_prov_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        xQueueReset(s_zero_prov_queue);
        goto err;
    }

    return ESP_OK;

err:
    if (recv_cb->data) {
        free(recv_cb->data);
        recv_cb->data = NULL;
    }
    return ESP_FAIL;
}

static void zero_prov_task(void *pvParameter)
{
    zero_prov_event_t evt;
    zero_prov_act_t g_fastnetwork;
    zero_prov_regist(&g_fastnetwork, zero_prov_table);

    while (1) {
        if (s_zero_prov_queue) {
            xQueueReceive(s_zero_prov_queue, &evt, portMAX_DELAY);
        }
        zero_prov_event_handle(&g_fastnetwork, evt.id, &evt);
    }
}

// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)

static void broadcast_task(void *pvParameter)
{
    size_t size = sizeof(zero_prov_esp_now_data_t) + sizeof(zero_prov_idle_node_data_t) + idle_br_data->cust_data_len;
    esp_now_data = calloc(1, size);
    if (esp_now_data == NULL) {
        ESP_LOGE(TAG, "calloc failed");
        return;
    }

    esp_now_data->type = ESPNOW_DATA_BROADCAST;
    esp_now_data->len = size;
    memcpy(esp_now_data->payload, idle_br_data, sizeof(zero_prov_idle_node_data_t) + idle_br_data->cust_data_len);
    esp_now_data->crc = 0;
    esp_now_data->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)esp_now_data, esp_now_data->len);

    size_t config_size = sizeof(esp_now_switch_channel_t) + esp_now_data->len;
    if (switch_channel_config != NULL) {
        free(switch_channel_config);
        switch_channel_config = NULL;
    }

    switch_channel_config = malloc(config_size);
    if (switch_channel_config == NULL) {
        ESP_LOGE(TAG, "Malloc switch channel config fail");
        return;
    }
    memset(switch_channel_config, 0, config_size);
    switch_channel_config->type = WIFI_OFFCHAN_TX_REQ;
    switch_channel_config->wait_time_ms = BROADCAST_TX_WAIT_TIME_MS;
    switch_channel_config->data_len = esp_now_data->len;
    memcpy(switch_channel_config->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    memcpy(switch_channel_config->data, esp_now_data, esp_now_data->len);

    uint8_t tx_send_cnt =0;

    while (1) {
        if (broadcast_tx == false) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (wifi_is_scanning == true) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (channel_increment == true) {
            broadcast_channel = 1;
        } else {
            broadcast_channel = 13;
        }

#if ZERO_PROV_DEBUG
        // ESP_LOG_BUFFER_HEXDUMP("Raw Data Send", buf, buf->len, ESP_LOG_WARN);
#endif
        switch_channel_config->channel = broadcast_channel;
        esp_err_t err = esp_mesh_lite_espnow_switch_channel_send(ESPNOW_DATA_TYPE_ZERO_PROV, switch_channel_config, false);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(MAX_POLL_CHANNEL_SEND_INTERVAL));
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(zero_provision_event_group,
                POLL_CHANNEL_TX_DONE,
                pdTRUE,
                pdFALSE,
                pdMS_TO_TICKS(MAX_POLL_CHANNEL_SEND_INTERVAL));

        tx_send_cnt++;
        channel_increment = !channel_increment;

        if (bits & POLL_CHANNEL_TX_DONE) {
            ESP_LOGI(TAG, "broadcast tx data done :tx_cnt=%d", tx_send_cnt);
        } else {
            ESP_LOGE(TAG, "broadcast tx data timeout :tx_cnt=%d", tx_send_cnt);
        }
    }
}

// #endif

void zero_prov_listening_stop(void)
{
    zero_provisioner = false;

    zero_prov_deinit();

    if(g_listen_timer != NULL){
        esp_timer_delete(g_listen_timer);
        g_listen_timer = NULL;
    }

    ESP_LOGW(TAG,"Stop Listening");
}

static void listen_timer_cb(void *arg)
{
    ESP_LOGW(TAG,"Listening Over");

    zero_prov_listening_stop();
}

void zero_prov_listening(uint64_t timeout_s)
{
    if (g_listen_timer) {
        return;
    }

    ESP_LOGI(TAG, "Start zero prov listening, %"PRIu64" timeout", timeout_s);

    if (timeout_s*1000*1000 <= 0) {
        return;
    }

    if(g_listen_timer == NULL){
        esp_timer_create_args_t timer;
        timer.arg = NULL;
        timer.callback = listen_timer_cb;
        timer.dispatch_method = ESP_TIMER_TASK;
        timer.name = "ft_listen";
        esp_timer_create(&timer, &g_listen_timer);
    }

    zero_prov_esp_now_init();

    zero_provisioner = true;

    esp_timer_start_once(g_listen_timer, timeout_s*1000*1000);
}

esp_err_t zero_prov_br_start(void)
{
    broadcast_tx = true;

    return ESP_OK;
}

static void wifi_prov_stop_timer_cb(TimerHandle_t timer)
{
    wifi_provision_stop();
    xTimerStop(timer, 0);
    xTimerDelete(timer, 0);
}

static void zero_prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        TimerHandle_t wifi_prov_stop_timer = xTimerCreate("wifi_prov_stop_timer", pdMS_TO_TICKS(200), pdTRUE,
                NULL, wifi_prov_stop_timer_cb);
        xTimerStart(wifi_prov_stop_timer, portMAX_DELAY);

        wifi_second_chan_t g_channel2;
        esp_wifi_get_channel(&home_channel, &g_channel2);

        zero_prov_listening(ZERO_PROV_LISTENING_TIMEOUT);
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        switch (disconnected->reason) {
        case WIFI_REASON_NO_AP_FOUND:
            zero_prov_listening(ZERO_PROV_LISTENING_TIMEOUT);
            break;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
            zero_prov_listening(ZERO_PROV_LISTENING_TIMEOUT);
            break;
        case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
            zero_prov_listening(ZERO_PROV_LISTENING_TIMEOUT);
            break;
        case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
            zero_prov_listening(ZERO_PROV_LISTENING_TIMEOUT);
            break;
#endif
        default:
            break;
        }
    }

    if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_RECV) {
        ESP_LOGW(TAG, "WIFI_PROV_CRED_RECV");
        is_use_zero_prov = false;
        flg_is_wifi_provisioning = true;
        zero_prov_br_stop();
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_START) {
        ESP_LOGW(TAG, "WIFI_PROV_START");
        zero_prov_br_start();
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_END) {
        ESP_LOGW(TAG, "WIFI_PROV_END");
        zero_prov_br_stop();
    }
}

static esp_err_t zero_prov_esp_now_init(void)
{
    if (zero_prov_esp_now_init_done) {
        ESP_LOGW(TAG, "fast network have been inited");
        return ESP_ERR_INVALID_STATE;
    }
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &zero_prov_event_handler, NULL);
    esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &zero_prov_event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &zero_prov_event_handler, NULL);

    s_zero_prov_queue = xQueueCreate(ZERO_PROV_QUEUE_SIZE, sizeof(zero_prov_event_t));
    if (s_zero_prov_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    zero_provision_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK( esp_now_register_send_cb(zero_prov_send_cb) );
    esp_mesh_lite_espnow_recv_cb_register(ESPNOW_DATA_TYPE_ZERO_PROV, zero_prov_recv_cb);

    esp_err_t ret = zero_prov_espnow_create_peer(s_broadcast_mac, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Create peer fail");
        esp_now_unregister_send_cb();
        vSemaphoreDelete(s_zero_prov_queue);
        zero_prov_handle = NULL;
        return ESP_FAIL;
    }

    router_cfg = calloc(1, sizeof(wifi_config_t));

    xTaskCreate(zero_prov_task, "zero_prov_task", 4096, NULL, 5, &zero_prov_handle);
    xTaskCreate(broadcast_task, "broadcast_task", 4096, NULL, 5, &broadcast_handle);
    zero_prov_esp_now_init_done = true;
    return ESP_OK;
}

esp_err_t zero_provision_set_customer_data(uint8_t *cust_data, size_t cust_data_len)
{
    if (idle_br_data) {
        free(idle_br_data);
        idle_br_data = NULL;
    }

    size_t data_size = cust_data_len;
    if (cust_data_len > ZERO_PROV_CUST_DATA_MAX_LEN || cust_data_len == 0) {
        data_size = 0;
    }

    idle_br_data = calloc(1, sizeof(zero_prov_idle_node_data_t) + data_size);
    if (idle_br_data == NULL) {
        ESP_LOGE(TAG, "Malloc cust data fail");
        return ESP_FAIL;
    }

    uint8_t g_channel = 0;
    wifi_second_chan_t g_channel2;
    esp_wifi_get_channel(&g_channel, &g_channel2);
    idle_br_data->channel = g_channel;
    idle_br_data->cust_data_len = data_size;

    if (cust_data && data_size > 0) {
        memcpy(idle_br_data->cust_data, cust_data, data_size);
        zero_prov_br_start();
    }
    return ESP_OK;
}

esp_err_t zero_prov_init(uint8_t *cust_data, size_t cust_data_len)
{
    zero_provision_set_customer_data(cust_data, cust_data_len);

    zero_prov_is_wifi_config(&flg_is_wifi_provisioning);
    ESP_LOGW(TAG, "Device provisioning state: %s",flg_is_wifi_provisioning?"true":"false");
    if (flg_is_wifi_provisioning) {
        goto exit;
    }

    resend_timer = xTimerCreate("resend_timer", 500 / portTICK_PERIOD_MS,
                                true, NULL, resend_timer_timercb);

    zero_prov_esp_now_init();
    zero_prov_done = false;
    ESP_LOGW(TAG,"fast wifi network init");

exit:
    return ESP_OK;
}

static void zero_prov_deinit(void)
{
    if (zero_prov_esp_now_init_done == false) {
        ESP_LOGW(TAG, "fast network have been deinited");
        return;
    }

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &zero_prov_event_handler);
    esp_event_handler_unregister(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &zero_prov_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &zero_prov_event_handler);
    esp_mesh_lite_espnow_recv_cb_unregister(ESPNOW_DATA_TYPE_ZERO_PROV);
    esp_now_unregister_send_cb();

    if (zero_prov_handle) {
        vTaskDelete(zero_prov_handle);
        zero_prov_handle = NULL;
    }

    if (broadcast_handle) {
        vTaskDelete(broadcast_handle);
        broadcast_handle = NULL;
    }

    if (s_zero_prov_queue) {
        vSemaphoreDelete(s_zero_prov_queue);
        s_zero_prov_queue = NULL;
    }

    if (router_cfg) {
        free(router_cfg);
        router_cfg = NULL;
    }

    if (idle_br_data) {
        free(idle_br_data);
        idle_br_data = NULL;
    }

    zero_prov_esp_now_init_done = false;
}
