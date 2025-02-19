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
#if defined(CONFIG_MESH_LITE_PROV_TRANSPORT_BLE)
#include "wifi_prov_mgr.h"
#endif
#if defined(CONFIG_MESH_LITE_PROV_ENABLE)
#include "zero_provisioning.h"
#endif
#include <wifi_provisioning/manager.h>

static int g_sockfd    = -1;
static const char *TAG = "local_control";

#include "led.h"
#include "app_espnow.h"
#include "iot_button.h"
#define BUTTON_NUM            1
#define BUTTON_SW1            9
static button_handle_t g_btns[BUTTON_NUM] = { 0 };

bool espnow_control = false;

/**
 * @brief Create a tcp client
 */
static int socket_tcp_client_create(const char *ip, uint16_t port)
{
    ESP_LOGI(TAG, "Create a tcp client, ip: %s, port: %d", ip, port);

    esp_err_t ret = ESP_OK;
    int sockfd    = -1;
    struct ifreq iface;
    memset(&iface, 0x0, sizeof(iface));
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = inet_addr(ip),
    };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ESP_LOGE(TAG, "socket create, sockfd: %d", sockfd);
        goto ERR_EXIT;
    }

    esp_netif_get_netif_impl_name(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), iface.ifr_name);
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(struct ifreq)) != 0) {
        ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sockfd, iface.ifr_name);
    }

    ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        ESP_LOGD(TAG, "socket connect, ret: %d, ip: %s, port: %d", ret, ip, port);
        goto ERR_EXIT;
    }
    return sockfd;

ERR_EXIT:

    if (sockfd != -1) {
        close(sockfd);
    }

    return -1;
}

void tcp_client_write_task(void *arg)
{
    size_t size        = 0;
    int count          = 0;
    char *data         = NULL;
    esp_err_t ret      = ESP_OK;
    uint8_t sta_mac[6] = {0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    ESP_LOGI(TAG, "TCP client write task is running");

    while (1) {
        if (g_sockfd == -1) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            g_sockfd = socket_tcp_client_create(CONFIG_SERVER_IP, CONFIG_SERVER_PORT);
            continue;
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS);

        size = asprintf(&data, "{\"src_addr\": \"" MACSTR "\",\"data\": \"Hello TCP Server!\",\"level\": %d,\"led\": %d,\"count\": %d}\r\n",
                        MAC2STR(sta_mac), esp_mesh_lite_get_level(), get_led_status(), count++);

        printf("TCP write, size: %d, data: %s", size, data);
        ret = write(g_sockfd, data, size);
        free(data);

        if (ret <= 0) {
            ESP_LOGE(TAG, "<%s> TCP write", strerror(errno));
            close(g_sockfd);
            g_sockfd = -1;
            continue;
        }
    }

    ESP_LOGI(TAG, "TCP client write task is exit");

    close(g_sockfd);
    g_sockfd = -1;
    if (data) {
        free(data);
    }
    vTaskDelete(NULL);
}

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

    esp_wifi_sta_get_ap_info(&ap_info);
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, free heap: %"PRIu32"", primary,
             esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());
#if CONFIG_MESH_LITE_NODE_INFO_REPORT
    ESP_LOGI(TAG, "All node number: %"PRIu32"", esp_mesh_lite_get_mesh_node_number());
#endif /* MESH_LITE_NODE_INFO_REPORT */
    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    static bool tcp_task = false;

    if (!tcp_task) {
        xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024, NULL, 5, NULL);
        tcp_task = true;
    }
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
    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
    strlcpy((char *)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[32];
    uint8_t softap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));

#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
    snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
    snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
    esp_mesh_lite_set_softap_ssid_to_nvs(softap_ssid);
    esp_mesh_lite_set_softap_psw_to_nvs(CONFIG_BRIDGE_SOFTAP_PASSWORD);
    esp_mesh_lite_set_softap_info(softap_ssid, CONFIG_BRIDGE_SOFTAP_PASSWORD);
}

void report_led_status(bool led_status)
{
    cJSON *item = cJSON_CreateObject();
    if (item) {
        if (led_status) {
            cJSON_AddTrueToObject(item, "led_status");
        } else {
            cJSON_AddFalseToObject(item, "led_status");
        }
        esp_mesh_lite_try_sending_msg("led_status_report", "led_status_rsp", 5, item, &esp_mesh_lite_send_msg_to_root);
        cJSON_Delete(item);
    }
}

void broadcast_led_status(bool led_status)
{
    cJSON *item = cJSON_CreateObject();
    if (item) {
        if (led_status) {
            cJSON_AddTrueToObject(item, "led_status");
        } else {
            cJSON_AddFalseToObject(item, "led_status");
        }
        esp_mesh_lite_try_sending_msg("led_status_br", NULL, 7, item, &esp_mesh_lite_send_broadcast_msg_to_child);
        cJSON_Delete(item);
    }
}

static cJSON* led_status_report_process(cJSON *payload, uint32_t seq)
{
    cJSON *found = cJSON_GetObjectItem(payload, "led_status");
    if (found) {
        ESP_LOGI(TAG, "led_status_report_process, led_status: %s", cJSON_IsTrue(found) ? "true" : "false");
        bool led_status = cJSON_IsTrue(found);
        if (espnow_control == true) {
            app_espnow_control_led_status(led_status);
        } else {
            broadcast_led_status(led_status);
            if (led_status) {
                led_on();
            } else {
                led_off();
            }
        }
    }
    return NULL;
}

static cJSON* led_status_br_process(cJSON *payload, uint32_t seq)
{
    static uint32_t last_recv_seq;

    if (last_recv_seq != seq) {
        cJSON *found = cJSON_GetObjectItem(payload, "led_status");
        if (found) {
            // ESP_LOGI(TAG, "led_status_br_process, led_status: %s", cJSON_IsTrue(found) ? "true" : "false");
            bool led_status = cJSON_IsTrue(found);
            broadcast_led_status(led_status);

            if (led_status) {
                led_on();
            } else {
                led_off();
            }
        }
        last_recv_seq = seq;
    }
    return NULL;
}

static const esp_mesh_lite_msg_action_t app_msg_action[] = {
    {"led_status_br", NULL, led_status_br_process},

    {"led_status_report", "led_status_rsp", led_status_report_process},
    {"led_status_rsp", NULL, NULL},

    {NULL, NULL, NULL}
};

static void button_press_up_cb(void *hardware_data, void *usr_data)
{
    ESP_LOGI(TAG, "BTN: BUTTON_PRESS_UP");

    bool status = !get_led_status();
    if (esp_mesh_lite_get_level() == ROOT) {
        if (espnow_control == true) {
            app_espnow_control_led_status(status);
        } else {
            broadcast_led_status(status);
            if (status) {
                led_on();
            } else {
                led_off();
            }
        }
    } else {
        report_led_status(status);
    }
}

static void button_init(void)
{
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

void app_main()
{
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();

    esp_mesh_lite_fusion_config_t fusion_config = {
        .fusion_frequency_sec = 40,   // Set the network to perform fusion every 120 seconds thereafter.
    };
    // Apply the fusion configuration to the mesh network.
    esp_mesh_lite_comm_set_fusion_config(&fusion_config);

    esp_mesh_lite_start();

#if defined(CONFIG_MESH_LITE_PROV_ENABLE)
    zero_prov_init(NULL, NULL);
#endif

#if defined(CONFIG_MESH_LITE_PROV_TRANSPORT_BLE)
    esp_mesh_lite_wifi_prov_mgr_init();
#endif /* CONFIG_APP_BRIDGE_USE_WIFI_PROVISIONING_OVER_BLE */

    /**
     * @breif Create handler
     */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);

    esp_mesh_lite_msg_action_list_register(app_msg_action);

    configure_led();

    button_init();

    bool provisioned = false;
    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));
    /* If device is not yet provisioned start provisioning service */
    if (provisioned) {
        printf("Already provisioned------------------------\n");
        espnow_control = true;
        group_control_init();
    }
}
