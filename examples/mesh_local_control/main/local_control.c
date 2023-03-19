/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <sys/socket.h>

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

#include "led_indicator.h"
#include "led_indicator_blink_default.h"
#include "usbh_modem_board.h"
#include "usbh_modem_wifi.h"

static const char *TAG = "local_control";

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

    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
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

static esp_err_t wifi_init(void)
{
#if defined(CONFIG_BRIDGE_EXTERNAL_NETIF_STATION)
    // Station
    esp_bridge_wifi_set(WIFI_MODE_STA, CONFIG_ROUTER_SSID, CONFIG_ROUTER_PASSWORD, NULL);
#endif

#if defined(CONFIG_BRIDGE_DATA_FORWARDING_NETIF_SOFTAP)
    // Softap
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x0, sizeof(wifi_config_t));
    size_t softap_ssid_len = sizeof(wifi_config.ap.ssid);
    if (esp_mesh_lite_get_softap_ssid_from_nvs((char *)wifi_config.ap.ssid, &softap_ssid_len) != ESP_OK) {
        snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
    }
    size_t softap_psw_len = sizeof(wifi_config.ap.password);
    if (esp_mesh_lite_get_softap_psw_from_nvs((char *)wifi_config.ap.password, &softap_psw_len) != ESP_OK) {
        strlcpy((char *)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    }
    esp_bridge_wifi_set(WIFI_MODE_AP, (char *)wifi_config.ap.ssid, (char *)wifi_config.ap.password, NULL);
#endif
    return ESP_OK;
}

#define LED_RED_SYSTEM_GPIO                 CONFIG_EXAMPLE_LED_RED_SYSTEM_GPIO
#define LED_BLUE_WIFI_GPIO                  CONFIG_EXAMPLE_LED_BLUE_WIFI_GPIO
#define LED_GREEN_4GMODEM_GPIO              CONFIG_EXAMPLE_LED_GREEN_4GMODEM_GPIO
#define LED_ACTIVE_LEVEL                    1

static led_indicator_handle_t s_led_system_handle = NULL;
static led_indicator_handle_t s_led_wifi_handle = NULL;
static led_indicator_handle_t s_led_4g_handle = NULL;

static void _led_indicator_init()
{
    led_indicator_gpio_config_t led_indicator_gpio_config = {
        .is_active_level_high = LED_ACTIVE_LEVEL,
    };

    led_indicator_config_t led_config = {
        .led_indicator_gpio_config = &led_indicator_gpio_config,
        .mode = LED_GPIO_MODE,
    };

    if (LED_RED_SYSTEM_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_RED_SYSTEM_GPIO;
        s_led_system_handle = led_indicator_create(&led_config);
        assert(s_led_system_handle != NULL);
    }
    if (LED_BLUE_WIFI_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_BLUE_WIFI_GPIO;
        s_led_wifi_handle = led_indicator_create(&led_config);
        assert(s_led_wifi_handle != NULL);
        led_indicator_stop(s_led_wifi_handle, BLINK_CONNECTED);
        led_indicator_start(s_led_wifi_handle, BLINK_CONNECTING);
    }
    if (LED_GREEN_4GMODEM_GPIO) {
        led_indicator_gpio_config.gpio_num = LED_GREEN_4GMODEM_GPIO;
        s_led_4g_handle = led_indicator_create(&led_config);
        assert(s_led_4g_handle != NULL);
        led_indicator_stop(s_led_4g_handle, BLINK_CONNECTED);
        led_indicator_start(s_led_4g_handle, BLINK_CONNECTING);
    }
}

static void on_modem_event(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == MODEM_BOARD_EVENT) {
        if ( event_id == MODEM_EVENT_SIMCARD_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: SIM Card disconnected");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_SIMCARD_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: SIM Card Connected");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_DTE_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: USB disconnected");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTING);
        } else if ( event_id == MODEM_EVENT_DTE_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: USB connected");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTING);
        } else if ( event_id == MODEM_EVENT_DTE_RESTART) {
            ESP_LOGW(TAG, "Modem Board Event: Hardware restart");
            led_indicator_start(s_led_system_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_DTE_RESTART_DONE) {
            ESP_LOGI(TAG, "Modem Board Event: Hardware restart done");
            led_indicator_stop(s_led_system_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_NET_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: Network connected");
            led_indicator_start(s_led_4g_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_NET_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: Network disconnected");
            led_indicator_stop(s_led_4g_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_WIFI_STA_CONN) {
            ESP_LOGI(TAG, "Modem Board Event: Station connected");
            led_indicator_start(s_led_wifi_handle, BLINK_CONNECTED);
        } else if ( event_id == MODEM_EVENT_WIFI_STA_DISCONN) {
            ESP_LOGW(TAG, "Modem Board Event: All stations disconnected");
            led_indicator_stop(s_led_wifi_handle, BLINK_CONNECTED);
        }
    }
}

void modem_test()
{
    /* Initialize led indicator */
    _led_indicator_init();

    /* Waiting for modem powerup */
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "     ESP 4G Cat.1 Wi-Fi Router");
    ESP_LOGI(TAG, "====================================");

    /* Initialize modem board. Dial-up internet */
    modem_config_t modem_config = MODEM_DEFAULT_CONFIG();
    /* Modem init flag, used to control init process */
// #ifndef CONFIG_EXAMPLE_ENTER_PPP_DURING_INIT
    /* if Not enter ppp, modem will enter command mode after init */
    modem_config.flags |= MODEM_FLAGS_INIT_NOT_ENTER_PPP;
    /* if Not waiting for modem ready, just return after modem init */
    modem_config.flags |= MODEM_FLAGS_INIT_NOT_BLOCK;
// #endif
    modem_config.handler = on_modem_event;
    modem_board_init(&modem_config);
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

    modem_test();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

#ifndef CONFIG_BRIDGE_EXTERNAL_NETIF_STATION
#if defined(CONFIG_BRIDGE_EXTERNAL_NETIF_ETHERNET) || defined(CONFIG_BRIDGE_EXTERNAL_NETIF_MODEM)
    esp_mesh_lite_set_router_ssid(CONFIG_ROUTER_SSID);
#endif
#endif

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
