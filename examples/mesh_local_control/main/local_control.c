/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/socket.h>

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

static int g_sockfd    = -1;
static const char *TAG = "local_control";

#define TCP_SERVER_PORT  80
#define TCP_SERVER_MAX_CONN 5

#define TCP_CLIENT_IP "192.168.5.1"
#define TCP_CLIENT_PORT 80

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

    ESP_LOGD(TAG, "Creating TCP socket...");
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ESP_LOGE(TAG, "socket create failed, sockfd: %d, errno: %d (%s)", sockfd, errno, strerror(errno));
        goto ERR_EXIT;
    }
    ESP_LOGD(TAG, "Socket created successfully, sockfd: %d", sockfd);

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to get WIFI_STA_DEF netif handle");
        goto ERR_EXIT;
    }
    esp_netif_get_netif_impl_name(netif, iface.ifr_name);
    ESP_LOGD(TAG, "Binding socket to interface: %s", iface.ifr_name);

    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(struct ifreq)) != 0) {
        ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail, errno: %d (%s)", sockfd, iface.ifr_name, errno, strerror(errno));
    } else {
        ESP_LOGD(TAG, "Bind [sock=%d] to interface %s success", sockfd, iface.ifr_name);
    }

    ESP_LOGD(TAG, "Connecting to %s:%d...", ip, port);
    ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        ESP_LOGE(TAG, "socket connect failed, ret: %d, ip: %s, port: %d, errno: %d (%s)", ret, ip, port, errno, strerror(errno));
        goto ERR_EXIT;
    }
    ESP_LOGI(TAG, "Connected to %s:%d successfully, sockfd: %d", ip, port, sockfd);
    return sockfd;

ERR_EXIT:
    ESP_LOGD(TAG, "socket_tcp_client_create failed, cleaning up...");
    if (sockfd != -1) {
        close(sockfd);
        ESP_LOGD(TAG, "Socket %d closed", sockfd);
    }

    return -1;
}

/**
 * @brief Create a tcp server bound to station netif
 */
static int socket_tcp_server_create(uint16_t port)
{
    int sockfd = -1;
    int opt = 1;
    struct ifreq iface;
    memset(&iface, 0x0, sizeof(iface));
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ESP_LOGE(TAG, "Failed to create socket: %d", errno);
        return -1;
    }

    // 设置 socket 选项，允许地址重用
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // // 绑定到 station netif
    // esp_netif_get_netif_impl_name(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), iface.ifr_name);
    // if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(struct ifreq)) != 0) {
    //     ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sockfd, iface.ifr_name);
    // } else {
    //     ESP_LOGI(TAG, "Bindto interface %s success", iface.ifr_name);
    // }

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: %d", errno);
        close(sockfd);
        return -1;
    }

    if (listen(sockfd, TCP_SERVER_MAX_CONN) < 0) {
        ESP_LOGE(TAG, "Failed to listen on socket: %d", errno);
        close(sockfd);
        return -1;
    }

    ESP_LOGI(TAG, "TCP server created on port %d", port);
    return sockfd;
}

/**
 * @brief Handle data from a connected client
 * @return true if client is still connected, false if disconnected
 */
static bool tcp_server_handle_client_data(int client_sock)
{
    char rx_buffer[128];
    int len;

    len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len < 0) {
        ESP_LOGE(TAG, "recv failed: %d", errno);
        return false;
    } else if (len == 0) {
        ESP_LOGI(TAG, "Client (fd=%d) disconnected", client_sock);
        return false;
    }

    rx_buffer[len] = '\0';
    ESP_LOGI(TAG, "Received %d bytes from fd=%d: %s", len, client_sock, rx_buffer);

    // 回复客户端
    const char *response = "Server received your message\r\n";
    int ret = send(client_sock, response, strlen(response), 0);
    if (ret < 0) {
        ESP_LOGE(TAG, "send failed: %d", errno);
        return false;
    }

    return true;
}

void tcp_server_write_task(void *arg)
{
    int server_sock = -1;
    int client_socks[TCP_SERVER_MAX_CONN];
    int client_count = 0;
    fd_set read_fds;
    int max_fd;
    struct timeval timeout;

    // 初始化客户端数组
    for (int i = 0; i < TCP_SERVER_MAX_CONN; i++) {
        client_socks[i] = -1;
    }

    ESP_LOGI(TAG, "TCP server task is running (select mode, max %d clients)", TCP_SERVER_MAX_CONN);

    while (1) {
        // 创建 server socket
        if (server_sock == -1) {
            server_sock = socket_tcp_server_create(TCP_SERVER_PORT);
            if (server_sock < 0) {
                ESP_LOGE(TAG, "Failed to create TCP server, retry in 5s");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                continue;
            }
        }

        // 初始化 fd_set
        FD_ZERO(&read_fds);
        FD_SET(server_sock, &read_fds);
        max_fd = server_sock;

        // 添加所有已连接的客户端到 fd_set
        for (int i = 0; i < TCP_SERVER_MAX_CONN; i++) {
            if (client_socks[i] != -1) {
                FD_SET(client_socks[i], &read_fds);
                if (client_socks[i] > max_fd) {
                    max_fd = client_socks[i];
                }
            }
        }

        // 设置超时，避免永久阻塞
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);

        if (activity < 0) {
            if (errno == EINTR) {
                continue;  // 被信号中断，继续
            }
            ESP_LOGE(TAG, "select error: %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        if (activity == 0) {
            // 超时，没有活动，继续循环
            continue;
        }

        // 检查是否有新的客户端连接
        if (FD_ISSET(server_sock, &read_fds)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);

            if (new_sock < 0) {
                ESP_LOGE(TAG, "Accept failed: %d", errno);
            } else {
                char addr_str[16];
                inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(TAG, "New client connected: %s:%d (fd=%d)",
                         addr_str, ntohs(client_addr.sin_port), new_sock);

                // 找一个空位存放新客户端
                bool added = false;
                for (int i = 0; i < TCP_SERVER_MAX_CONN; i++) {
                    if (client_socks[i] == -1) {
                        client_socks[i] = new_sock;
                        client_count++;
                        added = true;
                        ESP_LOGI(TAG, "Client added to slot %d, total clients: %d", i, client_count);
                        break;
                    }
                }

                if (!added) {
                    ESP_LOGW(TAG, "Max clients reached (%d), rejecting new connection", TCP_SERVER_MAX_CONN);
                    const char *msg = "Server full, try again later\r\n";
                    send(new_sock, msg, strlen(msg), 0);
                    close(new_sock);
                }
            }
        }

        // 检查所有已连接客户端的数据
        for (int i = 0; i < TCP_SERVER_MAX_CONN; i++) {
            if (client_socks[i] != -1 && FD_ISSET(client_socks[i], &read_fds)) {
                if (!tcp_server_handle_client_data(client_socks[i])) {
                    // 客户端断开或发生错误
                    close(client_socks[i]);
                    ESP_LOGI(TAG, "Client removed from slot %d", i);
                    client_socks[i] = -1;
                    client_count--;
                    ESP_LOGI(TAG, "Remaining clients: %d", client_count);
                }
            }
        }
    }

    // 清理所有连接
    for (int i = 0; i < TCP_SERVER_MAX_CONN; i++) {
        if (client_socks[i] != -1) {
            close(client_socks[i]);
        }
    }
    if (server_sock != -1) {
        close(server_sock);
    }
    ESP_LOGI(TAG, "TCP server task is exit");
    vTaskDelete(NULL);
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
            g_sockfd = socket_tcp_client_create(TCP_CLIENT_IP, TCP_CLIENT_PORT);
            continue;
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS);

        size = asprintf(&data, "{\"src_addr\": \"" MACSTR "\",\"data\": \"Hello TCP Server!\",\"level\": %d,\"count\": %d}\r\n",
                        MAC2STR(sta_mac), esp_mesh_lite_get_level(), count++);

        ESP_LOGD(TAG, "TCP write, size: %d, data: %s", size, data);
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
        if (esp_mesh_lite_get_level() == 1) {
            xTaskCreate(tcp_server_write_task, "tcp_server_write_task", 4 * 1024, NULL, 5, NULL);
        } else {
            xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024, NULL, 5, NULL);
        }
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
    // Station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ROUTER_SSID,
            .password = CONFIG_ROUTER_PASSWORD,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    wifi_config_t wifi_softap_config = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
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
    /**
     * @brief Set the log level for serial port printing.
     */
    // esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();

    esp_mesh_lite_start();

    /**
     * @breif Create handler
     */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
