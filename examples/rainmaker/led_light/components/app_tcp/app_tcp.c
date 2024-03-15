#include "inttypes.h"
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_mesh_lite.h"

#define PAYLOAD_LEN       (1456) /**< Max payload size(in bytes) */
#define UDP_PORT 7788

static char *TAG = "APP_TCP";

static bool tcp_create_flag = false;
static int udp_socket = -1;
static int tcp_socket = -1;
static uint8_t* udp_rx_buffer;
static uint32_t udp_rx_buffer_len;
static char tcp_server_ip[16] = "172.168.30.49";

static TaskHandle_t tcp_task_handle = NULL;

void delete_tcp_task(void)
{
    if (tcp_socket != -1) {
        // ESP_LOGE(TAG, "Shutting down tcp socket1");
        // shutdown(tcp_socket, 0);
        // ESP_LOGE(TAG, "Shutting down tcp socket2");
        close(tcp_socket);
        // ESP_LOGE(TAG, "Shutting down tcp socket3");
        tcp_socket = -1;
    }
    vTaskDelete(tcp_task_handle);
    tcp_create_flag = false;
}

static int socket_tcp_client_create(const char *ip, uint16_t port)
{
    ESP_LOGD(TAG, "Create a tcp client, ip: %s, port: %d", ip, port);

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
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE,  &iface, sizeof(struct ifreq)) != 0) {
        ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sockfd, iface.ifr_name);
    }

    ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        ESP_LOGD(TAG, "socket connect, ret: %d, ip: %s, port: %d",
                   ret, ip, port);
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

    ESP_LOGI(TAG, "TCP client write task is running");

    while (1) {
        if (esp_mesh_lite_get_level() != ROOT) {
            goto exit;
        }

        if (tcp_socket == -1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // printf("tcp_server_ip %s, len:%d\r\n", tcp_server_ip, strlen(tcp_server_ip));
            if (strlen(tcp_server_ip) >= 7) {
                tcp_socket = socket_tcp_client_create(tcp_server_ip, 8070);
                if (tcp_socket == -1) {
                    continue;
                } else {
                    count = 0;
                }
            } else {
                continue;
            }
        }

        uint8_t sta_mac[6]              = {0};
        wifi_ap_record_t ap_info        = {0};
        esp_wifi_sta_get_ap_info(&ap_info);
        esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

        size = asprintf(&data, "{\"self mac\": \"" MACSTR "\", \"level\": %d, \"count\": %d, \"node num\": %d, \"parent mac\": \"" MACSTR "\", parent rssi: %d, free heap: %"PRIu32"}\r\n",
                        MAC2STR(sta_mac), esp_mesh_lite_get_level(), count++, esp_mesh_lite_get_child_node_number() + 1, MAC2STR(ap_info.bssid), (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());

        printf("TCP write, size: %d, data: %s", size, data);
        ret = write(tcp_socket, data, size);
        if (data) {
            free(data);
            data = NULL;
        }

        if (ret <= 0) {
            ESP_LOGE(TAG, "<%s> TCP write", strerror(errno));
            close(tcp_socket);
            tcp_socket = -1;
            continue;
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

exit:
    ESP_LOGI(TAG, "TCP client write task is exit");
    if (data) {
        free(data);
        data = NULL;
    }
    delete_tcp_task();
}

static int app_udp_socket_bind(int sock, uint16_t port)
{
    int optval = 1;
    struct ifreq iface;
    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0x0, sizeof(sockaddr));
    memset(&iface, 0x0, sizeof(iface));

    sockaddr.sin_addr.s_addr = htonl(IPADDR_ANY);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);

    /* bind socket with the corresponding netif */
    esp_netif_get_netif_impl_name(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), iface.ifr_name);
    if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE,  &iface, sizeof(struct ifreq)) != 0) {
        ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sock, iface.ifr_name);
    }

    int err = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));
    assert(err == 0);

    err = bind(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Bind error %s", strerror(errno));
    } else {
        ESP_LOGI(TAG, "Bind Socket %d, port %d", sock, port);
    }

    return sock;
}

static void udp_server_task(void* param)
{
    int sock;
    fd_set rfds;
    int recv_len = 0;

    udp_rx_buffer_len = PAYLOAD_LEN + 1;
    udp_rx_buffer = (uint8_t*)calloc(1, udp_rx_buffer_len * sizeof(uint8_t));

    /* Create Socket for station */
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    assert(udp_socket >= 0);

    app_udp_socket_bind(udp_socket, UDP_PORT);

    while (1) {
        if (esp_mesh_lite_get_level() != ROOT) {
            goto exit;
        }

        FD_ZERO(&rfds);
        FD_SET(udp_socket, &rfds);
        ESP_LOGD(TAG, "free heap: %"PRIu32"",esp_get_free_heap_size());
        sock = select(udp_socket + 1, &rfds, NULL, NULL, NULL);

        if (sock < 0) {
            ESP_LOGE(TAG, "Select failed: errno %s", strerror(errno));
        } else if (sock == 0) {
            ESP_LOGD(TAG, "Timeout has been reached and nothing has been received");
        } else {
            struct sockaddr source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            recv_len = 0;

            if (FD_ISSET(udp_socket, &rfds)) {
                recv_len = recvfrom(udp_socket, udp_rx_buffer, udp_rx_buffer_len - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            }

            if (recv_len > 0) {
                udp_rx_buffer[recv_len] = '\0';
                printf("rx buffer is %s[end]\r\n", udp_rx_buffer);

                memcpy(tcp_server_ip, udp_rx_buffer, sizeof(tcp_server_ip));

                esp_err_t err = sendto(udp_socket, "OK", 3, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "[sock=%d]: sendto() has failed\n error=%d: %s", sock, errno, strerror(errno));
                }

                app_espnow_broadcast_ip_to_all_devices(tcp_server_ip);
            }
        }
    }

exit:
    if (udp_socket != -1) {
        ESP_LOGE(TAG, "Shutting down udp socket");
        // shutdown(udp_socket, 0);
        close(udp_socket);
        udp_socket = -1;
    }
    if (udp_rx_buffer) {
        free(udp_rx_buffer);
        udp_rx_buffer = NULL;
    }
    vTaskDelete(NULL);
}

void app_rewrite_tcp_server_ip(char *ip)
{
    memcpy(tcp_server_ip, ip, sizeof(tcp_server_ip));
}

void app_udp_server_create(void)
{
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
}

void app_tcp_client_create(void)
{
    xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024, NULL, 5, &tcp_task_handle);
}

esp_err_t app_node_info_report_task_create(void)
{
    esp_err_t ret = ESP_FAIL;

    if (!tcp_create_flag && (esp_mesh_lite_get_level() == 1)) {
        app_udp_server_create();
        app_tcp_client_create();
        tcp_create_flag = true;
        ret =  ESP_OK;
    }
    return ret;
}
