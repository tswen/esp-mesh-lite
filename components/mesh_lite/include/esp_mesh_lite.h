/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_mesh_lite_log.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_espnow.h"

#define ROOT    (1)

typedef enum {
    ESP_MESH_LITE_EVENT_NODE_JOIN = ESP_MESH_LITE_EVENT_OTA_MAX,
    ESP_MESH_LITE_EVENT_NODE_LEAVE,
    ESP_MESH_LITE_EVENT_NODE_CHANGE,
    ESP_MESH_LITE_EVENT_MAX,
} esp_mesh_lite_event_node_info_t;

/**
 * @brief Initialization Mesh-Lite.
 *
 */
void esp_mesh_lite_init(esp_mesh_lite_config_t* config);

/**
 * @brief Start Mesh-Lite.
 *
 */
void esp_mesh_lite_start(void);

/**
 * @brief Get the softap ssid from NVS
 *
 * @param[out]    softap_ssid: Pointer to the softap ssid.
 *
 * @param[inout]  size         A non-zero pointer to the variable holding the length of softap_ssid.
 *                             In case softap_ssid a zero, will be set to the length
 *                             required to hold the value. In case softap_ssid is not
 *                             zero, will be set to the actual length of the value
 *                             written. For nvs_get_str this includes zero terminator.
 *
 * @return
 *             - ESP_OK if the value was retrieved successfully
 *             - ESP_FAIL if there is an internal error; most likely due to corrupted
 */
esp_err_t esp_mesh_lite_get_softap_ssid_from_nvs(char* softap_ssid, size_t* size);

/**
 * @brief Get the softap password from NVS
 *
 * @param[out]    softap_psw: Pointer to the softap password.
 *
 * @param[inout]  size        A non-zero pointer to the variable holding the length of softap_psw.
 *                            In case softap_psw a zero, will be set to the length
 *                            required to hold the value. In case softap_psw is not
 *                            zero, will be set to the actual length of the value
 *                            written. For nvs_get_str this includes zero terminator.
 *
 * @return
 *             - ESP_OK if the value was retrieved successfully
 *             - ESP_FAIL if there is an internal error; most likely due to corrupted
 */
esp_err_t esp_mesh_lite_get_softap_psw_from_nvs(char* softap_psw, size_t* size);

/**
 * @brief Set the softap ssid to NVS
 *
 * @param[in]   softap_ssid: Pointer to the softap ssid buffer.
 *
 * @return
 *             - ESP_OK if the value was retrieved successfully
 *             - ESP_FAIL if there is an internal error; most likely due to corrupted
 */
esp_err_t esp_mesh_lite_set_softap_ssid_to_nvs(char* softap_ssid);

/**
 * @brief Set the softap password from NVS
 *
 * @param[in]   softap_psw: Pointer to the softap password buffer.
 *
 * @return
 *             - ESP_OK if the value was retrieved successfully
 *             - ESP_FAIL if there is an internal error; most likely due to corrupted
 */
esp_err_t esp_mesh_lite_set_softap_psw_to_nvs(char* softap_psw);

#ifdef __cplusplus
}
#endif
