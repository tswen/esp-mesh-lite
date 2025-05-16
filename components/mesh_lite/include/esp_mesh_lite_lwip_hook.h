/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "lwip/dhcp.h"
#include "lwip/netif.h"

int esp_mesh_lite_parse_options(struct dhcps_msg *msg, int16_t len, int16_t state);
#define LWIP_HOOK_DHCPS_POST_STATE(msg, len, state)\
    esp_mesh_lite_parse_options(msg, len, state)

err_t  esp_mesh_lite_dhcps_append_opts(struct netif *netif, uint8_t state, uint8_t **pp_opts);
#define LWIP_HOOK_DHCPS_POST_APPEND_OPTS(netif, dhcps, state, pp_opts) \
        esp_mesh_lite_dhcps_append_opts(netif, state, pp_opts);
