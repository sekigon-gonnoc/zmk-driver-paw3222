/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zmk/dynamic_settings/store.h>

#include "../include/paw3222.h"

int paw3222_dynamic_cpi_changed(const struct dynamic_setting_descriptor *descriptor,
                                const struct dynamic_setting_value *value,
                                enum dynamic_setting_change_reason reason, const void *context) {
    ARG_UNUSED(descriptor);
    ARG_UNUSED(reason);

    if (value->type != DYNAMIC_SETTING_TYPE_U16 || value->length != sizeof(uint16_t)) {
        return -EINVAL;
    }

    const struct device *device = context;
    if (!device) {
        return -ENODEV;
    }

    return paw32xx_set_resolution(device, sys_get_le16(value->data));
}

static int set_poll_interval(const struct dynamic_setting_value *value, const void *context,
                             enum paw32xx_poll_route route) {
    if (value->type != DYNAMIC_SETTING_TYPE_U8 || value->length != sizeof(uint8_t)) {
        return -EINVAL;
    }
    uint8_t interval_ms = value->data[0];
    if (!interval_ms) {
        return -ERANGE;
    }
    return paw32xx_set_poll_interval(context, route, interval_ms);
}

int paw3222_dynamic_esb_interval_changed(const struct dynamic_setting_descriptor *descriptor,
                                         const struct dynamic_setting_value *value,
                                         enum dynamic_setting_change_reason reason,
                                         const void *context) {
    ARG_UNUSED(descriptor);
    ARG_UNUSED(reason);
    if (value->type != DYNAMIC_SETTING_TYPE_U8 || value->length != sizeof(uint8_t)) {
        return -EINVAL;
    }
    uint8_t interval_ms = value->data[0];
    if (!interval_ms) {
        return -ERANGE;
    }
    uint32_t interval_us = (uint32_t)interval_ms * 1000U;
    if (interval_ms <= 2) {
        interval_us -= PAW32XX_ESB_FAST_POLL_LEAD_US;
    }
    return paw32xx_set_poll_interval_us(context, PAW32XX_POLL_ROUTE_ESB, interval_us);
}

int paw3222_dynamic_ble_interval_changed(const struct dynamic_setting_descriptor *descriptor,
                                         const struct dynamic_setting_value *value,
                                         enum dynamic_setting_change_reason reason,
                                         const void *context) {
    ARG_UNUSED(descriptor);
    ARG_UNUSED(reason);
    return set_poll_interval(value, context, PAW32XX_POLL_ROUTE_BLE);
}

int paw3222_dynamic_usb_interval_changed(const struct dynamic_setting_descriptor *descriptor,
                                         const struct dynamic_setting_value *value,
                                         enum dynamic_setting_change_reason reason,
                                         const void *context) {
    ARG_UNUSED(descriptor);
    ARG_UNUSED(reason);
    return set_poll_interval(value, context, PAW32XX_POLL_ROUTE_USB);
}
