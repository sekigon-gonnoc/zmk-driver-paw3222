/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 * Original source code:
 * https://github.com/zephyrproject-rtos/zephyr/blob/19c6240b6865bcb28e1d786d4dcadfb3a02067a0/include/zephyr/input/input_paw32xx.h
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_INPUT_PAW32XX_H_
#define ZEPHYR_INCLUDE_INPUT_PAW32XX_H_

#include <stdbool.h>
#include <stdint.h>

#define PAW32XX_ESB_FAST_POLL_LEAD_US 75

struct device;

/**
 * @brief Set resolution on a paw32xx device
 *
 * @param dev paw32xx device.
 * @param res_cpi CPI resolution, 200 to 3200.
 */
int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi);

/**
 * @brief Set force awake mode on a paw32xx device
 *
 * @param dev paw32xx device.
 * @param enable whether to enable or disable force awake mode.
 */
int paw32xx_force_awake(const struct device *dev, bool enable);

enum paw32xx_poll_route {
    PAW32XX_POLL_ROUTE_ESB,
    PAW32XX_POLL_ROUTE_BLE,
    PAW32XX_POLL_ROUTE_USB,
};

/** Set the polling interval used for one output route (1..65535 ms). */
int paw32xx_set_poll_interval(const struct device *dev, uint8_t route, uint16_t interval_ms);
/** Set the polling interval used for one output route in microseconds. */
int paw32xx_set_poll_interval_us(const struct device *dev, uint8_t route, uint32_t interval_us);

#endif /* ZEPHYR_INCLUDE_INPUT_PAW32XX_H_ */
