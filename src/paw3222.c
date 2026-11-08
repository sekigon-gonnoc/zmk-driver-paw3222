/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 *
 * Original source code:
 * https://github.com/zephyrproject-rtos/zephyr/blob/19c6240b6865bcb28e1d786d4dcadfb3a02067a0/drivers/input/input_paw32xx.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zmk/event_manager.h>
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/endpoints.h>
#include <zmk/events/endpoint_changed.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
#include <zmk/nemu_link/control.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS)
#include "settings_schema_generated.h"
#include <zephyr/sys/byteorder.h>
#include <zmk/dynamic_settings/store.h>
#endif

#if defined(CONFIG_SOC_SERIES_NRF52X)
#include <hal/nrf_gpio.h>
#include <hal/nrf_spim.h>
#endif

#include "../include/paw3222.h"

LOG_MODULE_REGISTER(paw32xx, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT pixart_paw3222

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define PAW32XX_PRODUCT_ID1 0x00
#define PAW32XX_PRODUCT_ID2 0x01
#define PAW32XX_MOTION 0x02
#define PAW32XX_DELTA_X 0x03
#define PAW32XX_DELTA_Y 0x04
#define PAW32XX_OPERATION_MODE 0x05
#define PAW32XX_CONFIGURATION 0x06
#define PAW32XX_WRITE_PROTECT 0x09
#define PAW32XX_SLEEP1 0x0a
#define PAW32XX_SLEEP2 0x0b
#define PAW32XX_SLEEP3 0x0c
#define PAW32XX_CPI_X 0x0d
#define PAW32XX_CPI_Y 0x0e
#define PAW32XX_DELTA_XY_HI 0x12
#define PAW32XX_MOUSE_OPTION 0x19

#define PRODUCT_ID_PAW32XX 0x30
#define SPI_WRITE BIT(7)

#define MOTION_STATUS_MOTION BIT(7)
#define OPERATION_MODE_SLP_ENH BIT(4)
#define OPERATION_MODE_SLP2_ENH BIT(3)
#define OPERATION_MODE_SLP_MASK (OPERATION_MODE_SLP_ENH | OPERATION_MODE_SLP2_ENH)
#define CONFIGURATION_PD_ENH BIT(3)
#define CONFIGURATION_RESET BIT(7)
#define WRITE_PROTECT_ENABLE 0x00
#define WRITE_PROTECT_DISABLE 0x5a
#define MOUSE_OPTION_MOVX_INV_BIT 3
#define MOUSE_OPTION_MOVY_INV_BIT 4

#define PAW32XX_DATA_SIZE_BITS 8

#define RESET_DELAY_MS 2

#define RES_STEP 38
#define RES_MIN (16 * RES_STEP)
#define RES_MAX (127 * RES_STEP)
#define PAW32XX_FIXED_COMPENSATION_1MS_TICKS 2U

struct paw32xx_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec power_gpio;
    int16_t res_cpi;
    bool force_awake;
    uint32_t poll_interval_us[3];
};

struct paw32xx_data {
    const struct device *dev;
    struct k_mutex lock;
    struct k_work motion_work;
    struct k_work route_work;
    struct gpio_callback motion_cb;
    struct k_timer motion_timer;
    uint32_t poll_interval_us[3];
    bool fast_input_registered;
    bool suspended;
    atomic_t timer_expiration_tick;
    atomic_t timer_expiration_sequence;
    uint32_t last_timer_expiration_sequence;
    uint32_t timer_dispatch_average_ticks;
#if defined(CONFIG_SOC_SERIES_NRF52X)
    NRF_SPIM_Type *spim;
    uint32_t spim_mosi_psel;
    uint32_t spim_miso_psel;
    uint32_t spim_sclk_psel;
    bool spim_mosi_psel_saved;
    bool spim_miso_psel_saved;
#endif
};

static uint8_t paw32xx_poll_route(void) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
    return zmk_nemu_link_owns_radio() ? PAW32XX_POLL_ROUTE_ESB : PAW32XX_POLL_ROUTE_BLE;
#else
    return PAW32XX_POLL_ROUTE_BLE;
#endif
#else
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
    /* Sensor input may be routed through ESB while this device's local USB
     * endpoint remains selected. The split-link route takes precedence over
     * the endpoint used for locally generated reports. */
    if (zmk_nemu_link_owns_radio()) {
        return PAW32XX_POLL_ROUTE_ESB;
    }
#endif
    if (zmk_endpoint_get_selected().transport == ZMK_TRANSPORT_USB) {
        return PAW32XX_POLL_ROUTE_USB;
    }
    return PAW32XX_POLL_ROUTE_BLE;
#endif
}

static int paw32xx_sync_fast_input(const struct device *dev) {
    struct paw32xx_data *data = dev->data;
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
    k_mutex_lock(&data->lock, K_FOREVER);
    bool desired = !data->suspended && paw32xx_poll_route() == PAW32XX_POLL_ROUTE_ESB &&
                   data->poll_interval_us[PAW32XX_POLL_ROUTE_ESB] <= 2000U;
    bool registered = data->fast_input_registered;
    k_mutex_unlock(&data->lock);
    if (desired == registered) {
        return 0;
    }

    int err = zmk_nemu_link_fast_input_register(dev, desired);
    if (!err) {
        k_mutex_lock(&data->lock, K_FOREVER);
        data->fast_input_registered = desired;
        k_mutex_unlock(&data->lock);
    }
    return err;
#else
    ARG_UNUSED(data);
    return 0;
#endif
}

static void paw32xx_route_work_handler(struct k_work *work);

#define PAW32XX_GET_DEVICE(node_id) DEVICE_DT_GET(node_id),
static const struct device *paw32xx_devices[] = {
    DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, PAW32XX_GET_DEVICE)};

static void paw32xx_schedule_route_updates(void) {
    for (size_t i = 0; i < ARRAY_SIZE(paw32xx_devices); i++) {
        const struct device *dev = paw32xx_devices[i];
        if (device_is_ready(dev)) {
            k_work_submit(&((struct paw32xx_data *)dev->data)->route_work);
        }
    }
}

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
static bool paw32xx_mode_observer_registration_attempted;

static void paw32xx_nemu_link_mode_changed(bool esb, int error) {
    ARG_UNUSED(esb);
    ARG_UNUSED(error);
    paw32xx_schedule_route_updates();
}

static const struct zmk_nemu_link_observer paw32xx_nemu_link_observer = {
    .mode_changed = paw32xx_nemu_link_mode_changed,
};

static int paw32xx_register_mode_observer(void) {
    if (paw32xx_mode_observer_registration_attempted) {
        return 0;
    }
    paw32xx_mode_observer_registration_attempted = true;

    int err = zmk_nemu_link_register_observer(&paw32xx_nemu_link_observer);
    if (err == -EALREADY) {
        LOG_WRN("Nemu-link mode observer is already registered; route changes may "
                "not be observed");
        return 0;
    }
    if (err < 0) {
        LOG_ERR("Failed to register Nemu-link mode observer: %d", err);
    }
    return err;
}
#endif

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int paw32xx_endpoint_changed(const zmk_event_t *eh) {
    if (!as_zmk_endpoint_changed(eh)) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    paw32xx_schedule_route_updates();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(paw32xx_endpoint_changed, paw32xx_endpoint_changed);
ZMK_SUBSCRIPTION(paw32xx_endpoint_changed, zmk_endpoint_changed);
#endif

static void paw32xx_route_work_handler(struct k_work *work) {
    struct paw32xx_data *data = CONTAINER_OF(work, struct paw32xx_data, route_work);
    const struct device *dev = data->dev;
    int err = paw32xx_sync_fast_input(dev);
    if (err) {
        LOG_WRN("High-speed input registration failed: %d", err);
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    if (data->suspended) {
        k_mutex_unlock(&data->lock);
        return;
    }
    if (k_timer_remaining_get(&data->motion_timer) > 0) {
        uint32_t interval_us = data->poll_interval_us[paw32xx_poll_route()];
        k_timer_start(&data->motion_timer, K_USEC(interval_us), K_NO_WAIT);
    }
    k_mutex_unlock(&data->lock);
}

#define PAW32XX_NRF_PSEL_CONNECT_BIT BIT(31)
#define PAW32XX_NRF_PSEL_PIN_MASK GENMASK(4, 0)
#define PAW32XX_NRF_PSEL_PORT_BIT BIT(5)

static NRF_SPIM_Type *paw32xx_nrf52_spim_from_bus(const struct device *dev) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    const struct paw32xx_config *cfg = dev->config;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) && defined(NRF_SPIM0)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi0))) {
        return NRF_SPIM0;
    }
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && defined(NRF_SPIM1)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi1))) {
        return NRF_SPIM1;
    }
#endif
    return NULL;
#else
    ARG_UNUSED(dev);
    return NULL;
#endif
}

static void paw32xx_nrf52_spim_deactivate(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim != NULL) {
        nrf_spim_disable(data->spim);
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_nrf52_spim_activate(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim != NULL) {
        nrf_spim_enable(data->spim);
    }
#else
    ARG_UNUSED(data);
#endif
}

static uint32_t paw32xx_nrf52_psel_to_pin(uint32_t psel) {
    uint32_t pin = psel & PAW32XX_NRF_PSEL_PIN_MASK;

    if ((psel & PAW32XX_NRF_PSEL_PORT_BIT) != 0U) {
        pin += 32U;
    }

    return pin;
}

static void paw32xx_sdio_init(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    data->spim_mosi_psel = data->spim->PSEL.MOSI;
    data->spim_miso_psel = data->spim->PSEL.MISO;
    data->spim_sclk_psel = data->spim->PSEL.SCK;

    data->spim_mosi_psel_saved = ((data->spim_mosi_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    data->spim_miso_psel_saved = ((data->spim_miso_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
    }
    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel));
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_PULLUP);
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_sdio_disconnect(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    if (!data->spim_mosi_psel_saved) {
        data->spim_mosi_psel = data->spim->PSEL.MOSI;
        data->spim_mosi_psel_saved = ((data->spim_mosi_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    }
    if (!data->spim_miso_psel_saved) {
        data->spim_miso_psel = data->spim->PSEL.MISO;
        data->spim_miso_psel_saved = ((data->spim_miso_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    }

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
        data->spim->PSEL.MOSI = data->spim_mosi_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    }

    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_PULLUP);
        data->spim->PSEL.MISO = data->spim_miso_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_sdio_connect(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_output(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
        data->spim->PSEL.MOSI = data->spim_mosi_psel & ~PAW32XX_NRF_PSEL_CONNECT_BIT;
    }

    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_NOPULL);
        data->spim->PSEL.MISO = data->spim_miso_psel & ~PAW32XX_NRF_PSEL_CONNECT_BIT;
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_spi_transaction_begin(const struct device *dev) {
    struct paw32xx_data *data = dev->data;

    paw32xx_sdio_connect(data);
    paw32xx_nrf52_spim_activate(data);
}

static void paw32xx_spi_transaction_end(const struct device *dev) {
    struct paw32xx_data *data = dev->data;

    paw32xx_sdio_disconnect(data);
    paw32xx_nrf52_spim_deactivate(data);
}

static int paw32xx_force_cs(const struct device *dev, bool force_low) {
    const struct paw32xx_config *cfg = dev->config;
    const struct gpio_dt_spec *cs = NULL;
    int ret;

    if (cfg->spi.config.cs.gpio.port != NULL) {
        cs = &cfg->spi.config.cs.gpio;
    }

    if (cs == NULL || cs->port == NULL || !device_is_ready(cs->port)) {
        LOG_ERR("CS GPIO not defined or not ready");
        return ENODEV;
    }

    ret = gpio_pin_set_dt(cs, force_low ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to drive CS pin: %d", ret);
        return ret;
    }

    return 0;
}

// Define a custom sign_extend function to avoid conflict with Zephyr's
// implementation
static inline int32_t _sign_extend(uint32_t value, uint8_t index) {
    __ASSERT_NO_MSG(index <= 31);

    uint8_t shift = 31 - index;

    return (int32_t)(value << shift) >> shift;
}

static int paw32xx_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    const struct spi_buf tx_buf = {
        .buf = &addr,
        .len = sizeof(addr),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf[] = {
        {
            .buf = NULL,
            .len = sizeof(addr),
        },
        {
            .buf = value,
            .len = 1,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = ARRAY_SIZE(rx_buf),
    };

    paw32xx_spi_transaction_begin(dev);
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    paw32xx_spi_transaction_end(dev);

    return ret;
}

static int paw32xx_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    uint8_t write_buf[] = {addr | SPI_WRITE, value};
    const struct spi_buf tx_buf = {
        .buf = write_buf,
        .len = sizeof(write_buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    paw32xx_spi_transaction_begin(dev);
    ret = spi_write_dt(&cfg->spi, &tx);
    paw32xx_spi_transaction_end(dev);

    return ret;
}

static int paw32xx_update_reg(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t value) {
    uint8_t val;
    int ret;

    ret = paw32xx_read_reg(dev, addr, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~mask) | (value & mask);

    ret = paw32xx_write_reg(dev, addr, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int paw32xx_read_xy(const struct device *dev, int16_t *x, int16_t *y) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    uint8_t tx_data[] = {
        PAW32XX_DELTA_X,
        0xff,
        PAW32XX_DELTA_Y,
        0xff,
    };
    uint8_t rx_data[sizeof(tx_data)];

    const struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    paw32xx_spi_transaction_begin(dev);
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    paw32xx_spi_transaction_end(dev);
    if (ret < 0) {
        return ret;
    }

    *x = rx_data[1];
    *y = rx_data[3];

    *x = _sign_extend(*x, PAW32XX_DATA_SIZE_BITS - 1);
    *y = _sign_extend(*y, PAW32XX_DATA_SIZE_BITS - 1);

    return 0;
}

static int paw32xx_interrupt_configure(const struct device *dev, gpio_flags_t flags) {
    const struct paw32xx_config *cfg = dev->config;

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        return -ENODEV;
    }

    return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, flags);
}

static int paw32xx_interrupt_enable(const struct device *dev) {
    return paw32xx_interrupt_configure(dev, GPIO_INT_LEVEL_LOW);
}

static int paw32xx_interrupt_disable(const struct device *dev) {
    return paw32xx_interrupt_configure(dev, GPIO_INT_DISABLE);
}

static void paw32xx_motion_timer_handler(struct k_timer *timer) {
    struct paw32xx_data *data = CONTAINER_OF(timer, struct paw32xx_data, motion_timer);
    atomic_set(&data->timer_expiration_tick, (atomic_val_t)(uint32_t)k_uptime_ticks());
    atomic_inc(&data->timer_expiration_sequence);
    k_work_submit(&data->motion_work);
}

static void paw32xx_schedule_next_poll_locked(struct paw32xx_data *data, int64_t work_start_ticks,
                                              uint32_t interval_us) {
    int64_t elapsed_ticks = k_uptime_ticks() - work_start_ticks;
    int64_t interval_ticks = k_us_to_ticks_ceil64(interval_us);
    uint32_t max_compensation = interval_ticks > 1 ? (uint32_t)(interval_ticks - 1) : 0;
    uint32_t compensation_ticks = MIN(data->timer_dispatch_average_ticks, max_compensation);
    if (interval_us == 1000) {
        compensation_ticks = MIN(PAW32XX_FIXED_COMPENSATION_1MS_TICKS, max_compensation);
    }
    int64_t delay_ticks = interval_ticks - compensation_ticks - elapsed_ticks;
    k_timer_start(&data->motion_timer, K_TICKS(delay_ticks > 1 ? delay_ticks : 1), K_NO_WAIT);
}

static void paw32xx_motion_work_handler(struct k_work *work) {
    struct paw32xx_data *data = CONTAINER_OF(work, struct paw32xx_data, motion_work);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;
    uint8_t status;
    int16_t x, y;
    int ret;
    int64_t work_start_ticks;

    k_mutex_lock(&data->lock, K_FOREVER);
    work_start_ticks = k_uptime_ticks();
    if (data->suspended) {
        goto unlock;
    }

    uint32_t timer_sequence_before = (uint32_t)atomic_get(&data->timer_expiration_sequence);
    uint32_t timer_tick = (uint32_t)atomic_get(&data->timer_expiration_tick);
    uint32_t timer_sequence_after = (uint32_t)atomic_get(&data->timer_expiration_sequence);
    bool timer_triggered_work = timer_sequence_before == timer_sequence_after &&
                                timer_sequence_after != data->last_timer_expiration_sequence;
    if (timer_triggered_work) {
        uint32_t dispatch_ticks = (uint32_t)work_start_ticks - timer_tick;
        if (dispatch_ticks <= CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
            if (data->timer_dispatch_average_ticks == 0) {
                data->timer_dispatch_average_ticks = dispatch_ticks;
            } else {
                data->timer_dispatch_average_ticks =
                    (data->timer_dispatch_average_ticks * 3 + dispatch_ticks) / 4;
            }
        }
        data->last_timer_expiration_sequence = timer_sequence_after;
    }

    ret = paw32xx_read_reg(dev, PAW32XX_MOTION, &status);
    if (ret < 0) {
        LOG_ERR("Failed to read motion status: %d", ret);
        goto unlock;
    }

    bool poll_again = (status & MOTION_STATUS_MOTION) != 0;
    if (!poll_again) {
        paw32xx_interrupt_enable(dev);
        if (gpio_pin_get_dt(&cfg->irq_gpio) == 0) {
            goto unlock;
        }
    } else {
        uint8_t route = paw32xx_poll_route();
        paw32xx_schedule_next_poll_locked(data, work_start_ticks, data->poll_interval_us[route]);
    }

    ret = paw32xx_read_xy(dev, &x, &y);
    if (ret < 0) {
        LOG_ERR("Failed to read motion deltas: %d", ret);
        goto unlock;
    }

    k_mutex_unlock(&data->lock);

    int fast_input_err = paw32xx_sync_fast_input(dev);
    if (fast_input_err) {
        LOG_WRN("High-speed input registration failed: %d", fast_input_err);
    }
#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
    k_mutex_lock(&data->lock, K_FOREVER);
    bool fast_input_enabled = data->fast_input_registered;
    k_mutex_unlock(&data->lock);
    if (!fast_input_err && fast_input_enabled) {
        zmk_nemu_link_fast_input_activity(dev);
    }
#endif

    k_mutex_lock(&data->lock, K_FOREVER);
    if (data->suspended) {
        k_mutex_unlock(&data->lock);
        return;
    }

    int input_x_ret = input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
    int input_y_ret = input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
    if (input_x_ret < 0 || input_y_ret < 0) {
        LOG_ERR("Failed to report pointer movement: %d/%d", input_x_ret, input_y_ret);
    }

    k_mutex_unlock(&data->lock);
    return;

unlock:
    k_mutex_unlock(&data->lock);
}

int paw32xx_set_poll_interval_us(const struct device *dev, uint8_t route, uint32_t interval_us) {
    if (!dev || route > PAW32XX_POLL_ROUTE_USB || interval_us < 500 || interval_us > 65535000) {
        return -EINVAL;
    }
    struct paw32xx_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    data->poll_interval_us[route] = interval_us;
    if (!data->suspended && k_timer_remaining_get(&data->motion_timer) > 0) {
        uint32_t active_interval_us = data->poll_interval_us[paw32xx_poll_route()];
        k_timer_start(&data->motion_timer, K_USEC(active_interval_us), K_NO_WAIT);
    }
    k_mutex_unlock(&data->lock);
    return paw32xx_sync_fast_input(dev);
}

int paw32xx_set_poll_interval(const struct device *dev, uint8_t route, uint16_t interval_ms) {
    if (interval_ms == 0) {
        return -EINVAL;
    }
    return paw32xx_set_poll_interval_us(dev, route, (uint32_t)interval_ms * 1000U);
}

static void paw32xx_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb,
                                   uint32_t pins) {
    struct paw32xx_data *data = CONTAINER_OF(cb, struct paw32xx_data, motion_cb);
    const struct device *dev = data->dev;

    ARG_UNUSED(gpio_dev);
    ARG_UNUSED(pins);

    paw32xx_interrupt_disable(dev);
    k_timer_stop(&data->motion_timer);
    k_work_submit(&data->motion_work);
}

int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi) {
    struct paw32xx_data *data = dev->data;
    uint8_t val;
    int ret;

    if (!IN_RANGE(res_cpi, RES_MIN, RES_MAX)) {
        LOG_ERR("res_cpi out of range: %d", res_cpi);
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    val = res_cpi / RES_STEP;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        goto unlock;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_X, val);
    if (ret < 0) {
        goto unlock;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_Y, val);
    if (ret < 0) {
        goto unlock;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    if (ret < 0) {
        goto unlock;
    }

unlock:
    k_mutex_unlock(&data->lock);
    return ret;
}

static int paw32xx_force_awake_locked(const struct device *dev, bool enable) {
    uint8_t val = enable ? 0 : OPERATION_MODE_SLP_MASK;
    int ret;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_update_reg(dev, PAW32XX_OPERATION_MODE, OPERATION_MODE_SLP_MASK, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    return ret;
}

int paw32xx_force_awake(const struct device *dev, bool enable) {
    struct paw32xx_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    int ret = paw32xx_force_awake_locked(dev, enable);
    k_mutex_unlock(&data->lock);
    return ret;
}

static int paw32xx_configure(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int ret;
    int retry_count = 10;

    // Check if the device is ready
    while (retry_count--) {
        ret = paw32xx_read_reg(dev, PAW32XX_PRODUCT_ID1, &val);
        if (ret < 0) {
            if (retry_count == 0) {
                return ret;
            }
            k_sleep(K_MSEC(100)); // Wait before retrying
            continue;
        }

        if (val != PRODUCT_ID_PAW32XX) {
            LOG_ERR("Invalid product id: %02x", val);

            if (retry_count == 0) {
                return -ENODEV; // Device not ready after retries
            }
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
            // reboot
            ret = paw32xx_force_cs(dev, true);
            if (ret < 0) {
                return ret;
            }

            gpio_pin_set_dt(&cfg->power_gpio, 0);
            k_sleep(K_MSEC(50)); // Wait before retrying
            gpio_pin_set_dt(&cfg->power_gpio, 1);

            ret = paw32xx_force_cs(dev, false);
            if (ret < 0) {
                return ret;
            }
#endif
            k_sleep(K_MSEC(100)); // Wait before retrying
            continue;
        } else {
            break; // Device is ready
        }
    }

    ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_RESET, CONFIGURATION_RESET);
    if (ret < 0) {
        return ret;
    }

    k_sleep(K_MSEC(RESET_DELAY_MS));

    if (cfg->res_cpi > 0) {
        paw32xx_set_resolution(dev, cfg->res_cpi);
    }

    paw32xx_force_awake(dev, cfg->force_awake);

    // Dummy reads to clear any residual data
    paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_X, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_Y, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_XY_HI, &val);

    return 0;
}

static int paw32xx_init(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    int ret;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("%s is not ready", cfg->spi.bus->name);
        return -ENODEV;
    }

    data->dev = dev;
    k_mutex_init(&data->lock);
    memcpy(data->poll_interval_us, cfg->poll_interval_us, sizeof(data->poll_interval_us));
#if defined(CONFIG_SOC_SERIES_NRF52X)
    data->spim = paw32xx_nrf52_spim_from_bus(dev);
#endif

    paw32xx_sdio_init(data);
    paw32xx_sdio_disconnect(data);

    k_work_init(&data->motion_work, paw32xx_motion_work_handler);
    k_work_init(&data->route_work, paw32xx_route_work_handler);
    // Initialize the timer for delayed motion checks
    k_timer_init(&data->motion_timer, paw32xx_motion_timer_handler, NULL);

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    // Initialize power GPIO if defined
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        ret = paw32xx_force_cs(dev, true);
        if (ret != 0) {
            return ret;
        }

        // Configure as output but start with power OFF
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Power pin configuration failed: %d", ret);
            return ret;
        }

        // Wait 0.01 seconds before turning on power
        k_sleep(K_MSEC(10));

        // Now turn on power
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret != 0) {
            LOG_ERR("Power pin set failed: %d", ret);
            return ret;
        }

        // Wait for power stabilization
        k_sleep(K_MSEC(500));

        ret = paw32xx_force_cs(dev, false);
        if (ret != 0) {
            return ret;
        }

        // Wait for power stabilization
        k_sleep(K_MSEC(50));
    }
#endif

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        LOG_ERR("%s is not ready", cfg->irq_gpio.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Motion pin configuration failed: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->motion_cb, paw32xx_motion_handler, BIT(cfg->irq_gpio.pin));

    ret = gpio_add_callback_dt(&cfg->irq_gpio, &data->motion_cb);
    if (ret < 0) {
        LOG_ERR("Could not set motion callback: %d", ret);
        return ret;
    }

    ret = paw32xx_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }

#if IS_ENABLED(CONFIG_ZMK_DYNAMIC_SETTINGS)
    uint32_t cpi_setting_id = 0;
#ifdef DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_LEFT
    const struct dynamic_setting_group_descriptor *left_cpi_group =
        dynamic_settings_find_setting_group(DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_LEFT);
    if (left_cpi_group && left_cpi_group->context == dev) {
        cpi_setting_id = DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_LEFT;
    }
#endif
#ifdef DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_RIGHT
    const struct dynamic_setting_group_descriptor *right_cpi_group =
        dynamic_settings_find_setting_group(DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_RIGHT);
    if (right_cpi_group && right_cpi_group->context == dev) {
        cpi_setting_id = DYNAMIC_SETTING_ID_POINTER_SENSOR_CPI_RIGHT;
    }
#endif
    if (cpi_setting_id != 0U) {
        struct dynamic_setting_value initial_cpi;
        ret = dynamic_settings_read(cpi_setting_id, &initial_cpi, NULL);
        if (ret != 0) {
            LOG_ERR("Failed to read initial dynamic CPI setting: %d", ret);
            return ret;
        }
        if (initial_cpi.type != DYNAMIC_SETTING_TYPE_U16 ||
            initial_cpi.length != sizeof(uint16_t)) {
            LOG_ERR("Invalid initial dynamic CPI setting");
            return -EINVAL;
        }
        ret = paw32xx_set_resolution(dev, sys_get_le16(initial_cpi.data));
        if (ret != 0) {
            LOG_ERR("Failed to apply initial dynamic CPI setting: %d", ret);
            return ret;
        }
    }

    ret = dynamic_settings_apply_init_required(dev);
    if (ret != 0) {
        LOG_ERR("Failed to apply init-only settings: %d", ret);
        return ret;
    }
#endif

    ret = paw32xx_interrupt_enable(dev);
    if (ret != 0) {
        LOG_ERR("Motion interrupt configuration failed: %d", ret);
        return ret;
    }

    ret = pm_device_runtime_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable runtime power management: %d", ret);
        return ret;
    }

#if IS_ENABLED(CONFIG_ZMK_NEMU_LINK)
    ret = paw32xx_register_mode_observer();
    if (ret < 0) {
        return ret;
    }
    ret = paw32xx_sync_fast_input(dev);
    if (ret < 0) {
        LOG_ERR("Failed to synchronize Nemu-link fast input state: %d", ret);
    }
#endif

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int paw32xx_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    int ret;
    uint8_t val;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_mutex_lock(&data->lock, K_FOREVER);
        if (data->suspended) {
            k_mutex_unlock(&data->lock);
            return 0;
        }
        data->suspended = true;
        k_timer_stop(&data->motion_timer);
        ret = paw32xx_interrupt_disable(dev);
        k_mutex_unlock(&data->lock);
        if (ret < 0) {
            LOG_ERR("Failed to disable IRQ interrupt: %d", ret);
            return ret;
        }

        struct k_work_sync work_sync;
        k_work_cancel_sync(&data->motion_work, &work_sync);
        k_work_cancel_sync(&data->route_work, &work_sync);
        ret = paw32xx_sync_fast_input(dev);
        if (ret < 0) {
            LOG_ERR("Failed to unregister high-speed input during suspend: %d", ret);
        }

        // Disconnect IRQ GPIO
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_DISCONNECTED);
        if (ret < 0) {
            LOG_ERR("Failed to disconnect IRQ GPIO: %d", ret);
            return ret;
        }

        val = CONFIGURATION_PD_ENH;
        k_mutex_lock(&data->lock, K_FOREVER);
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        k_mutex_unlock(&data->lock);
        if (ret < 0) {
            return ret;
        }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        // Power off the device
        gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_INPUT | GPIO_PULL_DOWN);
#if defined(CONFIG_SOC_SERIES_NRF52X)
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_PULLDOWN);
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_sclk_psel), NRF_GPIO_PIN_PULLDOWN);
#endif
        gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
        gpio_pin_configure_dt(&cfg->power_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
#endif

        break;

    case PM_DEVICE_ACTION_RESUME:
        k_mutex_lock(&data->lock, K_FOREVER);
        if (!data->suspended) {
            k_mutex_unlock(&data->lock);
            return 0;
        }
        k_mutex_unlock(&data->lock);

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&cfg->power_gpio, 1);
        k_sleep(K_MSEC(500));

        ret = paw32xx_configure(dev);
        if (ret < 0) {
            return ret;
        }
#endif

        val = 0;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }

        // Reconfigure IRQ GPIO as input
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
            return ret;
        }

        k_mutex_lock(&data->lock, K_FOREVER);
        data->suspended = false;
        ret = paw32xx_interrupt_enable(dev);
        k_mutex_unlock(&data->lock);
        if (ret < 0) {
            LOG_ERR("Failed to enable IRQ interrupt: %d", ret);
            return ret;
        }

        ret = paw32xx_sync_fast_input(dev);
        if (ret < 0) {
            LOG_ERR("Failed to register high-speed input during resume: %d", ret);
        }
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif

#define PAW32XX_SPI_MODE                                                                           \
    (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PAW32XX_INIT(n)                                                                            \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP_OR(n, res_cpi, RES_MIN), RES_MIN, RES_MAX),                 \
                 "invalid res-cpi");                                                               \
    BUILD_ASSERT(DT_INST_PROP_OR(n, poll_interval_esb_ms, 1) > 0,                                  \
                 "poll-interval-esb-ms must be at least 1");                                       \
    BUILD_ASSERT(PAW32XX_ESB_FAST_POLL_LEAD_US < 1000U, "ESB poll lead must not underflow 1 ms");  \
                                                                                                   \
    static const struct paw32xx_config paw32xx_cfg_##n = {                                         \
        .spi = SPI_DT_SPEC_INST_GET(n, PAW32XX_SPI_MODE, 0),                                       \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                               \
        .res_cpi = DT_INST_PROP_OR(n, res_cpi, -1),                                                \
        .force_awake = DT_INST_PROP(n, force_awake),                                               \
        .poll_interval_us = {((uint32_t)DT_INST_PROP_OR(n, poll_interval_esb_ms, 1) * 1000U -      \
                              (DT_INST_PROP_OR(n, poll_interval_esb_ms, 1) <= 2                    \
                                   ? PAW32XX_ESB_FAST_POLL_LEAD_US                                 \
                                   : 0U)),                                                         \
                             DT_INST_PROP_OR(n, poll_interval_ble_ms, 15) * 1000U,                 \
                             DT_INST_PROP_OR(n, poll_interval_usb_ms, 1) * 1000U},                 \
    };                                                                                             \
                                                                                                   \
    static struct paw32xx_data paw32xx_data_##n;                                                   \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, paw32xx_pm_action);                                                \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, paw32xx_init, PM_DEVICE_DT_INST_GET(n), &paw32xx_data_##n,            \
                          &paw32xx_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PAW32XX_INIT)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
