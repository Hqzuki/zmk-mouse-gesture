/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_lock

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>

#include <drivers/input_processor.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

enum axis_lock_mode {
    AXIS_LOCK_NONE = 0,
    AXIS_LOCK_X,
    AXIS_LOCK_Y,
};

struct input_processor_axis_lock_config {
    uint32_t lock_threshold;
    uint32_t dominance_ratio_percent;
    uint32_t idle_timeout_ms;
};

struct input_processor_axis_lock_data {
    struct k_mutex lock;
    enum axis_lock_mode mode;
    int32_t acc_x;
    int32_t acc_y;
    int64_t last_event_time;
};

static void axis_lock_reset(struct input_processor_axis_lock_data *data) {
    data->mode = AXIS_LOCK_NONE;
    data->acc_x = 0;
    data->acc_y = 0;
}

static bool axis_lock_timed_out(const struct input_processor_axis_lock_config *config,
                                struct input_processor_axis_lock_data *data,
                                int64_t now) {
    if (config->idle_timeout_ms == 0) {
        return false;
    }

    if (data->last_event_time == 0) {
        return false;
    }

    return (now - data->last_event_time) > config->idle_timeout_ms;
}

static void axis_lock_accumulate(struct input_processor_axis_lock_data *data,
                                 uint16_t code,
                                 int32_t value) {
    if (code == INPUT_REL_X) {
        data->acc_x += value;
    } else if (code == INPUT_REL_Y) {
        data->acc_y += value;
    }
}

static void axis_lock_try_lock(const struct input_processor_axis_lock_config *config,
                               struct input_processor_axis_lock_data *data) {
    if (data->mode != AXIS_LOCK_NONE) {
        return;
    }

    int32_t abs_x = abs(data->acc_x);
    int32_t abs_y = abs(data->acc_y);
    uint32_t total = abs_x + abs_y;

    if (total < config->lock_threshold) {
        return;
    }

    /*
     * dominance_ratio_percent example:
     * 100 -> equal or greater
     * 140 -> 1.4x stronger required
     * 150 -> 1.5x stronger required
     */
    if (abs_x > 0 &&
        ((uint64_t)abs_x * 100U) >= ((uint64_t)abs_y * config->dominance_ratio_percent)) {
        data->mode = AXIS_LOCK_X;
        LOG_DBG("Axis lock engaged: X (acc_x=%d acc_y=%d)", data->acc_x, data->acc_y);
        return;
    }

    if (abs_y > 0 &&
        ((uint64_t)abs_y * 100U) >= ((uint64_t)abs_x * config->dominance_ratio_percent)) {
        data->mode = AXIS_LOCK_Y;
        LOG_DBG("Axis lock engaged: Y (acc_x=%d acc_y=%d)", data->acc_x, data->acc_y);
        return;
    }
}

static int input_processor_axis_lock_handle_event(const struct device *dev,
                                                  struct input_event *event,
                                                  uint32_t param1,
                                                  uint32_t param2,
                                                  struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    if (!(event->type == INPUT_EV_REL &&
          (event->code == INPUT_REL_X || event->code == INPUT_REL_Y))) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const struct input_processor_axis_lock_config *config = dev->config;
    struct input_processor_axis_lock_data *data = dev->data;
    int64_t now = k_uptime_get();

    if (k_mutex_lock(&data->lock, K_MSEC(5)) != 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (axis_lock_timed_out(config, data, now)) {
        LOG_DBG("Axis lock timeout, resetting");
        axis_lock_reset(data);
    }

    axis_lock_accumulate(data, event->code, event->value);
    axis_lock_try_lock(config, data);

    if (data->mode == AXIS_LOCK_X && event->code == INPUT_REL_Y) {
        event->value = 0;
    } else if (data->mode == AXIS_LOCK_Y && event->code == INPUT_REL_X) {
        event->value = 0;
    }

    data->last_event_time = now;

    k_mutex_unlock(&data->lock);
    return ZMK_INPUT_PROC_CONTINUE;
}

static int input_processor_axis_lock_init(const struct device *dev) {
    struct input_processor_axis_lock_data *data = dev->data;

    k_mutex_init(&data->lock);
    axis_lock_reset(data);
    data->last_event_time = 0;

    LOG_INF("Axis lock input processor initialized");
    return 0;
}

static const struct zmk_input_processor_driver_api input_processor_axis_lock_driver_api = {
    .handle_event = input_processor_axis_lock_handle_event,
};

#define AXIS_LOCK_INPUT_PROCESSOR_INST(n)                                        \
    static struct input_processor_axis_lock_data                                 \
        input_processor_axis_lock_data_##n = {};                                 \
    static const struct input_processor_axis_lock_config                         \
        input_processor_axis_lock_config_##n = {                                 \
            .lock_threshold = DT_INST_PROP_OR(n, lock_threshold, 24),            \
            .dominance_ratio_percent = DT_INST_PROP_OR(n, dominance_ratio_percent, 140), \
            .idle_timeout_ms = DT_INST_PROP_OR(n, idle_timeout_ms, 80),          \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(n, input_processor_axis_lock_init, NULL,               \
                          &input_processor_axis_lock_data_##n,                   \
                          &input_processor_axis_lock_config_##n, POST_KERNEL,    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                          &input_processor_axis_lock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_LOCK_INPUT_PROCESSOR_INST)

#endif
