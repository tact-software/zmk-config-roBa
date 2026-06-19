/*
 * Trackball gesture input processor.
 *
 * Sits in a trackball input-listener's input-processors chain. While a
 * configured gesture layer is active, it consumes the trackball's relative
 * motion (so the cursor does not move and the auto-mouse temp-layer does not
 * trigger), resolves a forced 4-way direction, and emits OS-specific keyboard
 * shortcuts:
 *
 *   - UP / DOWN (single): one tap of the configured keycode. Committed when the
 *     ball stops, or when the gesture layer is released. Repeatable while held.
 *
 *   - LEFT / RIGHT (continuous): an Alt-Tab / Cmd-Tab / Ctrl-Tab style switcher.
 *     The modifier is pressed when the gesture starts and is held for as long as
 *     the gesture layer is held. Rolling right taps "forward" (e.g. Tab), rolling
 *     left taps "backward" (e.g. Shift+Tab); the user can roll back and forth and
 *     the steps follow, with no upper bound. The modifier is released only when
 *     the gesture layer is released, committing the selection.
 *
 * Every tunable (thresholds, timeout, ratio, per-direction keycodes, held
 * modifiers, OS detection layer, axis orientation) is a devicetree property so
 * behaviour can be configured entirely from the keymap.
 */

#define DT_DRV_COMPAT zmk_input_processor_trackball_gesture

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>

#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>

LOG_MODULE_REGISTER(trackball_gesture, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Index into the per-direction <windows macos> arrays. */
#define TBG_OS_WIN 0
#define TBG_OS_MAC 1

enum tbg_dir { TBG_NONE = 0, TBG_UP, TBG_DOWN, TBG_LEFT, TBG_RIGHT };

enum tbg_mode { TBG_MODE_NONE = 0, TBG_MODE_SINGLE, TBG_MODE_CONTINUOUS };

struct tbg_config {
    uint8_t layer;         /* gesture layer this instance handles */
    uint8_t windows_layer; /* OS detection: active => Windows, else macOS */
    uint16_t start_threshold;
    uint16_t stop_timeout_ms;
    uint16_t direction_ratio_x10; /* fixed point: 20 == ratio 2.0 */
    uint16_t step_threshold;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    /* [TBG_OS_WIN] = Windows value, [TBG_OS_MAC] = macOS value */
    uint32_t up_keys[2];
    uint32_t down_keys[2];
    uint32_t left_keys[2];
    uint32_t right_keys[2];
    uint32_t left_mods[2];
    uint32_t right_mods[2];
};

struct tbg_data {
    const struct device *dev;
    struct k_work_delayable stop_work;
    enum tbg_mode mode;
    int32_t accum_dx;
    int32_t accum_dy;
    int32_t pend_dx;
    int32_t pend_dy;
    int32_t anchor_x; /* CONTINUOUS: accum_dx at the last emitted step boundary */
    uint32_t held_mod; /* 0 == no modifier currently held */
};

static inline uint8_t tbg_os_index(const struct tbg_config *cfg) {
    return zmk_keymap_layer_active(cfg->windows_layer) ? TBG_OS_WIN : TBG_OS_MAC;
}

static inline int32_t tbg_abs(int32_t v) { return v < 0 ? -v : v; }

static void tbg_tap(uint32_t encoded) {
    if (encoded == 0) {
        return;
    }
    int64_t t = k_uptime_get();
    raise_zmk_keycode_state_changed_from_encoded(encoded, true, t);
    raise_zmk_keycode_state_changed_from_encoded(encoded, false, t);
}

static void tbg_mod_press(struct tbg_data *data, uint32_t mod) {
    if (mod == 0 || data->held_mod != 0) {
        return;
    }
    raise_zmk_keycode_state_changed_from_encoded(mod, true, k_uptime_get());
    data->held_mod = mod;
}

static void tbg_mod_release(struct tbg_data *data) {
    if (data->held_mod == 0) {
        return;
    }
    raise_zmk_keycode_state_changed_from_encoded(data->held_mod, false, k_uptime_get());
    data->held_mod = 0;
}

static enum tbg_dir tbg_resolve(const struct tbg_config *cfg, int32_t dx, int32_t dy) {
    int64_t ax = tbg_abs(dx);
    int64_t ay = tbg_abs(dy);
    if (ax * 10 > ay * cfg->direction_ratio_x10) {
        return dx > 0 ? TBG_RIGHT : TBG_LEFT;
    }
    if (ay * 10 > ax * cfg->direction_ratio_x10) {
        return dy > 0 ? TBG_DOWN : TBG_UP;
    }
    return TBG_NONE;
}

static void tbg_reset(struct tbg_data *data) {
    data->mode = TBG_MODE_NONE;
    data->accum_dx = 0;
    data->accum_dy = 0;
    data->pend_dx = 0;
    data->pend_dy = 0;
    data->anchor_x = 0;
}

/* CONTINUOUS: emit forward/backward taps to follow net X travel, both ways. */
static void tbg_continuous_step(const struct device *dev) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;
    uint8_t os = tbg_os_index(cfg);
    uint32_t fwd = cfg->right_keys[os];
    uint32_t back = cfg->left_keys[os];
    int32_t thr = cfg->step_threshold > 0 ? cfg->step_threshold : 1;

    while (data->accum_dx - data->anchor_x >= thr) {
        tbg_tap(fwd);
        data->anchor_x += thr;
    }
    while (data->accum_dx - data->anchor_x <= -thr) {
        tbg_tap(back);
        data->anchor_x -= thr;
    }
}

/* SINGLE: commit one action based on the resolved up/down direction. */
static void tbg_fire_single(const struct device *dev) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;
    uint8_t os = tbg_os_index(cfg);
    enum tbg_dir dir = tbg_resolve(cfg, data->accum_dx, data->accum_dy);
    if (dir == TBG_UP) {
        tbg_tap(cfg->up_keys[os]);
    } else if (dir == TBG_DOWN) {
        tbg_tap(cfg->down_keys[os]);
    }
}

static void tbg_process(const struct device *dev) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;

    int32_t dx = data->pend_dx;
    int32_t dy = data->pend_dy;
    data->pend_dx = 0;
    data->pend_dy = 0;

    if (cfg->swap_xy) {
        int32_t tmp = dx;
        dx = dy;
        dy = tmp;
    }
    if (cfg->invert_x) {
        dx = -dx;
    }
    if (cfg->invert_y) {
        dy = -dy;
    }

    data->accum_dx += dx;
    data->accum_dy += dy;

    if (data->mode == TBG_MODE_NONE) {
        if (tbg_abs(data->accum_dx) + tbg_abs(data->accum_dy) <= cfg->start_threshold) {
            return;
        }
        enum tbg_dir dir = tbg_resolve(cfg, data->accum_dx, data->accum_dy);
        uint8_t os = tbg_os_index(cfg);
        if (dir == TBG_LEFT || dir == TBG_RIGHT) {
            data->mode = TBG_MODE_CONTINUOUS;
            k_work_cancel_delayable(&data->stop_work);
            tbg_mod_press(data, cfg->right_mods[os]);
            data->anchor_x = data->accum_dx;
            /* Emit the first step immediately so a quick flick switches once. */
            tbg_tap(dir == TBG_RIGHT ? cfg->right_keys[os] : cfg->left_keys[os]);
        } else if (dir == TBG_UP || dir == TBG_DOWN) {
            data->mode = TBG_MODE_SINGLE;
        } else {
            return; /* ambiguous, wait for more movement */
        }
    }

    if (data->mode == TBG_MODE_CONTINUOUS) {
        tbg_continuous_step(dev);
    } else if (data->mode == TBG_MODE_SINGLE) {
        /* Commit the single action once the ball stops. */
        k_work_reschedule(&data->stop_work, K_MSEC(cfg->stop_timeout_ms));
    }
}

static void tbg_stop_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tbg_data *data = CONTAINER_OF(dwork, struct tbg_data, stop_work);

    /* Only SINGLE gestures commit on stop; CONTINUOUS ends on layer release. */
    if (data->mode == TBG_MODE_SINGLE) {
        tbg_fire_single(data->dev);
        tbg_reset(data);
    }
}

static int tbg_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                            uint32_t param2, struct zmk_input_processor_state *state) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;

    if (!zmk_keymap_layer_active(cfg->layer)) {
        /* Not our gesture layer: leave motion for downstream processors. */
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type == INPUT_EV_REL) {
        switch (event->code) {
        case INPUT_REL_X:
            data->pend_dx += event->value;
            break;
        case INPUT_REL_Y:
            data->pend_dy += event->value;
            break;
        default:
            break;
        }
    }

    if (event->sync) {
        tbg_process(dev);
    }

    /* Consume the event so the cursor stays still and auto-mouse stays off. */
    return ZMK_INPUT_PROC_STOP;
}

static int tbg_init(const struct device *dev) {
    struct tbg_data *data = dev->data;
    data->dev = dev;
    data->held_mod = 0;
    k_work_init_delayable(&data->stop_work, tbg_stop_work_cb);
    tbg_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api tbg_driver_api = {
    .handle_event = tbg_handle_event,
};

#define TBG_INST(n)                                                                                \
    static struct tbg_data tbg_data_##n;                                                           \
    static const struct tbg_config tbg_config_##n = {                                              \
        .layer = DT_INST_PROP(n, layer),                                                           \
        .windows_layer = DT_INST_PROP(n, windows_layer),                                           \
        .start_threshold = DT_INST_PROP(n, start_threshold),                                       \
        .stop_timeout_ms = DT_INST_PROP(n, stop_timeout_ms),                                       \
        .direction_ratio_x10 = DT_INST_PROP(n, direction_ratio_x10),                               \
        .step_threshold = DT_INST_PROP(n, step_threshold),                                         \
        .invert_x = DT_INST_PROP(n, invert_x),                                                     \
        .invert_y = DT_INST_PROP(n, invert_y),                                                     \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                                       \
        .up_keys = DT_INST_PROP(n, up_keys),                                                       \
        .down_keys = DT_INST_PROP(n, down_keys),                                                   \
        .left_keys = DT_INST_PROP(n, left_keys),                                                   \
        .right_keys = DT_INST_PROP(n, right_keys),                                                 \
        .left_mods = DT_INST_PROP(n, left_mods),                                                   \
        .right_mods = DT_INST_PROP(n, right_mods),                                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, tbg_init, NULL, &tbg_data_##n, &tbg_config_##n, POST_KERNEL,          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &tbg_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TBG_INST)

/* Release the held modifier and reset the instant a gesture layer turns off. */
#define TBG_DEV(n) DEVICE_DT_INST_GET(n),
static const struct device *const tbg_devs[] = {DT_INST_FOREACH_STATUS_OKAY(TBG_DEV)};

static int tbg_layer_state_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL || ev->state) {
        /* Only react to deactivation. */
        return ZMK_EV_EVENT_BUBBLE;
    }
    for (size_t i = 0; i < ARRAY_SIZE(tbg_devs); i++) {
        const struct device *dev = tbg_devs[i];
        const struct tbg_config *cfg = dev->config;
        struct tbg_data *data = dev->data;
        if (cfg->layer == ev->layer) {
            k_work_cancel_delayable(&data->stop_work);
            /* Commit a single gesture that was released before the ball stopped. */
            if (data->mode == TBG_MODE_SINGLE) {
                tbg_fire_single(dev);
            }
            tbg_mod_release(data);
            tbg_reset(data);
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(trackball_gesture, tbg_layer_state_listener);
ZMK_SUBSCRIPTION(trackball_gesture, zmk_layer_state_changed);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
