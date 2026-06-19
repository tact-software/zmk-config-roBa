/*
 * Trackball gesture input processor.
 *
 * Sits in a trackball input-listener's input-processors chain. While a
 * configured gesture layer is active, it consumes the trackball's relative
 * motion (so the cursor does not move and the auto-mouse temp-layer does not
 * trigger), resolves a forced 4-way direction, and emits OS-specific keyboard
 * shortcuts:
 *   - UP / DOWN  : single tap of the configured keycode.
 *   - LEFT/RIGHT : continuous mode. A modifier is held down for the duration of
 *                  the gesture while the configured key is tapped once per
 *                  movement step, and the modifier is released when the ball
 *                  stops (or when the gesture layer is deactivated). This keeps
 *                  Alt-Tab / Cmd-Tab / Ctrl-Tab style switcher UIs open.
 *
 * Every tunable (thresholds, timeouts, ratio, per-direction keycodes, held
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

struct tbg_config {
    uint8_t layer;         /* gesture layer this instance handles */
    uint8_t windows_layer; /* OS detection: active => Windows, else macOS */
    uint16_t start_threshold;
    uint16_t stop_timeout_ms;
    uint16_t direction_ratio_x10; /* fixed point: 20 == ratio 2.0 */
    uint16_t step_threshold;
    uint16_t quick_flick_ms;
    uint8_t max_steps;
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
    bool active;
    int32_t accum_dx;
    int32_t accum_dy;
    int32_t pend_dx;
    int32_t pend_dy;
    int64_t start_time;
    int64_t last_input_time;
    uint8_t emitted_steps;
    enum tbg_dir dir;
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
    data->active = false;
    data->accum_dx = 0;
    data->accum_dy = 0;
    data->pend_dx = 0;
    data->pend_dy = 0;
    data->emitted_steps = 0;
    data->dir = TBG_NONE;
}

static void tbg_emit_continuous(const struct device *dev, enum tbg_dir dir, int64_t now) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;
    uint8_t os = tbg_os_index(cfg);
    uint32_t mod = (dir == TBG_LEFT) ? cfg->left_mods[os] : cfg->right_mods[os];
    uint32_t key = (dir == TBG_LEFT) ? cfg->left_keys[os] : cfg->right_keys[os];

    tbg_mod_press(data, mod);

    int32_t primary = tbg_abs(data->accum_dx);
    int64_t duration = now - data->start_time;
    uint8_t target;
    if (duration < cfg->quick_flick_ms) {
        /* A fast flick is one step regardless of distance travelled. */
        target = 1;
    } else {
        uint32_t steps = cfg->step_threshold ? (primary / cfg->step_threshold) : 1;
        target = (uint8_t)CLAMP(steps, 1, cfg->max_steps);
    }

    while (data->emitted_steps < target) {
        tbg_tap(key);
        data->emitted_steps++;
    }
}

static void tbg_process(const struct device *dev) {
    const struct tbg_config *cfg = dev->config;
    struct tbg_data *data = dev->data;
    int64_t now = k_uptime_get();

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
    data->last_input_time = now;

    if (!data->active) {
        if (tbg_abs(data->accum_dx) + tbg_abs(data->accum_dy) > cfg->start_threshold) {
            data->active = true;
            data->start_time = now;
            data->emitted_steps = 0;
            data->dir = TBG_NONE;
        } else {
            return;
        }
    }

    enum tbg_dir dir = tbg_resolve(cfg, data->accum_dx, data->accum_dy);
    if (dir != TBG_NONE) {
        data->dir = dir;
    }

    if (data->dir == TBG_LEFT || data->dir == TBG_RIGHT) {
        tbg_emit_continuous(dev, data->dir, now);
    }

    k_work_reschedule(&data->stop_work, K_MSEC(cfg->stop_timeout_ms));
}

/* Trackball stopped: finalize single-shot directions and release held modifier. */
static void tbg_stop_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tbg_data *data = CONTAINER_OF(dwork, struct tbg_data, stop_work);
    const struct device *dev = data->dev;
    const struct tbg_config *cfg = dev->config;

    if (data->active) {
        uint8_t os = tbg_os_index(cfg);
        switch (data->dir) {
        case TBG_UP:
            tbg_tap(cfg->up_keys[os]);
            break;
        case TBG_DOWN:
            tbg_tap(cfg->down_keys[os]);
            break;
        case TBG_LEFT:
        case TBG_RIGHT:
            /* Continuous already emitted live; guarantee at least one step. */
            if (data->emitted_steps == 0) {
                tbg_tap(data->dir == TBG_LEFT ? cfg->left_keys[os] : cfg->right_keys[os]);
            }
            break;
        default:
            break;
        }
    }

    tbg_mod_release(data);
    tbg_reset(data);
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
        .quick_flick_ms = DT_INST_PROP(n, quick_flick_ms),                                         \
        .max_steps = DT_INST_PROP(n, max_steps),                                                   \
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

/* Safety: release any held modifier the instant a gesture layer turns off. */
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
            tbg_mod_release(data);
            tbg_reset(data);
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(trackball_gesture, tbg_layer_state_listener);
ZMK_SUBSCRIPTION(trackball_gesture, zmk_layer_state_changed);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
