/*
 * Mouse key press with delayed layer-deactivate on release.
 *
 * Mirrors the upstream zmk,behavior-mouse-key-press but additionally schedules
 * a delayed work that deactivates a configured layer some milliseconds after
 * the button is released. Re-presses cancel and reschedule the pending work so
 * double-clicks keep the layer alive until the user stops clicking.
 *
 * param2 optionally carries a fallback keycode. When it is non-zero and
 * motion-grace-ms is non-zero, a press with no pointer activity (trackball
 * motion recorded by the motion-note input processor, or a previous click)
 * within the grace period sends the fallback keycode instead of the mouse
 * button, so typing shortly after mouse use produces letters again. The
 * fallback path leaves the layer state untouched: a locked (toggled) mouse
 * layer stays locked, and an auto-activated one expires on its own timeout.
 */

#define DT_DRV_COMPAT zmk_behavior_mkp_quick_exit

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#include <zmk/matrix.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

enum mqe_press_kind {
    MQE_PRESS_NONE = 0,
    MQE_PRESS_CLICK,
    MQE_PRESS_FALLBACK,
};

struct mqe_config {
    uint8_t layer;
    uint32_t timeout_ms;
    uint32_t motion_grace_ms;
};

struct mqe_data {
    const struct device *dev;
    struct k_work_delayable layer_off_work;
    /* Press decision per key position, so release matches even if the grace
     * window expires while the key is held. */
    uint8_t pressed_kind[ZMK_KEYMAP_LEN];
};

#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_MOTION_NOTE)
extern uint32_t roba_pointer_activity_ts_ms;

static void mqe_note_activity(void) { roba_pointer_activity_ts_ms = k_uptime_get_32(); }

static bool mqe_pointer_recent(uint32_t grace_ms) {
    /* Unsigned subtraction keeps the comparison wrap-safe. */
    return grace_ms == 0 || (k_uptime_get_32() - roba_pointer_activity_ts_ms) <= grace_ms;
}
#else
/* No motion source available: never fall back, keep plain click behavior. */
static void mqe_note_activity(void) {}

static bool mqe_pointer_recent(uint32_t grace_ms) {
    ARG_UNUSED(grace_ms);
    return true;
}
#endif

static void process_key_state(const struct device *dev, int32_t val, bool pressed) {
    for (int i = 0; i < ZMK_HID_MOUSE_NUM_BUTTONS; i++) {
        if (val & BIT(i)) {
            WRITE_BIT(val, i, 0);
            input_report_key(dev, INPUT_BTN_0 + i, pressed ? 1 : 0, val == 0, K_FOREVER);
        }
    }
}

static void layer_off_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mqe_data *data = CONTAINER_OF(dwork, struct mqe_data, layer_off_work);
    const struct mqe_config *cfg = data->dev->config;
    LOG_DBG("mqe deactivating layer %d", cfg->layer);
    zmk_keymap_layer_deactivate(cfg->layer);
}

static int behavior_mqe_init(const struct device *dev) {
    struct mqe_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->layer_off_work, layer_off_work_cb);
    return 0;
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mqe_data *data = dev->data;
    const struct mqe_config *cfg = dev->config;

    if (binding->param2 != 0 && !mqe_pointer_recent(cfg->motion_grace_ms)) {
        if (event.position < ZMK_KEYMAP_LEN) {
            data->pressed_kind[event.position] = MQE_PRESS_FALLBACK;
        }
        return raise_zmk_keycode_state_changed_from_encoded(binding->param2, true,
                                                            event.timestamp);
    }

    if (event.position < ZMK_KEYMAP_LEN) {
        data->pressed_kind[event.position] = MQE_PRESS_CLICK;
    }
    mqe_note_activity();
    k_work_cancel_delayable(&data->layer_off_work);
    process_key_state(dev, binding->param1, true);
    return 0;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mqe_data *data = dev->data;
    const struct mqe_config *cfg = dev->config;

    enum mqe_press_kind kind = MQE_PRESS_CLICK;
    if (event.position < ZMK_KEYMAP_LEN) {
        kind = data->pressed_kind[event.position];
        data->pressed_kind[event.position] = MQE_PRESS_NONE;
    }

    if (kind == MQE_PRESS_FALLBACK) {
        return raise_zmk_keycode_state_changed_from_encoded(binding->param2, false,
                                                            event.timestamp);
    }

    process_key_state(dev, binding->param1, false);
    mqe_note_activity();
    k_work_reschedule(&data->layer_off_work, K_MSEC(cfg->timeout_ms));
    return 0;
}

static const struct behavior_driver_api mqe_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define MQE_INST(n)                                                                                \
    static struct mqe_data mqe_data_##n;                                                           \
    static const struct mqe_config mqe_config_##n = {                                              \
        .layer = DT_INST_PROP(n, layer),                                                           \
        .timeout_ms = DT_INST_PROP(n, timeout_ms),                                                 \
        .motion_grace_ms = DT_INST_PROP(n, motion_grace_ms),                                       \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_mqe_init, NULL, &mqe_data_##n, &mqe_config_##n,            \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mqe_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MQE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
