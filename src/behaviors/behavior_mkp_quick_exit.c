/*
 * Mouse key press with delayed layer-deactivate on release.
 *
 * Mirrors the upstream zmk,behavior-mouse-key-press but additionally schedules
 * a delayed work that deactivates a configured layer some milliseconds after
 * the button is released. Re-presses cancel and reschedule the pending work so
 * double-clicks keep the layer alive until the user stops clicking.
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

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct mqe_config {
    uint8_t layer;
    uint32_t timeout_ms;
};

struct mqe_data {
    const struct device *dev;
    struct k_work_delayable layer_off_work;
};

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

    k_work_cancel_delayable(&data->layer_off_work);
    process_key_state(dev, binding->param1, true);
    return 0;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mqe_data *data = dev->data;
    const struct mqe_config *cfg = dev->config;

    process_key_state(dev, binding->param1, false);
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
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_mqe_init, NULL, &mqe_data_##n, &mqe_config_##n,            \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mqe_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MQE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
