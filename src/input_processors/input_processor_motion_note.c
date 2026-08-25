/*
 * Pass-through input processor that records the uptime of the last relative
 * pointer event (motion or wheel). It never modifies or consumes events.
 *
 * Placed after the gesture processors and before the auto-mouse temp-layer in
 * the trackball listener chain, so motion consumed by a gesture does not count
 * as pointer activity. The timestamp is consumed by behavior_mkp_quick_exit to
 * gate clicks on recent pointer use.
 */

#define DT_DRV_COMPAT zmk_input_processor_motion_note

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

#include <drivers/input_processor.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Shared with behavior_mkp_quick_exit; 32-bit so loads/stores stay atomic. */
uint32_t roba_pointer_activity_ts_ms;

static int mn_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {
    if (event->type == INPUT_EV_REL) {
        roba_pointer_activity_ts_ms = k_uptime_get_32();
    }
    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api mn_driver_api = {
    .handle_event = mn_handle_event,
};

static int mn_init(const struct device *dev) { return 0; }

#define MN_INST(n)                                                                                 \
    DEVICE_DT_INST_DEFINE(n, mn_init, NULL, NULL, NULL, POST_KERNEL,                               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mn_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MN_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
