
#include <errno.h>

#include <zmk/stdlib.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/central.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/pointing/input_split.h>

#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// TODO: Active transport selection

struct zmk_split_transport_central *active_transport;

int zmk_split_transport_central_peripheral_event_handler(
    const struct zmk_split_transport_central *transport, uint8_t source,
    struct zmk_split_transport_peripheral_event ev) {
    if (transport != active_transport) {
        // Ignoring events from non-active transport
        LOG_WRN("Ignoring peripheral event from non-active transport");
        return -EINVAL;
    }
    switch (ev.type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION: {
        struct zmk_position_state_changed state_ev = {.source = source,
                                                      .position = ev.data.key_position.position,
                                                      .state = ev.data.key_position.pressed,
                                                      .timestamp = k_uptime_get()};
        return raise_zmk_position_state_changed(state_ev);
    }
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT: {
        return zmk_input_split_report_peripheral_event(
            ev.data.input_event.reg, ev.data.input_event.type, ev.data.input_event.code,
            ev.data.input_event.value, ev.data.input_event.sync);
    }
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT: {
        struct zmk_sensor_event sensor_ev = {.sensor_index = ev.data.sensor_event.sensor_index,
                                             .channel_data_size = 1,
                                             .timestamp = ev.data.sensor_event.timestamp};

        sensor_ev.channel_data[0] = ev.data.sensor_event.channel_data;

        return raise_zmk_sensor_event(sensor_ev);
    }
    default:
        LOG_WRN("GOT AN UNKNOWN EVENT TYPE %d", ev.type);
        return -ENOTSUP;
    }
}

int zmk_split_cental_invoke_behavior(uint8_t source, struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event, bool state) {
    if (!active_transport || !active_transport->api || !active_transport->api->send_command) {
        return -ENODEV;
    }

    struct zmk_split_transport_central_command command =
        (struct zmk_split_transport_central_command){
            .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_TRIGGER_BEHAVIOR,
            .data =
                {
                    .trigger_behavior =
                        {
                            .param1 = binding->param1,
                            .param2 = binding->param2,
                            .position = event.position,
                            .event_source = event.source,
                            .state = state ? 1 : 0,
                        },
                },
        };

    const size_t payload_dev_size = sizeof(command.data.trigger_behavior.behavior_dev);
    if (strlcpy(command.data.trigger_behavior.behavior_dev, binding->behavior_dev,
                payload_dev_size) >= payload_dev_size) {
        LOG_ERR("Truncated behavior label %s to %s before invoking peripheral behavior",
                binding->behavior_dev, command.data.trigger_behavior.behavior_dev);
    }

    return active_transport->api->send_command(source, command);
};

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

int zmk_split_central_update_hid_indicator(zmk_hid_indicators_t indicators) {
    if (!active_transport || !active_transport->api ||
        !active_transport->api->get_available_source_ids || !active_transport->api->send_command) {
        return -ENODEV;
    }

    uint8_t source_ids[ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT];

    int ret = active_transport->api->get_available_source_ids(source_ids);

    if (ret < 0) {
        return ret;
    }

    struct zmk_split_transport_central_command command =
        (struct zmk_split_transport_central_command){
            .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS,
            .data =
                {
                    .set_hid_indicators =
                        {
                            .indicators = indicators,
                        },
                },
        };

    for (size_t i = 0; i < ret; i++) {
        ret = active_transport->api->send_command(source_ids[i], command);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS)

static int central_init(void) {
    STRUCT_SECTION_GET(zmk_split_transport_central, 0, &active_transport);

    return 0;
}

SYS_INIT(central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);