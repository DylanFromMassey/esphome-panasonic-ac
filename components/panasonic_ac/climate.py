from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_POWER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_WATT,
)
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, sensor, select, switch, binary_sensor

AUTO_LOAD = ["switch", "sensor", "select", "binary_sensor"]
DEPENDENCIES = ["uart"]

panasonic_ac_ns = cg.esphome_ns.namespace("panasonic_ac")
PanasonicAC = panasonic_ac_ns.class_(
    "PanasonicAC", cg.Component, uart.UARTDevice, climate.Climate
)
panasonic_ac_cnt_ns = panasonic_ac_ns.namespace("CNT")
PanasonicACCNT = panasonic_ac_cnt_ns.class_("PanasonicACCNT", PanasonicAC)
panasonic_ac_wlan_ns = panasonic_ac_ns.namespace("WLAN")
PanasonicACWLAN = panasonic_ac_wlan_ns.class_("PanasonicACWLAN", PanasonicAC)

PanasonicACSwitch = panasonic_ac_ns.class_(
    "PanasonicACSwitch", switch.Switch, cg.Component
)
PanasonicACSelect = panasonic_ac_ns.class_(
    "PanasonicACSelect", select.Select, cg.Component
)


CONF_HORIZONTAL_SWING_SELECT = "horizontal_swing_select"
CONF_VERTICAL_SWING_SELECT = "vertical_swing_select"
CONF_SWING_MODES = "swing_modes"

# "off" is always available; it is accepted in the list only so configs read naturally.
SWING_MODE_BITS = {"both": 0x01, "vertical": 0x02, "horizontal": 0x04}
CONF_OUTSIDE_TEMPERATURE = "outside_temperature"
CONF_OUTSIDE_TEMPERATURE_OFFSET = "outside_temperature_offset"
CONF_CURRENT_TEMPERATURE_SENSOR = "current_temperature_sensor"
CONF_CURRENT_TEMPERATURE_OFFSET = "current_temperature_offset"
CONF_NANOEX_SWITCH = "nanoex_switch"
CONF_NANOEX_STATUS = "nanoex_status"
CONF_IAUTOX_SWITCH = "iautox_switch"
CONF_IAUTOX_STATUS = "iautox_status"
CONF_ECO_SWITCH = "eco_switch"
CONF_ECONAVI_SWITCH = "econavi_switch"
CONF_MILD_DRY_SWITCH = "mild_dry_switch"
CONF_CURRENT_POWER_CONSUMPTION = "current_power_consumption"
CONF_DEFROST_SENSOR = "defrost_sensor"
CONF_WLAN = "wlan"
CONF_CNT = "cnt"

HORIZONTAL_SWING_OPTIONS = ["auto", "left", "left_center", "center", "right_center", "right"]

VERTICAL_SWING_OPTIONS = ["swing", "auto", "up", "up_center", "center", "down_center", "down"]

SWITCH_SCHEMA = switch.switch_schema(PanasonicACSwitch).extend(cv.COMPONENT_SCHEMA)

SELECT_SCHEMA = select.select_schema(PanasonicACSelect)

PANASONIC_COMMON_SCHEMA = {
    cv.Optional(CONF_HORIZONTAL_SWING_SELECT): SELECT_SCHEMA,
    cv.Optional(CONF_VERTICAL_SWING_SELECT): SELECT_SCHEMA,
    # Restrict which swing modes this unit advertises. Omit to keep all four
    # (existing behaviour); e.g. swing_modes: [off, both, horizontal] for a unit
    # whose vertical louver is adjusted by hand.
    cv.Optional(CONF_SWING_MODES): cv.All(
        cv.ensure_list(cv.one_of("off", "both", "vertical", "horizontal")),
        cv.Length(min=1),
    ),
    cv.Optional(CONF_OUTSIDE_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_DEFROST_SENSOR): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_NANOEX_SWITCH): SWITCH_SCHEMA,
    cv.Optional(CONF_NANOEX_STATUS): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_IAUTOX_SWITCH): SWITCH_SCHEMA,
    cv.Optional(CONF_IAUTOX_STATUS): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_OUTSIDE_TEMPERATURE_OFFSET): cv.int_range(min=-15, max=15),
    cv.Optional(CONF_CURRENT_TEMPERATURE_OFFSET): cv.int_range(min=-15, max=15),
}

PANASONIC_CNT_SCHEMA = {
    cv.Optional(CONF_ECO_SWITCH): SWITCH_SCHEMA,
    cv.Optional(CONF_ECONAVI_SWITCH): SWITCH_SCHEMA,
    cv.Optional(CONF_MILD_DRY_SWITCH): SWITCH_SCHEMA,
    cv.Optional(CONF_CURRENT_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_CURRENT_POWER_CONSUMPTION): sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_WLAN: climate.climate_schema(PanasonicACWLAN).extend(PANASONIC_COMMON_SCHEMA).extend(uart.UART_DEVICE_SCHEMA),
        CONF_CNT: climate.climate_schema(PanasonicACCNT).extend(PANASONIC_COMMON_SCHEMA).extend(PANASONIC_CNT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA),
    }
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_HORIZONTAL_SWING_SELECT in config:
        conf = config[CONF_HORIZONTAL_SWING_SELECT]
        swing_select = await select.new_select(conf, options=HORIZONTAL_SWING_OPTIONS)
        await cg.register_component(swing_select, conf)
        cg.add(var.set_horizontal_swing_select(swing_select))

    if CONF_VERTICAL_SWING_SELECT in config:
        conf = config[CONF_VERTICAL_SWING_SELECT]
        swing_select = await select.new_select(conf, options=VERTICAL_SWING_OPTIONS)
        await cg.register_component(swing_select, conf)
        cg.add(var.set_vertical_swing_select(swing_select))

    if CONF_SWING_MODES in config:
        mask = 0
        for mode in config[CONF_SWING_MODES]:
            mask |= SWING_MODE_BITS.get(mode, 0)
        cg.add(var.set_swing_mode_mask(mask))

    if CONF_OUTSIDE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTSIDE_TEMPERATURE])
        cg.add(var.set_outside_temperature_sensor(sens))

    if CONF_DEFROST_SENSOR in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DEFROST_SENSOR])
        cg.add(var.set_defrost_sensor(sens))

    if CONF_OUTSIDE_TEMPERATURE_OFFSET in config:
        cg.add(var.set_outside_temperature_offset(config[CONF_OUTSIDE_TEMPERATURE_OFFSET]))

    # Read-only mirrors of status keys the AC reports but does not accept writes for.
    for s in [CONF_NANOEX_STATUS, CONF_IAUTOX_STATUS]:
        if s in config:
            conf = config[s]
            bsens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(var, f"set_{s}")(bsens))

    for s in [CONF_ECO_SWITCH, CONF_NANOEX_SWITCH, CONF_IAUTOX_SWITCH, CONF_MILD_DRY_SWITCH, CONF_ECONAVI_SWITCH]:
        if s in config:
            conf = config[s]
            a_switch = await switch.new_switch(conf)
            await cg.register_component(a_switch, conf)
            cg.add(getattr(var, f"set_{s}")(a_switch))

    if CONF_CURRENT_TEMPERATURE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_CURRENT_TEMPERATURE_SENSOR])
        cg.add(var.set_current_temperature_sensor(sens))

    if CONF_CURRENT_TEMPERATURE_OFFSET in config:
        cg.add(var.set_current_temperature_offset(config[CONF_CURRENT_TEMPERATURE_OFFSET]))

    if CONF_CURRENT_POWER_CONSUMPTION in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT_POWER_CONSUMPTION])
        cg.add(var.set_current_power_consumption_sensor(sens))
