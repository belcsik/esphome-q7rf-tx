import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import switch
from esphome.components import button
from esphome.const import (
    CONF_ID,
    CONF_CLK_PIN,
    CONF_MISO_PIN,
    CONF_MOSI_PIN,
    CONF_CS_PIN,
    CONF_NAME,
)
from esphome import pins

AUTO_LOAD = ["button", "switch"]

CONF_DEVICES = "devices"
CONF_DEVICE_ID = "device_id"
CONF_PAIR_BUTTON = "pair_button"
CONF_THERMOSTAT_SWITCH = "thermostat_switch"
CONF_RESEND_INTERVAL = "resend_interval"
CONF_WATCHDOG_INTERVAL = "watchdog_interval"

q7rf_tx_ns = cg.esphome_ns.namespace("q7rf_tx")
Q7RfTx = q7rf_tx_ns.class_("Q7RfTx", cg.Component)

Q7RfPiarButton = q7rf_tx_ns.class_("PairButton", button.Button)

Q7RfThermosttatSwitcth = q7rf_tx_ns.class_("ThermostatSwitch", switch.Switch)

CTDEVICE_SCHEMA = {
    cv.Required(CONF_NAME): cv.string_strict,
    cv.Required(CONF_DEVICE_ID): cv.hex_uint16_t,
    cv.Optional(CONF_PAIR_BUTTON, default={CONF_NAME: "pair"}): button.button_schema(
        Q7RfPiarButton
    ),
    cv.Optional(
        CONF_THERMOSTAT_SWITCH, default={CONF_NAME: "switch"}
    ): switch.switch_schema(Q7RfThermosttatSwitcth),
    cv.Optional(CONF_RESEND_INTERVAL, default=60000): cv.uint32_t,
    cv.Optional(
        CONF_WATCHDOG_INTERVAL,
        default=0,
    ): cv.uint32_t,
}


def ensure_device_list(value):
    device_schema = cv.ensure_list(CTDEVICE_SCHEMA)
    devices = device_schema(value)

    names = []
    device_ids = []

    for _, dev in enumerate(devices):
        names.append(dev[CONF_NAME])
        device_ids.append(dev[CONF_DEVICE_ID])

    if len(names) != len(set(names)):
        raise cv.Invalid("Device names must be unique")

    if len(device_ids) != len(set(device_ids)):
        raise cv.Invalid("Device ids must be unique")

    return devices


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Q7RfTx),
        cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_MISO_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_MOSI_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_CS_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_DEVICES): ensure_device_list,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(
        var.set_spi(
            config[CONF_CLK_PIN],
            config[CONF_MISO_PIN],
            config[CONF_MOSI_PIN],
            config[CONF_CS_PIN],
        )
    )
    for _, dev in enumerate(config[CONF_DEVICES]):
        dev[CONF_PAIR_BUTTON][CONF_NAME] = (
            dev[CONF_NAME] + "_" + dev[CONF_PAIR_BUTTON][CONF_NAME]
        )
        dev[CONF_THERMOSTAT_SWITCH][CONF_NAME] = (
            dev[CONF_NAME] + "_" + dev[CONF_THERMOSTAT_SWITCH][CONF_NAME]
        )
        b = await button.new_button(dev[CONF_PAIR_BUTTON])
        s = await switch.new_switch(dev[CONF_THERMOSTAT_SWITCH])
        cg.add(
            var.add_device(
                dev[CONF_DEVICE_ID],
                dev[CONF_RESEND_INTERVAL],
                dev[CONF_WATCHDOG_INTERVAL],
                b,
                s,
            )
        )
