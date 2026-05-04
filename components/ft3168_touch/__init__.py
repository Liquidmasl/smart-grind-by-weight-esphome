"""FT3168 capacitive-touch driver for the Waveshare ESP32-S3-Touch-AMOLED-1.64.

The board doesn't wire TP_INT or TP_RST to the ESP, so we have to poll over
I2C and the chip will NACK polls when no finger is touching. ESPHome's stock
ft5x06 component aborts at the first NACK; this component talks to the chip
through a dedicated IDF I2C master with `disable_ack_check` set, exactly like
the original PlatformIO firmware's touch_driver.cpp."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import touchscreen
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["touchscreen"]

ft3168_ns = cg.esphome_ns.namespace("ft3168_touch")
FT3168Touchscreen = ft3168_ns.class_(
    "FT3168Touchscreen", touchscreen.Touchscreen, cg.Component
)

CONF_SDA_PIN = "sda_pin"
CONF_SCL_PIN = "scl_pin"
CONF_ADDRESS = "address"
CONF_FREQUENCY = "frequency"

CONFIG_SCHEMA = touchscreen.TOUCHSCREEN_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(FT3168Touchscreen),
        cv.Required(CONF_SDA_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_SCL_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_ADDRESS, default=0x38): cv.i2c_address,
        cv.Optional(CONF_FREQUENCY, default="300kHz"): cv.frequency,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await touchscreen.register_touchscreen(var, config)

    cg.add(var.set_sda_pin(config[CONF_SDA_PIN]))
    cg.add(var.set_scl_pin(config[CONF_SCL_PIN]))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_frequency_hz(int(config[CONF_FREQUENCY])))
