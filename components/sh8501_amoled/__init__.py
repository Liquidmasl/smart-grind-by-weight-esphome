import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display
from esphome.const import CONF_ID, CONF_WIDTH, CONF_HEIGHT, CONF_RESET_PIN

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["display"]

sh8501_amoled_ns = cg.esphome_ns.namespace("sh8501_amoled")
SH8501AmoledDisplay = sh8501_amoled_ns.class_(
    "SH8501AmoledDisplay", display.DisplayBuffer
)

CONF_CS_PIN    = "cs_pin"
CONF_SCK_PIN   = "sck_pin"
CONF_D0_PIN    = "d0_pin"
CONF_D1_PIN    = "d1_pin"
CONF_D2_PIN    = "d2_pin"
CONF_D3_PIN    = "d3_pin"

CONFIG_SCHEMA = display.FULL_DISPLAY_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(SH8501AmoledDisplay),
        cv.Required(CONF_CS_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_SCK_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_D0_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_D1_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_D2_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_D3_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_RESET_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_WIDTH,  default=280): cv.int_range(min=1, max=1024),
        cv.Optional(CONF_HEIGHT, default=456): cv.int_range(min=1, max=1024),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await display.register_display(var, config)

    for conf_key, setter in (
        (CONF_CS_PIN,  "set_cs_pin"),
        (CONF_SCK_PIN, "set_sck_pin"),
        (CONF_D0_PIN,  "set_d0_pin"),
        (CONF_D1_PIN,  "set_d1_pin"),
        (CONF_D2_PIN,  "set_d2_pin"),
        (CONF_D3_PIN,  "set_d3_pin"),
    ):
        pin = await cg.gpio_pin_expression(config[conf_key])
        cg.add(getattr(var, setter)(pin))

    if CONF_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(pin))

    cg.add(var.set_width(config[CONF_WIDTH]))
    cg.add(var.set_height(config[CONF_HEIGHT]))
