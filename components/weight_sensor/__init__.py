import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    UNIT_EMPTY,
    ICON_EMPTY,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor"]

weight_sensor_ns = cg.esphome_ns.namespace("weight_sensor")
WeightSensorComponent = weight_sensor_ns.class_(
    "WeightSensorComponent", cg.Component
)

CONF_DOUT_PIN = "dout_pin"
CONF_CLK_PIN = "clk_pin"
CONF_CALIBRATION_FACTOR = "calibration_factor"
CONF_SAMPLE_RATE_SPS = "sample_rate_sps"
CONF_WEIGHT_SENSOR = "weight_sensor_output"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WeightSensorComponent),
        cv.Required(CONF_DOUT_PIN): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_CLK_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_CALIBRATION_FACTOR, default=-7050.0): cv.float_,
        cv.Optional(CONF_SAMPLE_RATE_SPS, default=10): cv.int_range(min=1, max=80),
        cv.Optional(CONF_WEIGHT_SENSOR): sensor.sensor_schema(
            unit_of_measurement="g",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    dout = await cg.gpio_pin_expression(config[CONF_DOUT_PIN])
    cg.add(var.set_dout_pin(dout))

    clk = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    cg.add(var.set_clk_pin(clk))

    cg.add(var.set_calibration_factor(config[CONF_CALIBRATION_FACTOR]))
    cg.add(var.set_sample_rate_sps(config[CONF_SAMPLE_RATE_SPS]))

    if CONF_WEIGHT_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_WEIGHT_SENSOR])
        cg.add(var.set_weight_sensor(sens))
