import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_MILLISECOND,
    DEVICE_CLASS_EMPTY,
)

DEPENDENCIES = ["esp32", "weight_sensor"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

grind_controller_ns = cg.esphome_ns.namespace("grind_controller")
GrindController = grind_controller_ns.class_("GrindController", cg.Component)

CONF_WEIGHT_SENSOR        = "weight_sensor"
CONF_MOTOR_PIN            = "motor_pin"
CONF_MOTOR_SETTLING_MS    = "motor_settling_time_ms"

CONF_CURRENT_WEIGHT       = "current_weight_sensor"
CONF_FLOW_RATE            = "flow_rate_sensor"
CONF_PHASE_NAME           = "phase_name_sensor"
CONF_PROGRESS             = "progress_sensor"
CONF_PULSE_COUNT          = "pulse_count_sensor"
CONF_LAST_ERROR           = "last_grind_error_sensor"
CONF_LAST_DURATION        = "last_grind_duration_sensor"
CONF_LAST_FINAL           = "last_grind_final_weight_sensor"
CONF_MECHANICAL           = "mechanical_anomaly_sensor"
CONF_IS_CALIBRATED        = "is_calibrated_sensor"
CONF_MOTOR_LATENCY        = "motor_latency_sensor"

weight_sensor_ns = cg.esphome_ns.namespace("weight_sensor")
WeightSensorComponent = weight_sensor_ns.class_("WeightSensorComponent")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GrindController),
        cv.Required(CONF_WEIGHT_SENSOR): cv.use_id(WeightSensorComponent),
        cv.Required(CONF_MOTOR_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_MOTOR_SETTLING_MS, default=500): cv.positive_int,

        # Sensor children — all optional
        cv.Optional(CONF_CURRENT_WEIGHT): sensor.sensor_schema(
            unit_of_measurement="g",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FLOW_RATE): sensor.sensor_schema(
            unit_of_measurement="g/s",
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PHASE_NAME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_PROGRESS): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PULSE_COUNT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LAST_ERROR): sensor.sensor_schema(
            unit_of_measurement="g",
            accuracy_decimals=2,
        ),
        cv.Optional(CONF_LAST_DURATION): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLISECOND,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_LAST_FINAL): sensor.sensor_schema(
            unit_of_measurement="g",
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_MECHANICAL): sensor.sensor_schema(
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_IS_CALIBRATED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MOTOR_LATENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLISECOND,
            accuracy_decimals=1,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ws = await cg.get_variable(config[CONF_WEIGHT_SENSOR])
    cg.add(var.set_weight_sensor(ws))

    motor = await cg.gpio_pin_expression(config[CONF_MOTOR_PIN])
    cg.add(var.set_motor_pin(motor))

    cg.add(var.set_motor_settling_time_ms(config[CONF_MOTOR_SETTLING_MS]))

    if CONF_CURRENT_WEIGHT in config:
        s = await sensor.new_sensor(config[CONF_CURRENT_WEIGHT])
        cg.add(var.set_current_weight_sensor(s))

    if CONF_FLOW_RATE in config:
        s = await sensor.new_sensor(config[CONF_FLOW_RATE])
        cg.add(var.set_flow_rate_sensor(s))

    if CONF_PHASE_NAME in config:
        s = await text_sensor.new_text_sensor(config[CONF_PHASE_NAME])
        cg.add(var.set_phase_name_sensor(s))

    if CONF_PROGRESS in config:
        s = await sensor.new_sensor(config[CONF_PROGRESS])
        cg.add(var.set_progress_sensor(s))

    if CONF_PULSE_COUNT in config:
        s = await sensor.new_sensor(config[CONF_PULSE_COUNT])
        cg.add(var.set_pulse_count_sensor(s))

    if CONF_LAST_ERROR in config:
        s = await sensor.new_sensor(config[CONF_LAST_ERROR])
        cg.add(var.set_last_grind_error_sensor(s))

    if CONF_LAST_DURATION in config:
        s = await sensor.new_sensor(config[CONF_LAST_DURATION])
        cg.add(var.set_last_grind_duration_sensor(s))

    if CONF_LAST_FINAL in config:
        s = await sensor.new_sensor(config[CONF_LAST_FINAL])
        cg.add(var.set_last_grind_final_weight_sensor(s))

    if CONF_MECHANICAL in config:
        s = await sensor.new_sensor(config[CONF_MECHANICAL])
        cg.add(var.set_mechanical_anomaly_sensor(s))

    if CONF_IS_CALIBRATED in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_IS_CALIBRATED])
        cg.add(var.set_is_calibrated_sensor(s))

    if CONF_MOTOR_LATENCY in config:
        s = await sensor.new_sensor(config[CONF_MOTOR_LATENCY])
        cg.add(var.set_motor_latency_sensor(s))
