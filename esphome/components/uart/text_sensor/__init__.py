import esphome.codegen as cg
from esphome.components import text_sensor, uart
import esphome.config_validation as cv

from .. import uart_ns

UARTTextSensor = uart_ns.class_(
    "UARTTextSensor", text_sensor.TextSensor, uart.UARTDevice, cg.Component
)

CONF_SEPARATOR = "separator"

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(UARTTextSensor)
    .extend(
        {
            cv.Optional(CONF_SEPARATOR, default="\r\n"): cv.string,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_separator(config[CONF_SEPARATOR]))
