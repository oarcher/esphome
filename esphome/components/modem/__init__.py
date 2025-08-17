from esphome import pins
import esphome.codegen as cg
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
import esphome.config_validation as cv
from esphome.const import CONF_DEBUG, CONF_ID, CONF_MODEL, CONF_RX_PIN, CONF_TX_PIN
from esphome.core import coroutine_with_priority

CODEOWNERS = ["@oarcher"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["network", "uart"]
CONFLICTS_WITH = ["captive_portal", "ethernet"]

CONF_MODEM = "modem"
CONF_APN = "apn"
CONF_PIN = "pin"
CONF_ENABLE_CMUX = "enable_cmux"

MODEM_MODELS = ["BG96", "SIM800", "SIM7000", "SIM7080", "SIM7600", "SIM7670", "GENERIC"]

CONF_APN = "apn"
CONF_PIN = "pin"
CONF_ENABLE_CMUX = "enable_cmux"
CONF_INIT_AT = "init_at"

modem_ns = cg.esphome_ns.namespace("modem")
ModemComponent = modem_ns.class_("ModemComponent", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModemComponent),
            cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_RX_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_MODEL): cv.one_of(*MODEM_MODELS, upper=True),
            cv.Required(CONF_APN): cv.string,
            cv.Optional(CONF_PIN): cv.string_strict,
            cv.Optional(CONF_INIT_AT): cv.All(cv.ensure_list(cv.string)),
            cv.Optional(CONF_ENABLE_CMUX, default=True): cv.boolean,
            cv.Optional(CONF_DEBUG, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.require_framework_version(
        esp_idf=cv.Version(4, 0, 0),  # 5.2.0 OK
    ),
)


@coroutine_with_priority(60.0)
async def to_code(config):
    add_idf_component(
        name="esp_modem",
        repo="https://github.com/espressif/esp-protocols.git",
        ref="modem-v1.4.0",
        path="components/esp_modem",
    )
    add_idf_sdkconfig_option("CONFIG_PPP", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_PPP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_ESP_MODEM_CMUX_DELAY_AFTER_DLCI_SETUP", 1500)
    add_idf_sdkconfig_option("CONFIG_PPP_SUPPORT", True)

    cg.add_define("USE_MODEM")

    var = cg.new_Pvariable(config[CONF_ID])

    if config[CONF_DEBUG]:
        add_idf_sdkconfig_option("CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE", True)
        cg.add(var.enable_debug())
    cg.add(var.set_apn(config[CONF_APN]))

    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))

    cg.add(var.set_model(config[CONF_MODEL]))

    if pin_code := config.get(CONF_PIN, None):
        cg.add(var.set_pin_code(pin_code))

    if config[CONF_ENABLE_CMUX]:
        cg.add(var.enable_cmux())

    await cg.register_component(var, config)
