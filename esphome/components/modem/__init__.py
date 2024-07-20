from esphome.const import (
    CONF_ID,
    CONF_USE_ADDRESS,
    CONF_USERNAME,
    CONF_PASSWORD,
    CONF_MODEL,
    CONF_TRIGGER_ID,
    CONF_ON_CONNECT,
    CONF_ON_DISCONNECT,
    CONF_ENABLE_ON_BOOT,
)
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.core import coroutine_with_priority
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
from esphome import automation
from esphome.components import uart

CODEOWNERS = ["@oarcher"]
DEPENDENCIES = ["esp32", "uart"]
AUTO_LOAD = ["network"]
# following should be removed if conflicts are resolved (so we can have a wifi ap using modem)
CONFLICTS_WITH = ["wifi", "captive_portal", "ethernet"]

CONF_PIN_CODE = "pin_code"
CONF_APN = "apn"
CONF_DTR_PIN = "dtr_pin"
CONF_INIT_AT = "init_at"
CONF_ON_NOT_RESPONDING = "on_not_responding"

modem_ns = cg.esphome_ns.namespace("modem")
ModemComponent = modem_ns.class_("ModemComponent", cg.Component, uart.UARTDevice)
ModemComponentState = modem_ns.enum("ModemComponentState")
ModemOnNotRespondingTrigger = modem_ns.class_(
    "ModemOnNotRespondingTrigger", automation.Trigger.template()
)
ModemOnConnectTrigger = modem_ns.class_(
    "ModemOnConnectTrigger", automation.Trigger.template()
)
ModemOnDisconnectTrigger = modem_ns.class_(
    "ModemOnDisconnectTrigger", automation.Trigger.template()
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModemComponent),
            cv.Required(CONF_MODEL): cv.string,
            cv.Required(CONF_APN): cv.string,
            cv.Optional(CONF_DTR_PIN): cv.positive_int,
            cv.Optional(CONF_PIN_CODE): cv.string_strict,
            cv.Optional(CONF_USERNAME): cv.string,
            cv.Optional(CONF_PASSWORD): cv.string,
            cv.Optional(CONF_USE_ADDRESS): cv.string,
            cv.Optional(CONF_INIT_AT): cv.All(cv.ensure_list(cv.string)),
            cv.Optional(CONF_ENABLE_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_ON_NOT_RESPONDING): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        ModemOnNotRespondingTrigger
                    )
                }
            ),
            cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ModemOnConnectTrigger)}
            ),
            cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        ModemOnDisconnectTrigger
                    )
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    cv.require_framework_version(
        esp_idf=cv.Version(4, 0, 0),  # 5.2.0 OK
    ),
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "modem", require_rx=True, require_tx=True
)


@coroutine_with_priority(60.0)
async def to_code(config):
    add_idf_component(
        name="esp_modem",
        repo="https://github.com/espressif/esp-protocols.git",
        ref="modem-v1.1.0",
        path="components/esp_modem",
    )

    add_idf_sdkconfig_option("CONFIG_LWIP_DHCPS", False)
    add_idf_sdkconfig_option("CONFIG_PPP", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_PPP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_PPP_PAP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_PPP_PAP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_ESP_MODEM_CMUX_DEFRAGMENT_PAYLOAD", True)
    add_idf_sdkconfig_option("CONFIG_ESP_MODEM_CMUX_DELAY_AFTER_DLCI_SETUP", 0)
    add_idf_sdkconfig_option("CONFIG_PPP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_PPP_NOTIFY_PHASE_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_PPP_CHAP_SUPPORT", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_PPP_VJ_HEADER_COMPRESSION", True)
    add_idf_sdkconfig_option("CONFIG_LWIP_PPP_NOTIFY_PHASE_SUPPORT", True)
    # commented because cause crash if another UART is defined in the yaml
    # If enabled, it should increase the reliability and the speed of the connection (TODO: test)
    # add_idf_sdkconfig_option("CONFIG_UART_ISR_IN_IRAM", True)

    cg.add_define("USE_MODEM")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if use_address := config.get(CONF_USE_ADDRESS, None):
        cg.add(var.set_use_address(use_address))

    if username := config.get(CONF_USERNAME, None):
        cg.add(var.set_username(username))

    if password := config.get(CONF_PASSWORD, None):
        cg.add(var.set_password(password))

    if pin_code := config.get(CONF_PIN_CODE, None):
        cg.add(var.set_pin_code(pin_code))

    if enable_on_boot := config.get(CONF_ENABLE_ON_BOOT, None):
        if enable_on_boot:
            cg.add(var.enable())

    if init_at := config.get(CONF_INIT_AT, None):
        for cmd in init_at:
            cg.add(var.add_init_at_command(cmd))

    cg.add(var.set_model(config[CONF_MODEL]))
    cg.add(var.set_apn(config[CONF_APN]))

    for conf in config.get(CONF_ON_NOT_RESPONDING, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_CONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_DISCONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
