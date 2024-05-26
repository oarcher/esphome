#ifdef USE_ESP_IDF
#include "modem_component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/components/network/util.h"
#include <esp_netif.h>
#include <esp_netif_ppp.h>
#include <esp_event.h>
#include <cxx_include/esp_modem_dte.hpp>
#include <esp_modem_config.h>
#include <cxx_include/esp_modem_api.hpp>
#include <driver/gpio.h>
#include <lwip/dns.h>
#include <cstring>
#include <iostream>

static const size_t CONFIG_MODEM_UART_RX_BUFFER_SIZE = 2048;
static const size_t CONFIG_MODEM_UART_TX_BUFFER_SIZE = 1024;
static const uint8_t CONFIG_MODEM_UART_EVENT_QUEUE_SIZE = 30;
static const size_t CONFIG_MODEM_UART_EVENT_TASK_STACK_SIZE = 2048;
static const uint8_t CONFIG_MODEM_UART_EVENT_TASK_PRIORITY = 5;

namespace esphome {
namespace modem {

using namespace esp_modem;

ModemComponent *global_modem_component;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

#define ESPHL_ERROR_CHECK(err, message) \
  if ((err) != ESP_OK) { \
    ESP_LOGE(TAG, message ": (%d) %s", err, esp_err_to_name(err)); \
    this->mark_failed(); \
    return; \
  }

using namespace esp_modem;

ModemComponent::ModemComponent() { global_modem_component = this; }

void ModemComponent::dump_config() { ESP_LOGCONFIG(TAG, "Config Modem:"); }

float ModemComponent::get_setup_priority() const { return setup_priority::WIFI; }

bool ModemComponent::can_proceed() { return this->is_connected(); }

network::IPAddresses ModemComponent::get_ip_addresses() {
  network::IPAddresses addresses;
  esp_netif_ip_info_t ip;
  ESP_LOGV(TAG, "get_ip_addresses");
  esp_err_t err = esp_netif_get_ip_info(this->ppp_netif_, &ip);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "esp_netif_get_ip_info failed: %s", esp_err_to_name(err));
    // TODO: do something smarter
    // return false;
  } else {
    addresses[0] = network::IPAddress(&ip.ip);
  }
  return addresses;
}

std::string ModemComponent::get_use_address() const {
  if (this->use_address_.empty()) {
    return App.get_name() + ".local";
  }
  return this->use_address_;
}

void ModemComponent::set_use_address(const std::string &use_address) { this->use_address_ = use_address; }

bool ModemComponent::is_connected() { return this->state_ == ModemComponentState::CONNECTED; }

void ModemComponent::setup() {
  ESP_LOGI(TAG, "Setting up Modem...");

  ESP_LOGV(TAG, "DTE setup");
  esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
  this->dte_config_ = dte_config;

  this->dte_config_.uart_config.tx_io_num = this->tx_pin_;
  this->dte_config_.uart_config.rx_io_num = this->rx_pin_;
  // this->dte_config_.uart_config.rts_io_num =  static_cast<gpio_num_t>( CONFIG_EXAMPLE_MODEM_UART_RTS_PIN);
  // this->dte_config_.uart_config.cts_io_num =  static_cast<gpio_num_t>( CONFIG_EXAMPLE_MODEM_UART_CTS_PIN);
  this->dte_config_.uart_config.rx_buffer_size = CONFIG_MODEM_UART_RX_BUFFER_SIZE;
  this->dte_config_.uart_config.tx_buffer_size = CONFIG_MODEM_UART_TX_BUFFER_SIZE;
  this->dte_config_.uart_config.event_queue_size = CONFIG_MODEM_UART_EVENT_QUEUE_SIZE;
  this->dte_config_.task_stack_size = CONFIG_MODEM_UART_EVENT_TASK_STACK_SIZE * 2;
  this->dte_config_.task_priority = CONFIG_MODEM_UART_EVENT_TASK_PRIORITY;
  this->dte_config_.dte_buffer_size = CONFIG_MODEM_UART_RX_BUFFER_SIZE / 2;

  this->dte_ = create_uart_dte(&this->dte_config_);

  assert(this->dte_);

  ESP_LOGV(TAG, "Set APN: %s", this->apn_.c_str());
  esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(this->apn_.c_str());

  ESP_LOGV(TAG, "PPP netif setup");
  esp_err_t err;
  err = esp_netif_init();
  ESPHL_ERROR_CHECK(err, "PPP netif init error");
  err = esp_event_loop_create_default();
  ESPHL_ERROR_CHECK(err, "PPP event loop init error");
  esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();

  this->ppp_netif_ = esp_netif_new(&netif_ppp_config);
  assert(this->ppp_netif_);
  if (!this->username_.empty()) {
    ESP_LOGV(TAG, "Set auth: username: %s password: %s", this->username_.c_str(), this->password_.c_str());
    ESPHL_ERROR_CHECK(esp_netif_ppp_set_auth(this->ppp_netif_, NETIF_PPP_AUTHTYPE_PAP, this->username_.c_str(),
                                             this->password_.c_str()),
                      "ppp set auth");
  }
  // dns setup not needed (perhaps fallback ?)
  // esp_netif_dns_info_t dns_main = {};
  // dns_main.ip.u_addr.ip4.addr = esp_ip4addr_aton("8.8.8.8");
  // dns_main.ip.type = ESP_IPADDR_TYPE_V4;
  // ESPHL_ERROR_CHECK(esp_netif_set_dns_info(this->ppp_netif_, ESP_NETIF_DNS_MAIN, &dns_main), "dns_main");

  // Register user defined event handers
  err = esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, &ModemComponent::got_ip_event_handler, nullptr);
  ESPHL_ERROR_CHECK(err, "GOT IP event handler register error");

  ESP_LOGV(TAG, "DCE setup");

  switch (this->model_) {
    case ModemModel::BG96:
      this->dce = create_BG96_dce(&dce_config, this->dte_, this->ppp_netif_);
      break;
    case ModemModel::SIM800:
      this->dce = create_SIM800_dce(&dce_config, this->dte_, this->ppp_netif_);
      break;
    case ModemModel::SIM7000:
      this->dce = create_SIM7000_dce(&dce_config, this->dte_, this->ppp_netif_);
      break;
    case ModemModel::SIM7600:
      this->dce = create_SIM7600_dce(&dce_config, this->dte_, this->ppp_netif_);
      break;
    default:
      ESP_LOGE(TAG, "Unknown model");
      return;
      break;
  }

  assert(this->dce);

  this->started_ = true;
  ESP_LOGV(TAG, "Setup finished");
}

void ModemComponent::start_connect_() {
  this->connect_begin_ = millis();
  this->status_set_warning("Starting connection");

  global_modem_component->got_ipv4_address_ = false;  // why not this ?

  this->dce->set_mode(modem_mode::COMMAND_MODE);
  vTaskDelay(pdMS_TO_TICKS(2000));

  command_result res = command_result::TIMEOUT;

  res = this->dce->sync();

  if (res != command_result::OK) {
    ESP_LOGW(TAG, "Unable to sync modem. Will retry later");
    return;
  }

  if (this->dte_config_.uart_config.flow_control == ESP_MODEM_FLOW_CONTROL_HW) {
    if (command_result::OK != this->dce->set_flow_control(2, 2)) {
      ESP_LOGE(TAG, "Failed to set the set_flow_control mode");
      return;
    }
    ESP_LOGI(TAG, "set_flow_control OK");
  } else {
    ESP_LOGI(TAG, "not set_flow_control, because 2-wire mode active.");
  }

  bool pin_ok = true;
  if (this->dce->read_pin(pin_ok) == command_result::OK && !pin_ok) {
    if (!this->pin_code_.empty()) {
      ESP_LOGV(TAG, "Set pin code: %s", this->pin_code_.c_str());
      this->dce->set_pin(this->pin_code_);
      vTaskDelay(pdMS_TO_TICKS(2000));  // Need to wait for some time after unlocking the SIM
    }
    if (this->dce->read_pin(pin_ok) == command_result::OK && !pin_ok) {
      ESP_LOGE(TAG, "Invalid PIN");
      return;
    }
  }
  if (pin_ok) {
    if (this->pin_code_.empty()) {
      ESP_LOGD(TAG, "PIN not needed");
    } else {
      ESP_LOGD(TAG, "PIN unlocked");
    }
  }

  ESP_LOGD(TAG, "Entering CMUX mode");
  vTaskDelay(pdMS_TO_TICKS(2000));
  if (this->dce->set_mode(modem_mode::CMUX_MODE)) {
    ESP_LOGD(TAG, "Modem has correctly entered multiplexed command/data mode");
  } else {
    ESP_LOGE(TAG, "Failed to configure multiplexed command mode... exiting");
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(2000));

  // send initial AT commands from yaml
  for (const auto &cmd : this->init_at_commands_) {
    std::string result;
    command_result err = this->dce->at(cmd.c_str(), result, 1000);
    delay(100);  // NOLINT
    ESP_LOGI(TAG, "Init AT command: %s (status %d) -> %s", cmd.c_str(), (int) err, result.c_str());
  }
}

void ModemComponent::got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;
  ESP_LOGW(TAG, "[IP event] Got IP " IPSTR, IP2STR(&ip_info->ip));
  global_modem_component->got_ipv4_address_ = true;
  global_modem_component->connected_ = true;
}

void ModemComponent::loop() {
  const uint32_t now = millis();
  static uint32_t last_log_time = now;

  switch (this->state_) {
    case ModemComponentState::STOPPED:
      if (this->started_) {
        if (!this->modem_ready()) {
          if (now - last_log_time > 20000) {
            ESP_LOGD(TAG, "Waiting for the modem to be ready...");
            last_log_time = now;
          }
          break;
        } else {
          ESP_LOGI(TAG, "Starting modem connection");
          this->state_ = ModemComponentState::CONNECTING;
          this->start_connect_();
        }
      }
      break;
    case ModemComponentState::CONNECTING:
      if (!this->started_) {
        ESP_LOGI(TAG, "Stopped modem connection");
        this->state_ = ModemComponentState::STOPPED;
      } else if (this->connected_) {
        // connection established
        ESP_LOGI(TAG, "Connected via Modem");
        this->state_ = ModemComponentState::CONNECTED;

        this->dump_connect_params_();
        this->status_clear_warning();
      } else if (now - this->connect_begin_ > 45000) {
        ESP_LOGW(TAG, "Connecting via Modem failed! Re-connecting...");
        this->start_connect_();
      }
      break;
    case ModemComponentState::CONNECTED:
      if (!this->started_) {
        ESP_LOGI(TAG, "Stopped Modem connection");
        this->state_ = ModemComponentState::STOPPED;
      } else if (!this->connected_) {
        ESP_LOGW(TAG, "Connection via Modem lost! Re-connecting...");
        this->state_ = ModemComponentState::CONNECTING;
        this->start_connect_();
      }
      break;
  }
}

void ModemComponent::dump_connect_params_() {
  esp_netif_ip_info_t ip;
  esp_netif_get_ip_info(this->ppp_netif_, &ip);
  ESP_LOGCONFIG(TAG, "  IP Address: %s", network::IPAddress(&ip.ip).str().c_str());
  ESP_LOGCONFIG(TAG, "  Hostname: '%s'", App.get_name().c_str());
  ESP_LOGCONFIG(TAG, "  Subnet: %s", network::IPAddress(&ip.netmask).str().c_str());
  ESP_LOGCONFIG(TAG, "  Gateway: %s", network::IPAddress(&ip.gw).str().c_str());

  const ip_addr_t *dns_main_ip = dns_getserver(ESP_NETIF_DNS_MAIN);
  const ip_addr_t *dns_backup_ip = dns_getserver(ESP_NETIF_DNS_BACKUP);
  const ip_addr_t *dns_fallback_ip = dns_getserver(ESP_NETIF_DNS_FALLBACK);

  ESP_LOGCONFIG(TAG, "  DNS main: %s", network::IPAddress(dns_main_ip).str().c_str());
  ESP_LOGCONFIG(TAG, "  DNS backup: %s", network::IPAddress(dns_backup_ip).str().c_str());
  ESP_LOGCONFIG(TAG, "  DNS fallback: %s", network::IPAddress(dns_fallback_ip).str().c_str());
}

}  // namespace modem
}  // namespace esphome

#endif
