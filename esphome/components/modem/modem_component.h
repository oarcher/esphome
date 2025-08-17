#pragma once
#ifdef USE_ESP_IDF

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/core/automation.h"
#include "esphome/core/preferences.h"
#include "esphome/components/network/util.h"

#include <unordered_map>
#include <memory>

#include "modem_handler.h"
namespace esphome {
namespace modem {

enum class ModemComponentState {
  ENABLING,
  POWERING_ON,
  SYNCING,
  INIT_NETWORK,
  START_PPP,
  WAIT_IP,
  CONNECTED,
  DISCONNECTED,
  DISABLING,
  DISABLED,
  POWERING_OFF,
};

class ModemComponent : public Component {
 public:
  // Setters now modify the handler's attributes
  void set_rx_pin(InternalGPIOPin *rx_pin) { this->modem_handler->rx_pin = rx_pin; }
  void set_tx_pin(InternalGPIOPin *tx_pin) { this->modem_handler->tx_pin = tx_pin; }
  void set_model(const std::string &model) { this->modem_handler->model = model; }
  void set_pin_code(const std::string &pin_code) { this->modem_handler->pin_code = pin_code; }
  void set_apn(const std::string &apn) { this->modem_handler->apn = apn; }
  void enable_cmux() { this->modem_handler->cmux = true; }
  void add_init_at_command(const std::string &cmd) { this->modem_handler->init_at_commands.push_back(cmd); }
  bool is_connected() { return this->component_state_ == ModemComponentState::CONNECTED; }
  bool is_disabled() { return this->component_state_ == ModemComponentState::DISABLED && this->disable_wanted_; }
  bool is_enabled() { return !is_disabled(); }

  // Delegated methods
  AtCommandResult send_at(const std::string &cmd, uint32_t timeout = 0, bool verbose = false);
  void enable();
  void disable();
  void reset();

  network::IPAddresses get_ip_addresses();
  std::string get_use_address() const;

  // ========== INTERNAL METHODS ==========
  // (In most use cases, you won't need these)

  ModemComponent();
  void setup() override;
  void loop() override;

  void dump_config() override { this->dump_connect_params_(); }
  float get_setup_priority() const override { return setup_priority::WIFI + 1; }  // Just before Wi-Fi
  std::unique_ptr<ModemHandler> modem_handler{nullptr};

 protected:
  // ===== State handler methods =====
  void handle_state_enabling_();
  void handle_state_syncing_();
  void handle_state_init_network_();
  void handle_state_start_ppp_();
  void handle_state_wait_ip_();
  void handle_state_connected_();
  void handle_state_disconnected_();
  void handle_state_disabling_();
  void handle_state_disabled_();

  void abort_(const std::string &message);
  void loop_delay_(uint32_t delay_ms);
  void dump_connect_params_();

  // Attributes from YAML config
  uint32_t timeout_ = 300000;
  ;
  std::string use_address_;

  // Changes will trigger user callback
  ModemComponentState component_state_{ModemComponentState::DISABLED};
  ModemComponentState component_last_state_{ModemComponentState::DISABLED};

  uint32_t last_health_check_{0};
  uint32_t next_loop_millis_{0};
  bool disable_wanted_{true};
};

extern ModemComponent *global_modem_component;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace modem
}  // namespace esphome

#endif
