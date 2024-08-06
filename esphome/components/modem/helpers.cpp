#ifdef USE_ESP_IDF

#include "modem_component.h"
#include "helpers.h"

#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <esp_idf_version.h>
#include <esp_task_wdt.h>

#include <unordered_map>
#include <string>

namespace esphome {
namespace modem {

std::string command_result_to_string(command_result err) {
  std::string res = "UNKNOWN";
  switch (err) {
    case command_result::FAIL:
      res = "FAIL";
      break;
    case command_result::OK:
      res = "OK";
      break;
    case command_result::TIMEOUT:
      res = "TIMEOUT";
  }
  return res;
}

std::string state_to_string(ModemComponentState state) {
  std::string str;
  switch (state) {
    case ModemComponentState::NOT_RESPONDING:
      str = "NOT_RESPONDING";
      break;
    case ModemComponentState::DISCONNECTED:
      str = "DISCONNECTED";
      break;
    case ModemComponentState::CONNECTED:
      str = "CONNECTED";
      break;
    case ModemComponentState::DISABLED:
      str = "DISABLED";
      break;
  }
  return str;
}

}  // namespace modem
}  // namespace esphome
#endif  // USE_ESP_IDF