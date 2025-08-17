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

std::string modem_mode_to_string(modem_mode mode) {
  std::string res = "UNKNOWN";
  switch (mode) {
    case modem_mode::AUTODETECT:
      res = "AUTODETECT";
      break;
    case modem_mode::COMMAND_MODE:
      res = "COMMAND_MODE";
      break;
    case modem_mode::DATA_MODE:
      res = "DATA_MODE";
      break;
    case modem_mode::DUAL_MODE:
      res = "DUAL_MODE";
      break;
    case modem_mode::CMUX_MODE:
      res = "CMUX_MODE";
      break;
    case modem_mode::CMUX_MANUAL_MODE:
      res = "CMUX_MANUAL_MODE";
      break;
    case modem_mode::CMUX_MANUAL_EXIT:
      res = "CMUX_MANUAL_EXIT";
      break;
    case modem_mode::CMUX_MANUAL_DATA:
      res = "CMUX_MANUAL_DATA";
      break;
    case modem_mode::CMUX_MANUAL_COMMAND:
      res = "CMUX_MANUAL_COMMAND";
      break;
    case modem_mode::CMUX_MANUAL_SWAP:
      res = "CMUX_MANUAL_SWAP";
      break;
    case modem_mode::RESUME_DATA_MODE:
      res = "RESUME_DATA_MODE";
      break;
    case modem_mode::RESUME_COMMAND_MODE:
      res = "RESUME_COMMAND_MODE";
      break;
    case modem_mode::RESUME_CMUX_MANUAL_MODE:
      res = "RESUME_CMUX_MANUAL_MODE";
      break;
    case modem_mode::RESUME_CMUX_MANUAL_DATA:
      res = "RESUME_CMUX_MANUAL_DATA";
      break;
    case modem_mode::UNDEF:
      res = "UNDEF";
      break;
  }
  return res;
}

std::string state_to_string(ModemComponentState state) { return std::to_string((int) state); }

std::string network_system_mode_to_string(int mode) {
  if (mode == 0) {
    return "No service";
  } else {
    return std::to_string(mode);
  }
}

std::string get_signal_bars(float rssi) {
  if (std::isnan(rssi)) {
    return "None";
  } else if (rssi >= -50) {
    return "High";
  } else if (rssi >= -65) {
    return "Good";
  } else if (rssi >= -85) {
    return "Medium";
  } else {
    return "Low";
  }
}

}  // namespace modem
}  // namespace esphome
#endif  // USE_ESP_IDF
