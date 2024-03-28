#pragma once

#include <stdint.h>

namespace esphome {
namespace ota_http {
namespace watchdog {

static const char *const TAG = "ota_http.watchdog";

class Watchdog {
  public:
    static uint32_t get_timeout();
    static void set_timeout(uint32_t timeout_ms);
    static void reset();

   private:
    static uint32_t timeout_ms_;
    static uint32_t init_timeout_ms_;
    Watchdog() {}
};

}  // namespace watchdog
}  // namespace ota_http
}  // namespace esphome
