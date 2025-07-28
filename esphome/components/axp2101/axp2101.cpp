#include "axp2101.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp2101 {

static const char *const TAG = "axp2101";
static const uint8_t AXP2101_CHIP_ID_REG = 0x03;

void AXP2101Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AXP2101...");
  uint8_t chip_id = 0;
  if (this->read_byte(AXP2101_CHIP_ID_REG, &chip_id) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with AXP2101 failed!");
    this->mark_failed();
    return;
  }
  if (chip_id != 0x4A) {
    ESP_LOGE(TAG, "AXP2101 not found! (Chip ID: 0x%02X)", chip_id);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "AXP2101 found. Chip ID: 0x%02X", chip_id);
}

void AXP2101Component::dump_config() { LOG_I2C_DEVICE(this); }

}  // namespace axp2101
}  // namespace esphome
