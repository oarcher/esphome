#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace axp2101 {

class AXP2101Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
};

}  // namespace axp2101
}  // namespace esphome
