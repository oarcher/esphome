#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace uart {

class UARTTextSensor : public text_sensor::TextSensor, public uart::UARTDevice, public Component {
 public:
  void loop() override;
  void set_separator(const std::string &separator) { this->separator_ = separator; }

 protected:
  std::string buffer_;
  std::string separator_ = "\n\r";
};

}  // namespace uart
}  // namespace esphome
