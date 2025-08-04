#include "uart_text_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace uart {

static const char *const TAG = "uart.text_sensor";

void UARTTextSensor::loop() {
  while (this->available()) {
    char c = this->read();
    this->buffer_ += c;
    if (this->buffer_.size() >= this->separator_.size()) {
      if (this->buffer_.compare(this->buffer_.size() - this->separator_.size(), this->separator_.size(),
                                this->separator_) == 0) {
        // Remove separator from end of buffer
        this->buffer_.erase(this->buffer_.size() - this->separator_.size(), this->separator_.size());
        this->publish_state(this->buffer_);
        ESP_LOGV(TAG, "Published text from UART: %s", this->buffer_.c_str());
        this->buffer_.clear();
      }
    }
  }
}

}  // namespace uart
}  // namespace esphome
