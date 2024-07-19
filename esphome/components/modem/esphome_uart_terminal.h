#pragma once

#ifdef USE_ESP_IDF

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
// #include "uart_compat.h"
#include "esp_modem_config.h"
// #include "exception_stub.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "esp_modem_config.h"
// #include "uart_resource.hpp"

namespace esphome {
namespace modem {
namespace uart {

using namespace esp_modem;

static const char *const TAG = "modem_esphome_uart";

struct uart_task {
  explicit uart_task(size_t stack_size, size_t priority, void *task_param, TaskFunction_t task_function)
      : task_handle(nullptr) {
    BaseType_t ret = xTaskCreate(task_function, "uart_task", stack_size, task_param, priority, &task_handle);
    ESP_MODEM_THROW_IF_FALSE(ret == pdTRUE, "create uart event task failed");
  }

  ~uart_task() {
    if (task_handle) {
      vTaskDelete(task_handle);
    }
  }

  TaskHandle_t task_handle; /*!< UART event task handle */
};

struct uart_resource {
  explicit uart_resource(const esp_modem_uart_term_config *config, QueueHandle_t *event_queue, int fd);

  ~uart_resource();

  uart_port_t port{};
};

class UartTerminal : public Terminal {
 public:
  explicit UartTerminal(const esp_modem_dte_config *config)
      : event_queue(),
        uart(&config->uart_config, &event_queue, -1),
        signal(),
        task_handle(config->task_stack_size, config->task_priority, this, s_task) {}

  ~UartTerminal() override = default;

  void start() override { signal.set(TASK_START); }

  void stop() override { signal.set(TASK_STOP); }

  int write(uint8_t *data, size_t len) override;

  int read(uint8_t *data, size_t len) override;

  void set_read_cb(std::function<bool(uint8_t *data, size_t len)> f) override { on_read = std::move(f); }

 private:
  static void s_task(void *task_param) {
    auto t = static_cast<UartTerminal *>(task_param);
    t->task();
    vTaskDelete(nullptr);
  }

  void task();
  bool get_event(uart_event_t &event, uint32_t time_ms) {
    return xQueueReceive(event_queue, &event, pdMS_TO_TICKS(time_ms));
  }

  void reset_events() {
    uart_flush_input(uart.port);
    xQueueReset(event_queue);
  }

  static const size_t TASK_INIT = BIT0;
  static const size_t TASK_START = BIT1;
  static const size_t TASK_STOP = BIT2;

  QueueHandle_t event_queue;
  uart_resource uart;
  SignalGroup signal;
  uart_task task_handle;
};

}  // namespace uart
}  // namespace modem
}  // namespace esphome

#endif  // USE_ESP_IDF
