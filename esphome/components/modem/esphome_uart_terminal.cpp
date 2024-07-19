/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef USE_ESP_IDF

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "esp_log.h"
// #include "driver/uart.h"
// //#include "uart_compat.h"
// #include "esp_modem_config.h"
// #include "exception_stub.hpp"
// #include "cxx_include/esp_modem_dte.hpp"
// #include "uart_resource.hpp"

#include "esphome_uart_terminal.h"

namespace esphome {
namespace modem {
namespace uart {

using namespace esp_modem;

// std::unique_ptr<Terminal> create_uart_terminal(const esp_modem_dte_config *config)
// {
//     TRY_CATCH_RET_NULL(
//         auto term = std::make_unique<UartTerminal>(config);
//         term->start();
//         return term;
//     )
// }

#ifndef UART_HW_FIFO_LEN
// to build with IDF <= v5.1
#define UART_HW_FIFO_LEN(uart_nr) UART_FIFO_LEN
#endif

uart_resource::~uart_resource() {
  if (port >= UART_NUM_0 && port < UART_NUM_MAX) {
    uart_driver_delete(port);
  }
}

uart_resource::uart_resource(const esp_modem_uart_term_config *config, QueueHandle_t *event_queue, int fd)
    : port(UART_NUM_MAX) {
  esp_err_t res;

  /* Config UART */
  uart_config_t uart_config = {};
  uart_config.baud_rate = config->baud_rate;
  uart_config.data_bits = config->data_bits;
  uart_config.parity = config->parity;
  uart_config.stop_bits = config->stop_bits;
  uart_config.flow_ctrl =
      (config->flow_control == ESP_MODEM_FLOW_CONTROL_HW) ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = config->source_clk;

  ESP_MODEM_THROW_IF_ERROR(uart_param_config(config->port_num, &uart_config), "config uart parameter failed");

  if (config->flow_control == ESP_MODEM_FLOW_CONTROL_HW) {
    res = uart_set_pin(config->port_num, config->tx_io_num, config->rx_io_num, config->rts_io_num, config->cts_io_num);
  } else {
    res = uart_set_pin(config->port_num, config->tx_io_num, config->rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }
  ESP_MODEM_THROW_IF_ERROR(res, "config uart gpio failed");
  /* Set flow control threshold */
  if (config->flow_control == ESP_MODEM_FLOW_CONTROL_HW) {
    res = uart_set_hw_flow_ctrl(config->port_num, UART_HW_FLOWCTRL_CTS_RTS, UART_HW_FIFO_LEN(config->port_num) - 8);
  } else if (config->flow_control == ESP_MODEM_FLOW_CONTROL_SW) {
    res = uart_set_sw_flow_ctrl(config->port_num, true, 8, UART_HW_FIFO_LEN(config->port_num) - 8);
  }
  ESP_MODEM_THROW_IF_ERROR(res, "config uart flow control failed");

  /* Install UART driver and get event queue used inside driver */
  res = uart_driver_install(config->port_num, config->rx_buffer_size, config->tx_buffer_size, config->event_queue_size,
                            config->event_queue_size ? event_queue : nullptr, 0);
  ESP_MODEM_THROW_IF_ERROR(res, "install uart driver failed");
  ESP_MODEM_THROW_IF_ERROR(uart_set_rx_timeout(config->port_num, 1), "set rx timeout failed");

  ESP_MODEM_THROW_IF_ERROR(uart_set_rx_full_threshold(config->port_num, 64), "config rx full threshold failed");

  /* mark UART as initialized */
  port = config->port_num;
}

void UartTerminal::task() {
  uart_event_t event;
  size_t len;
  signal.set(TASK_INIT);
  signal.wait_any(TASK_START | TASK_STOP, portMAX_DELAY);
  if (signal.is_any(TASK_STOP)) {
    return;  // exits to the static method where the task gets deleted
  }
  while (signal.is_any(TASK_START)) {
    if (get_event(event, 100)) {
      switch (event.type) {
        case UART_DATA:
          uart_get_buffered_data_len(uart.port, &len);
          if (len && on_read) {
            on_read(nullptr, len);
          }
          break;
        case UART_FIFO_OVF:
          ESP_LOGW(TAG, "HW FIFO Overflow");
          if (on_error) {
            on_error(terminal_error::BUFFER_OVERFLOW);
          }
          reset_events();
          break;
        case UART_BUFFER_FULL:
          ESP_LOGW(TAG, "Ring Buffer Full");
          if (on_error) {
            on_error(terminal_error::BUFFER_OVERFLOW);
          }
          reset_events();
          break;
        case UART_BREAK:
          ESP_LOGW(TAG, "Rx Break");
          if (on_error) {
            on_error(terminal_error::UNEXPECTED_CONTROL_FLOW);
          }
          break;
        case UART_PARITY_ERR:
          ESP_LOGE(TAG, "Parity Error");
          if (on_error) {
            on_error(terminal_error::CHECKSUM_ERROR);
          }
          break;
        case UART_FRAME_ERR:
          ESP_LOGE(TAG, "Frame Error");
          if (on_error) {
            on_error(terminal_error::UNEXPECTED_CONTROL_FLOW);
          }
          break;
        default:
          ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
          break;
      }
    }
  }
}

int UartTerminal::read(uint8_t *data, size_t len) {
  size_t length = 0;
  uart_get_buffered_data_len(uart.port, &length);
  length = std::min(len, length);
  if (length > 0) {
    return uart_read_bytes(uart.port, data, length, portMAX_DELAY);
  }
  return 0;
}

int UartTerminal::write(uint8_t *data, size_t len) { return uart_write_bytes(uart.port, data, len); }

}  // namespace uart
}  // namespace modem
}  // namespace esphome

#endif