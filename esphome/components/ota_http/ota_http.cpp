#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/md5/md5.h"
#include "esphome/components/ota/ota_backend_arduino_esp32.h"
#include "esphome/components/ota/ota_backend_arduino_esp8266.h"
#include "esphome/components/ota/ota_backend_arduino_rp2040.h"
#include "esphome/components/ota/ota_backend_esp_idf.h"
#include "esphome/components/ota/ota_backend.h"
#include "ota_http.h"

#ifdef USE_ESP8266
#include "esphome/components/esp8266/preferences.h"
#endif
#ifdef USE_RP2040
#include "esphome/components/rp2040/preferences.h"
#endif

#ifdef CONFIG_WATCHDOG_TIMEOUT
#include "watchdog.h"
#endif

namespace esphome {
namespace ota_http {

std::unique_ptr<ota::OTABackend> make_ota_backend() {
#ifdef USE_ESP8266
  ESP_LOGD(TAG, "Using ArduinoESP8266OTABackend");
  return make_unique<ota::ArduinoESP8266OTABackend>();
#endif  // USE_ESP8266

#ifdef USE_ARDUINO
#ifdef USE_ESP32
  ESP_LOGD(TAG, "Using ArduinoESP32OTABackend");
  return make_unique<ota::ArduinoESP32OTABackend>();
#endif  // USE_ESP32
#endif  // USE_ARDUINO

#ifdef USE_ESP_IDF
  ESP_LOGD(TAG, "Using IDFOTABackend");
  return make_unique<ota::IDFOTABackend>();
#endif  // USE_ESP_IDF
#ifdef USE_RP2040
  ESP_LOGD(TAG, "Using ArduinoRP2040OTABackend");
  return make_unique<ota::ArduinoRP2040OTABackend>();
#endif  // USE_RP2040
  ESP_LOGE(TAG, "No OTA backend!");
}

const std::unique_ptr<ota::OTABackend> OtaHttpComponent::BACKEND = make_ota_backend();

OtaHttpComponent::OtaHttpComponent() {
  this->pref_obj_.load(&this->pref_);
  if (!this->pref_obj_.save(&this->pref_)) {
    // error at 'load' might be caused by 1st usage, but error at 'save' is a real error.
    ESP_LOGE(TAG, "Unable to use flash memory. Safe mode might be not available");
  }
}

void OtaHttpComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "OTA Update over http:");
  pref_.last_md5[MD5_SIZE] = '\0';
  ESP_LOGCONFIG(TAG, "  Last flashed md5: %s", pref_.last_md5);
  ESP_LOGCONFIG(TAG, "  Max url length: %d", CONFIG_MAX_URL_LENGTH);
  ESP_LOGCONFIG(TAG, "  Timeout: %llus", this->timeout_ / 1000);
#ifdef CONFIG_WATCHDOG_TIMEOUT
  ESP_LOGCONFIG(TAG, "  Watchdog timeout: %ds", CONFIG_WATCHDOG_TIMEOUT / 1000);
#endif
#ifdef OTA_HTTP_ONLY_AT_BOOT
  ESP_LOGCONFIG(TAG, "  Safe mode: Yes");
#else
  ESP_LOGCONFIG(TAG, "  Safe mode: %s", this->safe_mode_ ? "Fallback" : "No");
#endif
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  ESP_LOGCONFIG(TAG, "  TLS server verification: Yes");
#else
  ESP_LOGCONFIG(TAG, "  TLS server verification: No");
#endif
#ifdef USE_ESP8266
#ifdef USE_HTTP_REQUEST_ESP8266_HTTPS
  ESP_LOGCONFIG(TAG, "  ESP8266 SSL support: No");
#else
  ESP_LOGCONFIG(TAG, "  ESP8266 SSL support: Yes");
#endif
#endif
};

void OtaHttpComponent::flash() {
  if (this->pref_.ota_http_state != OTA_HTTP_STATE_SAFE_MODE) {
    ESP_LOGV(TAG, "Setting state to 'progress'");
    this->pref_.ota_http_state = OTA_HTTP_STATE_PROGRESS;
    this->pref_obj_.save(&this->pref_);
  }

  global_preferences->sync();

#ifdef OTA_HTTP_ONLY_AT_BOOT
  if (this->pref_.ota_http_state != OTA_HTTP_STATE_SAFE_MODE) {
    ESP_LOGI(TAG, "Rebooting before flashing new firmware");
    App.safe_reboot();
  }
#endif
#ifdef CONFIG_WATCHDOG_TIMEOUT
  watchdog::Watchdog::set_timeout(CONFIG_WATCHDOG_TIMEOUT);
#endif
  uint32_t update_start_time = millis();
  uint8_t buf[this->http_recv_buffer_ + 1];
  int error_code = 0;
  uint32_t last_progress = 0;
  md5::MD5Digest md5_receive;
  std::unique_ptr<char[]> md5_receive_str(new char[33]);
  if (!this->http_get_md5()) {
    return;
  }

  ESP_LOGD(TAG, "MD5 expected: %s", this->md5_expected_);

  if (!CONFIG_FORCE_UPDATE) {
    if (strncmp(this->pref_.last_md5, this->md5_expected_, MD5_SIZE) == 0) {
      this->http_end();
      ESP_LOGW(TAG, "OTA Update skipped: retrieved md5 %s match the last installed firmware", this->pref_.last_md5);
#ifdef CONFIG_WATCHDOG_TIMEOUT
      watchdog::Watchdog::reset();
#endif
      return;
    }
  }

  if (!this->set_url(this->pref_.url))
    return;
  ESP_LOGI(TAG, "Trying to connect to url: %s", this->safe_url_);
  this->http_init();
  if (!this->check_status()) {
    this->http_end();
    return;
  }

  // we will compute MD5 on the fly for verification -- Arduino OTA seems to ignore it
  md5_receive.init();
  ESP_LOGV(TAG, "MD5Digest initialized");

  error_code = esphome::ota_http::OtaHttpComponent::BACKEND->begin(this->body_length_);
  if (error_code != 0) {
    ESP_LOGW(TAG, "BACKEND->begin error: %d", error_code);
    this->cleanup_();
    return;
  }
  ESP_LOGI(TAG, "OTA backend begin");

  this->bytes_read_ = 0;
  while (this->bytes_read_ != this->body_length_) {
    // read a maximum of chunk_size bytes into buf. (real read size returned)
    int bufsize = this->http_read(buf, this->http_recv_buffer_);

    // feed watchdog and give other tasks a chance to run
    App.feed_wdt();
    yield();

    if (bufsize < 0) {
      ESP_LOGE(TAG, "Stream closed");
      this->cleanup_();
      return;
    }

    // add read bytes to MD5
    md5_receive.add(buf, bufsize);

    // write bytes to OTA backend
    this->update_started_ = true;
    error_code = ota_http::OtaHttpComponent::BACKEND->write(buf, bufsize);
    if (error_code != 0) {
      // error code explaination available at
      // https://github.com/esphome/esphome/blob/dev/esphome/components/ota/ota_component.h
      ESP_LOGE(TAG, "Error code (%d) writing binary data to flash at offset %d and size %d", error_code,
               this->bytes_read_ - bufsize, this->body_length_);
      this->cleanup_();
      return;
    }

    uint32_t now = millis();
    if ((now - last_progress > 1000) or (this->bytes_read_ == this->body_length_)) {
      last_progress = now;
      ESP_LOGI(TAG, "Progress: %0.1f%%", this->bytes_read_ * 100. / this->body_length_);
    }
  }  // while

  ESP_LOGI(TAG, "Done in %.0f seconds", float(millis() - update_start_time) / 1000);

  // verify MD5 is as expected and act accordingly
  md5_receive.calculate();
  md5_receive.get_hex(md5_receive_str.get());
  if (strncmp(md5_receive_str.get(), this->md5_expected_, MD5_SIZE) != 0) {
    ESP_LOGE(TAG, "MD5 computed: %s - Aborting due to MD5 mismatch", md5_receive_str.get());
    this->cleanup_();
    return;
  } else {
    ota_http::OtaHttpComponent::BACKEND->set_update_md5(md5_receive_str.get());
  }

  this->http_end();

  // feed watchdog and give other tasks a chance to run
  App.feed_wdt();
  yield();
  delay(100);  // NOLINT

  error_code = ota_http::OtaHttpComponent::BACKEND->end();
  if (error_code != 0) {
    ESP_LOGE(TAG, "Error ending OTA (%d)", error_code);
    this->cleanup_();
    return;
  }

  this->pref_.ota_http_state = OTA_HTTP_STATE_OK;
  strncpy(this->pref_.last_md5, this->md5_expected_, MD5_SIZE);
  this->pref_obj_.save(&this->pref_);
    // on rp2040 and esp8266, reenable write to flash that was disabled by OTA
#ifdef USE_ESP8266
  esp8266::preferences_prevent_write(false);
#endif
#ifdef USE_RP2040
  rp2040::preferences_prevent_write(false);
#endif
  global_preferences->sync();
  delay(10);
  ESP_LOGI(TAG, "OTA update completed");
  delay(10);
  esphome::App.safe_reboot();
}

void OtaHttpComponent::cleanup_() {
  if (this->update_started_) {
    ESP_LOGV(TAG, "Aborting OTA backend");
    ota_http::OtaHttpComponent::BACKEND->abort();
  }
  ESP_LOGV(TAG, "Aborting HTTP connection");
  this->http_end();
  if (this->pref_.ota_http_state == OTA_HTTP_STATE_SAFE_MODE) {
    ESP_LOGE(TAG, "Previous safe mode unsuccessful; skipped ota_http");
    this->pref_.ota_http_state = OTA_HTTP_STATE_ABORT;
  }
  this->pref_obj_.save(&this->pref_);
#ifdef CONFIG_WATCHDOG_TIMEOUT
  watchdog::Watchdog::reset();
#endif
};

void OtaHttpComponent::check_upgrade() {
  // function called at boot time if CONF_SAFE_MODE is True or "fallback"
  this->safe_mode_ = true;
  if (this->pref_obj_.load(&this->pref_)) {
    if (this->pref_.ota_http_state == OTA_HTTP_STATE_PROGRESS) {
      // progress at boot time means that there was a problem

      // Delay here to allow power to stabilise before Wi-Fi/Ethernet is initialised.
      delay(300);  // NOLINT
      App.setup();

      ESP_LOGI(TAG, "Previous ota_http unsuccessful. Retrying...");
      this->pref_.ota_http_state = OTA_HTTP_STATE_SAFE_MODE;
      this->pref_obj_.save(&this->pref_);
      this->flash();
      return;
    }
    if (this->pref_.ota_http_state == OTA_HTTP_STATE_SAFE_MODE) {
      ESP_LOGE(TAG, "Previous safe mode unsuccessful; skipped ota_http");
      this->pref_.ota_http_state = OTA_HTTP_STATE_ABORT;
      this->pref_obj_.save(&this->pref_);
      global_preferences->sync();
    }
  }
}

bool OtaHttpComponent::http_get_md5() {
  if (!this->set_url(this->pref_.md5_url))
    return false;
  ESP_LOGI(TAG, "Trying to connect to url: %s", this->safe_url_);
  this->http_init();
  if (!this->check_status()) {
    this->http_end();
    return false;
  }
  int length = this->body_length_;
  if (length < 0) {
    this->http_end();
    return false;
  }
  if (length < MD5_SIZE) {
    ESP_LOGE(TAG, "MD5 file must be %u bytes; %u bytes reported by HTTP server. Aborting", MD5_SIZE,
             this->body_length_);
    this->http_end();
    return false;
  }

  auto read_len = this->http_read((uint8_t *) this->md5_expected_, MD5_SIZE);
  this->http_end();

  return read_len == MD5_SIZE;
}

bool OtaHttpComponent::set_url(char *url) {
  this->body_length_ = 0;
  this->status_ = -1;
  this->bytes_read_ = 0;
  if (url == nullptr) {
    ESP_LOGE(TAG, "Bad url: (nullptr)");
    return false;
  }
  if (strncmp(url, "http", 4) != 0) {
    ESP_LOGE(TAG, "Bad url: %s", url);
    return false;
  }
  this->url_ = url;
  this->set_safe_url_();
  return true;
}

bool OtaHttpComponent::save_url_(const std::string &value, char *url) {
  if (value.length() > CONFIG_MAX_URL_LENGTH - 1) {
    ESP_LOGE(TAG, "Url max length is %d, and attempted to set url with length %d: %s", CONFIG_MAX_URL_LENGTH,
             value.length(), value.c_str());
    return false;
  }
  strncpy(url, value.c_str(), value.length());
  url[value.length()] = '\0';  // null terminator
  this->pref_obj_.save(&this->pref_);
  return true;
}

bool OtaHttpComponent::check_status() {
  // status can be -1, or http status code
  if (this->status_ < 100) {
    ESP_LOGE(TAG, "No answer from http server (error %d). Network error?", this->status_);
    return false;
  }
  if (this->status_ >= 310) {
    ESP_LOGE(TAG, "HTTP error %d", this->status_);
    return false;
  }
  ESP_LOGV(TAG, "HTTP status %d", this->status_);
  return true;
}

}  // namespace ota_http
}  // namespace esphome