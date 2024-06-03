#include "../mpu6050.h"
#include "mpu6050_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mpu6050 {

static const char *const TAG = "mpu6050";

void MPU6050Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MPU6050 sensor:");
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MPU6050 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void MPU6050Sensor::update() {
  ESP_LOGV(TAG, "    Updating MPU6050...");
  uint16_t raw_data[7];
  if (!this->parent_->read_bytes_16(MPU6050_REGISTER_ACCEL_XOUT_H, raw_data, 7)) {
    this->status_set_warning();
    return;
  }
  auto *data = reinterpret_cast<int16_t *>(raw_data);

  float accel_x = data[0] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_y = data[1] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_z = data[2] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;

  float temperature = data[3] / 340.0f + 36.53f;

  float gyro_x = data[4] * MPU6050_SCALE_DPS_PER_DIGIT_2000;
  float gyro_y = data[5] * MPU6050_SCALE_DPS_PER_DIGIT_2000;
  float gyro_z = data[6] * MPU6050_SCALE_DPS_PER_DIGIT_2000;

  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, "
           "gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, temp=%.3f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z);

  this->status_clear_warning();
}
float MPU6050Sensor::get_setup_priority() const { return setup_priority::DATA - 1; }

}  // namespace mpu6050
}  // namespace esphome
