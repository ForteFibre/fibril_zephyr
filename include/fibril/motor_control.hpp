#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

namespace fibril
{

class SoftwareWatchdog
{
public:
  explicit SoftwareWatchdog(int32_t timeout_ms = 1000);

  void kick();
  void reset();
  void kill();
  void set_timeout(int32_t timeout_ms);
  bool alive() const;
  int32_t timeout_ms() const;

private:
  std::atomic<int32_t> last_alive_ms_;
  std::atomic<int32_t> timeout_ms_;
};

enum class MotorMode : uint8_t {
  Duty,
  Speed,
  Position,
  PositionSpeed,
};

struct MotorControlConfig
{
  const struct device * dev;
  int16_t max_current;
};

class MotorControlService
{
public:
  static constexpr std::size_t MotorCount = 8;

  MotorControlService();

  int init(const std::array<MotorControlConfig, MotorCount> & configs);
  int start();

  bool claim(std::size_t index, uintptr_t owner);
  void release(std::size_t index, uintptr_t owner);
  bool has_owner(std::size_t index, uintptr_t owner) const;
  void release_all(uintptr_t owner);
  void reset_all();

  int set_duty(std::size_t index, float value);
  int set_speed(std::size_t index, float value);
  int set_position(std::size_t index, float value);
  int set_position_speed(std::size_t index, float value);
  int reset_encoder(std::size_t index);
  int set_encoder_value(std::size_t index, float value);
  int reset_safety(std::size_t index);

  int configure_speed_pid(std::size_t index, float kp, float ki, float kd, float max_output);
  int configure_position_pid(std::size_t index, float kp, float ki, float kd, float max_output);
  int configure_speed_filter(std::size_t index, float coefficient);
  int configure_duty_acceleration_limit(std::size_t index, float acc_max);
  void set_position_tolerance(std::size_t index, float tolerance);

  MotorMode mode(std::size_t index) const;
  float target_position(std::size_t index) const;
  float current_output(std::size_t index) const;
  bool is_online(std::size_t index) const;

private:
  struct Impl;
  Impl * impl_;
};

}  // namespace fibril
