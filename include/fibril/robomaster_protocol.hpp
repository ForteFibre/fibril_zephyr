#pragma once

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>

#include <fibril/motor_control.hpp>

namespace fibril
{

class RoboMasterProtocolService
{
public:
  static constexpr std::size_t MotorCount = MotorControlService::MotorCount;

  RoboMasterProtocolService(
    const struct device * control_can, MotorControlService & motor_control,
    const std::array<const struct device *, MotorCount> & motors);

  int start();
  bool connected() const;

private:
  struct Impl;
  Impl * impl_;
};

}  // namespace fibril
