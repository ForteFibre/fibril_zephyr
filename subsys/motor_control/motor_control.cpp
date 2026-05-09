#include <fibril/motor_control.hpp>

#include <algorithm>
#include <cerrno>

#include <drivers/motor.h>
#include <fibril/controller/acceleration_limit.hpp>
#include <fibril/controller/pid.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fibril_motor_control, CONFIG_MOTOR_LOG_LEVEL);

namespace fibril
{
namespace
{
constexpr int64_t ControlPeriodUs = 1000;
constexpr float ControlPeriodSeconds = 0.001f;
constexpr int16_t DefaultMaxCurrent = 10000;
constexpr float DefaultPositionTolerance = 0.01f;
}  // namespace

class MotorControlService::Impl
{
public:
  struct MotorState
  {
    MotorState() : speed_controller(), position_controller(), duty_limit() {}

    const struct device * dev = nullptr;
    int16_t max_current = DefaultMaxCurrent;
    uintptr_t owner = 0U;
    bool enabled = false;
    bool initialized = false;
    bool encoder_valid = false;
    MotorMode mode = MotorMode::Duty;
    float target_position = 0.0f;
    float current_command = 0.0f;
    float speed_filtered = 0.0f;
    float speed_filter_coefficient = 0.9f;
    float position_offset = 0.0f;
    float tolerance = DefaultPositionTolerance;
    struct k_spinlock lock;
    PIDController<float> speed_controller;
    PIDController<float> position_controller;
    AccelerationLimit duty_limit;
  };

  std::array<MotorState, MotorCount> motors;
  struct k_thread thread;
  k_tid_t thread_id = nullptr;
  K_KERNEL_STACK_MEMBER(stack, 4096);
  bool running = false;

  static void thread_entry(void * arg1, void *, void *)
  {
    static_cast<Impl *>(arg1)->run();
  }

  void run()
  {
    while (running) {
      const int64_t started_at = k_uptime_get();
      for (std::size_t index = 0; index < motors.size(); ++index) {
        update_motor(index);
      }
      const int64_t elapsed_us = (k_uptime_get() - started_at) * 1000;
      if (elapsed_us < ControlPeriodUs) {
        k_sleep(K_USEC(ControlPeriodUs - elapsed_us));
      } else {
        k_yield();
      }
    }
  }

  void update_motor(std::size_t index)
  {
    auto & motor = motors.at(index);
    struct motor_feedback feedback = {};
    float requested_current = 0.0f;
    bool should_enable = false;
    bool encoder_valid = false;
    int ret;

    if (motor.dev == nullptr) {
      return;
    }

    ret = motor_get_feedback(motor.dev, &feedback);
    if (ret == 0) {
      encoder_valid = feedback.online && !feedback.stale &&
        (feedback.valid_mask & MOTOR_FEEDBACK_POSITION) != 0U &&
        (feedback.valid_mask & MOTOR_FEEDBACK_VELOCITY) != 0U;
    }

    {
      const k_spinlock_key_t key = k_spin_lock(&motor.lock);
      if (ret == 0) {
        const float velocity = static_cast<float>(feedback.velocity);
        const float position = static_cast<float>(feedback.position) + motor.position_offset;
        motor.speed_filtered =
          motor.speed_filter_coefficient * velocity +
          (1.0f - motor.speed_filter_coefficient) * motor.speed_filtered;
        motor.encoder_valid = encoder_valid;

        if (!encoder_valid && motor.mode != MotorMode::Duty) {
          motor.mode = MotorMode::Duty;
          motor.duty_limit.target(0.0f);
        }

        switch (motor.mode) {
          case MotorMode::Speed:
            motor.speed_controller.update(motor.speed_filtered);
            motor.duty_limit.target(motor.speed_controller.output());
            break;
          case MotorMode::Position:
            motor.position_controller.update(position);
            motor.duty_limit.target(motor.position_controller.output());
            break;
          case MotorMode::PositionSpeed:
            motor.position_controller.update(position);
            motor.speed_controller.target(motor.position_controller.output());
            motor.speed_controller.update(motor.speed_filtered);
            motor.duty_limit.target(motor.speed_controller.output());
            break;
          case MotorMode::Duty:
          default:
            break;
        }
      }

      motor.duty_limit.update(ControlPeriodSeconds);
      requested_current =
        std::clamp(motor.duty_limit.output(), -1.0f, 1.0f) * static_cast<float>(motor.max_current);
      motor.current_command = requested_current;
      should_enable = motor.enabled;
      k_spin_unlock(&motor.lock, key);
    }

    if (should_enable) {
      (void)motor_enable(motor.dev);
      (void)motor_set_output(
        motor.dev, MOTOR_OUTPUT_MODE_CURRENT, static_cast<int16_t>(requested_current));
    } else {
      (void)motor_disable(motor.dev);
    }
  }
};

SoftwareWatchdog::SoftwareWatchdog(int32_t timeout_ms) : last_alive_ms_(-1), timeout_ms_(timeout_ms)
{
}

void SoftwareWatchdog::kick()
{
  if (alive()) {
    last_alive_ms_.store(k_uptime_get_32(), std::memory_order_relaxed);
  }
}

void SoftwareWatchdog::reset()
{
  last_alive_ms_.store(k_uptime_get_32(), std::memory_order_relaxed);
}

void SoftwareWatchdog::kill()
{
  last_alive_ms_.store(-1, std::memory_order_relaxed);
}

void SoftwareWatchdog::set_timeout(int32_t timeout_ms)
{
  timeout_ms_.store(timeout_ms, std::memory_order_relaxed);
}

bool SoftwareWatchdog::alive() const
{
  const int32_t last_alive = last_alive_ms_.load(std::memory_order_relaxed);
  if (last_alive < 0) {
    return false;
  }
  return (int32_t)(k_uptime_get_32() - (uint32_t)last_alive) <
    timeout_ms_.load(std::memory_order_relaxed);
}

int32_t SoftwareWatchdog::timeout_ms() const
{
  return timeout_ms_.load(std::memory_order_relaxed);
}

MotorControlService::MotorControlService() : impl_(new Impl()) {}

int MotorControlService::init(const std::array<MotorControlConfig, MotorCount> & configs)
{
  for (std::size_t index = 0; index < configs.size(); ++index) {
    auto & motor = impl_->motors.at(index);
    motor.dev = configs.at(index).dev;
    motor.max_current =
      configs.at(index).max_current > 0 ? configs.at(index).max_current : DefaultMaxCurrent;
    motor.enabled = device_is_ready(motor.dev);
    motor.initialized = device_is_ready(motor.dev);
    motor.speed_controller.i_saturation(1.0f);
    motor.position_controller.i_saturation(1.0f);
    k_spinlock_key_t key = k_spin_lock(&motor.lock);
    motor.owner = 0U;
    motor.mode = MotorMode::Duty;
    motor.duty_limit.target(0.0f);
    motor.duty_limit.acc_max(0.0f);
    k_spin_unlock(&motor.lock, key);
    if (!motor.initialized) {
      return -ENODEV;
    }
  }
  return 0;
}

int MotorControlService::start()
{
  if (impl_->thread_id != nullptr) {
    return -EALREADY;
  }

  impl_->running = true;
  impl_->thread_id = k_thread_create(
    &impl_->thread, impl_->stack, K_KERNEL_STACK_SIZEOF(impl_->stack),
    Impl::thread_entry, impl_, nullptr, nullptr, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);

  if (impl_->thread_id == nullptr) {
    impl_->running = false;
    return -EINVAL;
  }

  k_thread_name_set(impl_->thread_id, "motor_ctrl");
  return 0;
}

bool MotorControlService::claim(std::size_t index, uintptr_t owner)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  if ((motor.owner == 0U) || (motor.owner == owner)) {
    motor.owner = owner;
    motor.enabled = true;
    k_spin_unlock(&motor.lock, key);
    return true;
  }
  k_spin_unlock(&motor.lock, key);
  return false;
}

void MotorControlService::release(std::size_t index, uintptr_t owner)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  if (motor.owner == owner) {
    motor.owner = 0U;
    motor.enabled = false;
    motor.mode = MotorMode::Duty;
    motor.duty_limit.target(0.0f);
  }
  k_spin_unlock(&motor.lock, key);
}

bool MotorControlService::has_owner(std::size_t index, uintptr_t owner) const
{
  const auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(const_cast<struct k_spinlock *>(&motor.lock));
  const bool result = motor.owner == owner;
  k_spin_unlock(const_cast<struct k_spinlock *>(&motor.lock), key);
  return result;
}

void MotorControlService::release_all(uintptr_t owner)
{
  for (std::size_t index = 0; index < MotorCount; ++index) {
    release(index, owner);
  }
}

void MotorControlService::reset_all()
{
  for (auto & motor : impl_->motors) {
    const k_spinlock_key_t key = k_spin_lock(&motor.lock);
    motor.owner = 0U;
    motor.enabled = false;
    motor.mode = MotorMode::Duty;
    motor.target_position = 0.0f;
    motor.current_command = 0.0f;
    motor.duty_limit.target(0.0f);
    motor.duty_limit.acc_max(0.0f);
    motor.speed_controller.reset();
    motor.speed_controller.kp(0.0f).ki(0.0f).kd(0.0f).max(1.0f).target(0.0f);
    motor.position_controller.reset();
    motor.position_controller.kp(0.0f).ki(0.0f).kd(0.0f).max(1.0f).target(0.0f);
    k_spin_unlock(&motor.lock, key);
    (void)motor_disable(motor.dev);
  }
}

int MotorControlService::set_duty(std::size_t index, float value)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.mode = MotorMode::Duty;
  motor.duty_limit.target(std::clamp(value, -1.0f, 1.0f));
  motor.enabled = true;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::set_speed(std::size_t index, float value)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  if (motor.mode != MotorMode::Speed) {
    motor.speed_controller.reset();
    motor.mode = MotorMode::Speed;
  }
  motor.speed_controller.target(value);
  motor.enabled = true;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::set_position(std::size_t index, float value)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  if (motor.mode != MotorMode::Position) {
    motor.position_controller.reset();
    motor.mode = MotorMode::Position;
  }
  motor.target_position = value;
  motor.position_controller.target(value);
  motor.enabled = true;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::set_position_speed(std::size_t index, float value)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  if (motor.mode != MotorMode::PositionSpeed) {
    motor.position_controller.reset();
    motor.speed_controller.reset();
    motor.mode = MotorMode::PositionSpeed;
  }
  motor.target_position = value;
  motor.position_controller.target(value);
  motor.enabled = true;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::reset_encoder(std::size_t index)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.position_offset = 0.0f;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::set_encoder_value(std::size_t index, float value)
{
  auto & motor = impl_->motors.at(index);
  struct motor_feedback feedback = {};
  if (motor_get_feedback(motor.dev, &feedback) != 0) {
    return -EIO;
  }
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.position_offset = value - static_cast<float>(feedback.position);
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::reset_safety(std::size_t index)
{
  ARG_UNUSED(index);
  return 0;
}

int MotorControlService::configure_speed_pid(
  std::size_t index, float kp, float ki, float kd, float max_output)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.speed_controller.kp(kp).ki(ki).kd(kd).max(max_output);
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::configure_position_pid(
  std::size_t index, float kp, float ki, float kd, float max_output)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.position_controller.kp(kp).ki(ki).kd(kd).max(max_output);
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::configure_speed_filter(std::size_t index, float coefficient)
{
  if ((coefficient < 0.0f) || (coefficient > 1.0f)) {
    return -EINVAL;
  }
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.speed_filter_coefficient = coefficient;
  k_spin_unlock(&motor.lock, key);
  return 0;
}

int MotorControlService::configure_duty_acceleration_limit(std::size_t index, float acc_max)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.duty_limit.acc_max(acc_max);
  k_spin_unlock(&motor.lock, key);
  return 0;
}

void MotorControlService::set_position_tolerance(std::size_t index, float tolerance)
{
  auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(&motor.lock);
  motor.tolerance = tolerance;
  k_spin_unlock(&motor.lock, key);
}

MotorMode MotorControlService::mode(std::size_t index) const
{
  const auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(const_cast<struct k_spinlock *>(&motor.lock));
  const auto result = motor.mode;
  k_spin_unlock(const_cast<struct k_spinlock *>(&motor.lock), key);
  return result;
}

float MotorControlService::target_position(std::size_t index) const
{
  const auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(const_cast<struct k_spinlock *>(&motor.lock));
  const float result = motor.target_position;
  k_spin_unlock(const_cast<struct k_spinlock *>(&motor.lock), key);
  return result;
}

float MotorControlService::current_output(std::size_t index) const
{
  const auto & motor = impl_->motors.at(index);
  const k_spinlock_key_t key = k_spin_lock(const_cast<struct k_spinlock *>(&motor.lock));
  const float result = motor.current_command;
  k_spin_unlock(const_cast<struct k_spinlock *>(&motor.lock), key);
  return result;
}

bool MotorControlService::is_online(std::size_t index) const
{
  struct motor_feedback feedback = {};
  return motor_get_feedback(impl_->motors.at(index).dev, &feedback) == 0 && feedback.online &&
    !feedback.stale;
}

}  // namespace fibril
