#include <fibril/robomaster_protocol.hpp>

#include <array>
#include <bitset>
#include <cerrno>
#include <cstdint>
#include <cstring>

#include <drivers/motor.h>
#include <fibril/data/serde.hpp>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(robomaster_protocol, CONFIG_APP_LOG_LEVEL);

namespace fibril
{
namespace
{
using CANID = uint16_t;
using MotorID = uint8_t;
using TestData = uint8_t;

constexpr CANID HeartbeatId = 0x10;
constexpr CANID ProtocolOffset = 0x00;
constexpr CANID CmdId = 0x170;
constexpr CANID CallbackId = 0x160;
constexpr uint8_t ProtocolVersion = 6;
constexpr uintptr_t OwnerToken = 0x524d4d31U;
constexpr size_t FrameQueueLen = 16;
constexpr int ThreadStackSize = 4096;

enum class Command : uint8_t {
  RESET,
  TEST,
  CMD_SET_DUTY,
  CMD_SET_SPEED,
  CMD_SET_CURRENT,
  CMD_SUB_SETTING,
  DEPRECATED_CALLBACK_ENC_DIFF,
  CALLBACK_OUTPUT_VALUE,
  CALLBACK_CURRENT_VALUE,
  CMD_SET_POSITION,
  CMD_SET_ENCODER_VALUE,
  CMD_SET_POSITION_WITH_SPEED,
  CMD_RESET_SAFETY,
  CALLBACK_MOTOR_STATUS,
  CALLBACK_ENC_VALUE,
  CMD_RESET_ENCODER_VALUE,
  ENABLE_FEATURE,
  DISABLE_FEATURE,
  CALLBACK_ARRIVAL,
  CMD_CALIBRATION_START,
  CALLBACK_CALIBRATION_DATA,
  CMD_SET_ENCODER_OFFSET,
  RESERVED_1,
  CALLBACK_BULK_POSITION,
  CALLBACK_BULK_CURRENT,
  CALLBACK_BULK_OUTPUT,
  CALLBACK_BULK_ADC,
  CALLBACK_BULK_VELOCITY,
  CMD_CALIBRATION_FINISH,
};

enum class Config : uint8_t {
  DUTY_SEND,
  ENCODER_GAIN,
  ENCODER_SEND,
  CURRENT_SEND,
  SPEED_CONTROLLER_MAX,
  SPEED_CONTROLLER_KP,
  SPEED_CONTROLLER_KI,
  SPEED_CONTROLLER_KD,
  CURRENT_CONTROLLER_MAX,
  CURRENT_CONTROLLER_KP,
  CURRENT_CONTROLLER_KI,
  CURRENT_CONTROLLER_KD,
  CMD_TIMEOUT,
  POSITION_CONTROLLER_MAX,
  POSITION_CONTROLLER_KP,
  POSITION_CONTROLLER_KI,
  POSITION_CONTROLLER_KD,
  POSITION_SPEED_CONTROLLER_MAX,
  POSITION_SPEED_CONTROLLER_KP,
  POSITION_SPEED_CONTROLLER_KI,
  POSITION_SPEED_CONTROLLER_KD,
  SAFETY_ENCODER_TIMEOUT,
  ENCODER_SEND_SUM,
  DUTY_DELTA_MAX,
  LIMIT_PROFILE_APPLY,
  LIMIT_PROFILE_CLEAR,
  LIMIT_PROFILE_ADD_POINT,
  LIMIT_PROFILE_SET_FORWARD_SPEED_LIMIT,
  LIMIT_PROFILE_SET_BACKWARD_SPEED_LIMIT,
  LIMIT_PROFILE_SET_ACCELERATION_LIMIT,
  LIMIT_PROFILE_SET_DECELERATION_LIMIT,
  CALIBRATION_CONFIG_PORT,
  CALIBRATION_CONFIG_MAPPING,
  POSITION_TOLERANCE,
  CONTROL_FREQUENCY,
  SPEED_LOW_PASS_FILTER_COEFFICIENT,
  SPEED_CONTROLLER_I_SATURATION,
  POSITION_CONTROLLER_I_SATURATION,
  CURRENT_CONTROLLER_I_SATURATION,
  BULK_POSITION_CALLBACK_INTERVAL,
  BULK_CURRENT_CALLBACK_INTERVAL,
  BULK_OUTPUT_CALLBACK_INTERVAL,
  BULK_ADC_CALLBACK_INTERVAL,
  BULK_VELOCITY_CALLBACK_INTERVAL,
};

enum class Response : uint8_t {
  SUCCESS,
  ERROR_UNKNOWN_CODE,
  ERROR_OUT_OF_RANGE,
  ERROR_NO_ITEM,
  ERROR_MOTOR_IS_USED,
};

struct ProtocolFrame
{
  uint32_t id;
  uint8_t dlc;
  uint8_t flags;
  uint8_t data[64];
};

K_MSGQ_DEFINE(robomaster_protocol_msgq, sizeof(ProtocolFrame), FrameQueueLen, 4);
}  // namespace

struct RoboMasterProtocolService::Impl
{
  const struct device * control_can;
  MotorControlService & motor_control;
  const std::array<const struct device *, MotorCount> & motors;
  SoftwareWatchdog board_watchdog{1000};
  int cmd_filter_id = -1;
  int heartbeat_filter_id = -1;
  struct k_thread thread;
  k_tid_t thread_id = nullptr;
  K_KERNEL_STACK_MEMBER(stack, ThreadStackSize);

  std::array<float, MotorCount> speed_kp{};
  std::array<float, MotorCount> speed_ki{};
  std::array<float, MotorCount> speed_kd{};
  std::array<float, MotorCount> speed_max{};
  std::array<float, MotorCount> position_kp{};
  std::array<float, MotorCount> position_ki{};
  std::array<float, MotorCount> position_kd{};
  std::array<float, MotorCount> position_max{};

  Impl(
    const struct device * control_can, MotorControlService & motor_control,
    const std::array<const struct device *, MotorCount> & motors)
  : control_can(control_can), motor_control(motor_control), motors(motors)
  {
    speed_max.fill(1.0f);
    position_max.fill(1.0f);
  }

  static void rx_callback(
    const struct device *, struct can_frame * frame, void * user_data)
  {
    auto * self = static_cast<Impl *>(user_data);
    ProtocolFrame queued = {
      .id = frame->id,
      .dlc = frame->dlc,
      .flags = frame->flags,
    };
    std::memcpy(queued.data, frame->data, can_dlc_to_bytes(frame->dlc));
    const int ret = k_msgq_put(&robomaster_protocol_msgq, &queued, K_NO_WAIT);
    if (ret != 0) {
      LOG_WRN("Protocol RX queue full");
    }
    ARG_UNUSED(self);
  }

  static void thread_entry(void * arg1, void *, void *)
  {
    static_cast<Impl *>(arg1)->run();
  }

  bool connected() const
  {
    return board_watchdog.alive();
  }

  template <typename... Args>
  int send_serialized(Args &&... args)
  {
    struct can_frame frame = {
      .id = CallbackId + ProtocolOffset,
      .flags = 0U,
    };
    static_assert(fibril::serde::get_size<Args...>() <= 64);
    const size_t len = fibril::serde::write(frame.data, std::forward<Args>(args)...);
    frame.dlc = can_bytes_to_dlc(len);
    if (len > CAN_MAX_DLEN) {
      frame.flags |= CAN_FRAME_FDF;
    }
    return can_send(control_can, &frame, K_MSEC(10), nullptr, nullptr);
  }

  void run()
  {
    int64_t last_heartbeat_ms = 0;
    int64_t last_status_ms = 0;

    while (true) {
      ProtocolFrame frame = {};
      const int ret = k_msgq_get(&robomaster_protocol_msgq, &frame, K_MSEC(20));
      if (ret == 0) {
        handle_frame(frame);
      }

      const int64_t now = k_uptime_get();
      if (connected() && (now - last_heartbeat_ms >= 100)) {
        (void)send_serialized(Command::TEST);
        last_heartbeat_ms = now;
      }
      if (connected() && (now - last_status_ms >= 1000)) {
        send_status();
        last_status_ms = now;
      }
      if (!connected()) {
        motor_control.reset_all();
      }
    }
  }

  void handle_frame(const ProtocolFrame & frame)
  {
    const auto len = can_dlc_to_bytes(frame.dlc);
    if (frame.id == HeartbeatId) {
      if (fibril::serde::read_callback(
            frame.data, len,
            [this](fibril::serde::constant<Command, Command::TEST>) { board_watchdog.kick(); })) {
        return;
      }
    }

    if (frame.id != (CmdId + ProtocolOffset)) {
      return;
    }

    if (fibril::serde::read_callback(
          frame.data, len,
          [this](fibril::serde::constant<Command, Command::RESET>) { motor_control.reset_all(); })) {
      return;
    }

    if (fibril::serde::read_callback(
          frame.data, len,
          [this](fibril::serde::constant<Command, Command::TEST>, uint8_t version, TestData test) {
            ARG_UNUSED(version);
            board_watchdog.reset();
            (void)send_serialized(Command::TEST, ProtocolVersion, test);
          })) {
      return;
    }

    if (!connected()) {
      return;
    }

    if (fibril::serde::read_callback(
          frame.data, len,
          [this](fibril::serde::constant<Command, Command::ENABLE_FEATURE>, MotorID id) {
            handle_enable_feature(id);
          },
          [this](fibril::serde::constant<Command, Command::DISABLE_FEATURE>, MotorID id) {
            handle_disable_feature(id);
          },
          [this](fibril::serde::constant<Command, Command::CMD_SET_DUTY>, MotorID id, float value) {
            handle_set_duty(id, value);
          },
          [this](fibril::serde::constant<Command, Command::CMD_SET_SPEED>, MotorID id, float value) {
            handle_set_speed(id, value);
          },
          [this](
            fibril::serde::constant<Command, Command::CMD_SET_POSITION>, MotorID id, float value) {
            handle_set_position(id, value);
          },
          [this](
            fibril::serde::constant<Command, Command::CMD_SET_POSITION_WITH_SPEED>, MotorID id,
            float value) { handle_set_position_speed(id, value); },
          [this](
            fibril::serde::constant<Command, Command::CMD_SET_ENCODER_VALUE>, MotorID id,
            float value) { handle_set_encoder_value(id, value); },
          [this](fibril::serde::constant<Command, Command::CMD_RESET_SAFETY>, MotorID id) {
            handle_reset_safety(id);
          },
          [this](fibril::serde::constant<Command, Command::CMD_RESET_ENCODER_VALUE>, MotorID id) {
            handle_reset_encoder(id);
          })) {
      board_watchdog.kick();
      return;
    }

    if (handle_config(frame.data, len)) {
      board_watchdog.kick();
    }
  }

  bool validate_motor_id(MotorID id, Command cmd)
  {
    if (id >= MotorCount) {
      (void)send_serialized(cmd, id, Response::ERROR_OUT_OF_RANGE);
      return false;
    }
    return true;
  }

  bool ensure_claimed(MotorID id, Command cmd)
  {
    if (!motor_control.claim(id, OwnerToken)) {
      (void)send_serialized(cmd, id, Response::ERROR_MOTOR_IS_USED);
      return false;
    }
    return true;
  }

  void handle_enable_feature(MotorID id)
  {
    if (!validate_motor_id(id, Command::ENABLE_FEATURE)) {
      return;
    }
    if (!ensure_claimed(id, Command::ENABLE_FEATURE)) {
      return;
    }
    (void)send_serialized(Command::ENABLE_FEATURE, id, Response::SUCCESS);
  }

  void handle_disable_feature(MotorID id)
  {
    if (!validate_motor_id(id, Command::DISABLE_FEATURE)) {
      return;
    }
    motor_control.release(id, OwnerToken);
    (void)send_serialized(Command::DISABLE_FEATURE, id, Response::SUCCESS);
  }

  void handle_set_duty(MotorID id, float value)
  {
    if (!validate_motor_id(id, Command::CMD_SET_DUTY) ||
        !ensure_claimed(id, Command::CMD_SET_DUTY)) {
      return;
    }
    (void)motor_control.set_duty(id, value);
  }

  void handle_set_speed(MotorID id, float value)
  {
    if (!validate_motor_id(id, Command::CMD_SET_SPEED) ||
        !ensure_claimed(id, Command::CMD_SET_SPEED)) {
      return;
    }
    (void)motor_control.set_speed(id, value);
  }

  void handle_set_position(MotorID id, float value)
  {
    if (!validate_motor_id(id, Command::CMD_SET_POSITION) ||
        !ensure_claimed(id, Command::CMD_SET_POSITION)) {
      return;
    }
    (void)motor_control.set_position(id, value);
  }

  void handle_set_position_speed(MotorID id, float value)
  {
    if (!validate_motor_id(id, Command::CMD_SET_POSITION_WITH_SPEED) ||
        !ensure_claimed(id, Command::CMD_SET_POSITION_WITH_SPEED)) {
      return;
    }
    (void)motor_control.set_position_speed(id, value);
  }

  void handle_set_encoder_value(MotorID id, float value)
  {
    if (!validate_motor_id(id, Command::CMD_SET_ENCODER_VALUE)) {
      return;
    }
    const int ret = motor_control.set_encoder_value(id, value);
    (void)send_serialized(
      Command::CMD_SET_ENCODER_VALUE, id,
      ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
  }

  void handle_reset_safety(MotorID id)
  {
    if (!validate_motor_id(id, Command::CMD_RESET_SAFETY)) {
      return;
    }
    const int ret = motor_control.reset_safety(id);
    (void)send_serialized(
      Command::CMD_RESET_SAFETY, id, ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
  }

  void handle_reset_encoder(MotorID id)
  {
    if (!validate_motor_id(id, Command::CMD_RESET_ENCODER_VALUE)) {
      return;
    }
    const int ret = motor_control.reset_encoder(id);
    (void)send_serialized(
      Command::CMD_RESET_ENCODER_VALUE, id,
      ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
  }

  bool handle_config(const uint8_t * data, size_t len)
  {
    return fibril::serde::read_callback(
      data, len,
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::CMD_TIMEOUT>, uint16_t timeout_ms) {
        board_watchdog.set_timeout(timeout_ms);
        (void)send_serialized(Command::CMD_SUB_SETTING, Config::CMD_TIMEOUT, Response::SUCCESS);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::SPEED_CONTROLLER_KP>, MotorID id, float value) {
        set_speed_pid_gain(id, value, speed_ki.at(id), speed_kd.at(id), speed_max.at(id), Config::SPEED_CONTROLLER_KP);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::SPEED_CONTROLLER_KI>, MotorID id, float value) {
        set_speed_pid_gain(id, speed_kp.at(id), value, speed_kd.at(id), speed_max.at(id), Config::SPEED_CONTROLLER_KI);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::SPEED_CONTROLLER_KD>, MotorID id, float value) {
        set_speed_pid_gain(id, speed_kp.at(id), speed_ki.at(id), value, speed_max.at(id), Config::SPEED_CONTROLLER_KD);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::SPEED_CONTROLLER_MAX>, MotorID id, float value) {
        set_speed_pid_gain(id, speed_kp.at(id), speed_ki.at(id), speed_kd.at(id), value, Config::SPEED_CONTROLLER_MAX);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::POSITION_CONTROLLER_KP>, MotorID id, float value) {
        set_position_pid_gain(id, value, position_ki.at(id), position_kd.at(id), position_max.at(id), Config::POSITION_CONTROLLER_KP);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::POSITION_CONTROLLER_KI>, MotorID id, float value) {
        set_position_pid_gain(id, position_kp.at(id), value, position_kd.at(id), position_max.at(id), Config::POSITION_CONTROLLER_KI);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::POSITION_CONTROLLER_KD>, MotorID id, float value) {
        set_position_pid_gain(id, position_kp.at(id), position_ki.at(id), value, position_max.at(id), Config::POSITION_CONTROLLER_KD);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::POSITION_CONTROLLER_MAX>, MotorID id, float value) {
        set_position_pid_gain(id, position_kp.at(id), position_ki.at(id), position_kd.at(id), value, Config::POSITION_CONTROLLER_MAX);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::DUTY_DELTA_MAX>, MotorID id, float value) {
        if (id >= MotorCount) {
          (void)send_serialized(Command::CMD_SUB_SETTING, Config::DUTY_DELTA_MAX, id, Response::ERROR_OUT_OF_RANGE);
          return;
        }
        const int ret = motor_control.configure_duty_acceleration_limit(id, value);
        (void)send_serialized(
          Command::CMD_SUB_SETTING, Config::DUTY_DELTA_MAX, id,
          ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::POSITION_TOLERANCE>, MotorID id, float value) {
        if (id >= MotorCount) {
          (void)send_serialized(Command::CMD_SUB_SETTING, Config::POSITION_TOLERANCE, id, Response::ERROR_OUT_OF_RANGE);
          return;
        }
        motor_control.set_position_tolerance(id, value);
        (void)send_serialized(Command::CMD_SUB_SETTING, Config::POSITION_TOLERANCE, id, Response::SUCCESS);
      },
      [this](
        fibril::serde::constant<Command, Command::CMD_SUB_SETTING>,
        fibril::serde::constant<Config, Config::SPEED_LOW_PASS_FILTER_COEFFICIENT>, MotorID id,
        float value) {
        if (id >= MotorCount) {
          (void)send_serialized(Command::CMD_SUB_SETTING, Config::SPEED_LOW_PASS_FILTER_COEFFICIENT, id, Response::ERROR_OUT_OF_RANGE);
          return;
        }
        const int ret = motor_control.configure_speed_filter(id, value);
        (void)send_serialized(
          Command::CMD_SUB_SETTING, Config::SPEED_LOW_PASS_FILTER_COEFFICIENT, id,
          ret == 0 ? Response::SUCCESS : Response::ERROR_OUT_OF_RANGE);
      });
  }

  void set_speed_pid_gain(
    MotorID id, float kp, float ki, float kd, float max_output, Config config)
  {
    if (id >= MotorCount) {
      (void)send_serialized(Command::CMD_SUB_SETTING, config, id, Response::ERROR_OUT_OF_RANGE);
      return;
    }
    speed_kp.at(id) = kp;
    speed_ki.at(id) = ki;
    speed_kd.at(id) = kd;
    speed_max.at(id) = max_output;
    const int ret = motor_control.configure_speed_pid(id, kp, ki, kd, max_output);
    (void)send_serialized(
      Command::CMD_SUB_SETTING, config, id, ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
  }

  void set_position_pid_gain(
    MotorID id, float kp, float ki, float kd, float max_output, Config config)
  {
    if (id >= MotorCount) {
      (void)send_serialized(Command::CMD_SUB_SETTING, config, id, Response::ERROR_OUT_OF_RANGE);
      return;
    }
    position_kp.at(id) = kp;
    position_ki.at(id) = ki;
    position_kd.at(id) = kd;
    position_max.at(id) = max_output;
    const int ret = motor_control.configure_position_pid(id, kp, ki, kd, max_output);
    (void)send_serialized(
      Command::CMD_SUB_SETTING, config, id, ret == 0 ? Response::SUCCESS : Response::ERROR_UNKNOWN_CODE);
  }

  void send_status()
  {
    std::bitset<16> connected_bits;
    std::bitset<16> safe_bits;

    for (std::size_t index = 0; index < motors.size(); ++index) {
      struct motor_feedback feedback = {};
      if (motor_get_feedback(motors.at(index), &feedback) == 0) {
        connected_bits.set(index, feedback.online && !feedback.stale);
        safe_bits.set(index, true);
      }
    }

    (void)send_serialized(Command::CALLBACK_MOTOR_STATUS, connected_bits, safe_bits);
  }
};

RoboMasterProtocolService::RoboMasterProtocolService(
  const struct device * control_can, MotorControlService & motor_control,
  const std::array<const struct device *, MotorCount> & motors)
: impl_(new Impl(control_can, motor_control, motors))
{
}

int RoboMasterProtocolService::start()
{
  const struct can_filter cmd_filter = {
    .id = CmdId + ProtocolOffset,
    .mask = CAN_STD_ID_MASK,
    .flags = 0U,
  };
  const struct can_filter heartbeat_filter = {
    .id = HeartbeatId,
    .mask = CAN_STD_ID_MASK,
    .flags = 0U,
  };

  impl_->cmd_filter_id =
    can_add_rx_filter(impl_->control_can, Impl::rx_callback, impl_, &cmd_filter);
  if (impl_->cmd_filter_id < 0) {
    return impl_->cmd_filter_id;
  }

  impl_->heartbeat_filter_id =
    can_add_rx_filter(impl_->control_can, Impl::rx_callback, impl_, &heartbeat_filter);
  if (impl_->heartbeat_filter_id < 0) {
    return impl_->heartbeat_filter_id;
  }

  const int ret = can_start(impl_->control_can);
  if ((ret < 0) && (ret != -EALREADY)) {
    return ret;
  }

  impl_->thread_id = k_thread_create(
    &impl_->thread, impl_->stack, K_KERNEL_STACK_SIZEOF(impl_->stack), Impl::thread_entry, impl_,
    nullptr, nullptr, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
  if (impl_->thread_id == nullptr) {
    return -EINVAL;
  }
  k_thread_name_set(impl_->thread_id, "rm_proto");
  return 0;
}

bool RoboMasterProtocolService::connected() const
{
  return impl_->connected();
}

}  // namespace fibril
