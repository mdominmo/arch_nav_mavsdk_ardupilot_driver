#include "mavsdk_platform_driver.hpp"

#include <chrono>
#include <mutex>
#include <optional>
#include <stdexcept>

#include <mavsdk/log_callback.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "arch_nav/context/vehicle_context.hpp"

#include "mavsdk_command_dispatcher.hpp"

using namespace mavsdk;

namespace arch_nav_mavsdk {
namespace {

const mavsdk::log::Callback kSilentMavsdkLogCallback =
    [](mavsdk::log::Level, const std::string&, const std::string&, int) {
      return true;
    };

void silence_mavsdk_logs() {
  mavsdk::log::subscribe(kSilentMavsdkLogCallback);
}

arch_nav::constants::ControlState control_state_from_flight_mode(
    mavsdk::Telemetry::FlightMode mode) {
  using arch_nav::constants::ControlState;
  using FlightMode = mavsdk::Telemetry::FlightMode;

  switch (mode) {
    case FlightMode::Ready:
    case FlightMode::Takeoff:
    case FlightMode::Hold:
    case FlightMode::Mission:
    case FlightMode::ReturnToLaunch:
    case FlightMode::Land:
    case FlightMode::FollowMe:
    case FlightMode::Offboard:
      return ControlState::KERNEL_CONTROLLED;
    case FlightMode::Unknown:
      return ControlState::UNKNOWN;
    default:
      return ControlState::EXTERNAL;
  }
}

}  // namespace

struct MavsdkPlatformDriver::Internals {
  MavsdkCommandDispatcher dispatcher;

  Internals(std::shared_ptr<System> system, const MavsdkConfig& config)
      : dispatcher(system, config) {}
};

MavsdkPlatformDriver::MavsdkPlatformDriver(const MavsdkConfig& config)
    : config_(config) {
  silence_mavsdk_logs();

  Mavsdk::Configuration mavsdk_config{ComponentType::GroundStation};
  mavsdk_ = std::make_unique<Mavsdk>(mavsdk_config);

  auto conn_result = mavsdk_->add_any_connection(config_.connection_url);
  if (conn_result != ConnectionResult::Success) {
    throw std::runtime_error(
        "Connection failed: " + std::to_string(static_cast<int>(conn_result)));
  }

  auto system_opt = mavsdk_->first_autopilot(config_.discover_timeout_s);
  if (!system_opt) {
    throw std::runtime_error("No autopilot found within timeout");
  }
  system_ = system_opt.value();

  internals_ = std::make_unique<Internals>(system_, config_);
}

MavsdkPlatformDriver::~MavsdkPlatformDriver() {
  stop();
}

arch_nav::platform::ICommandDispatcher& MavsdkPlatformDriver::dispatcher() {
  return internals_->dispatcher;
}

void MavsdkPlatformDriver::start(arch_nav::context::VehicleContext& context,
                                  std::chrono::milliseconds update_period) {
  silence_mavsdk_logs();
  running_ = true;
  telemetry_thread_ = std::thread([this, &context, update_period] {
    Telemetry telemetry(system_);

    std::mutex buf_mutex;
    std::optional<arch_nav::vehicle::GlobalPosition> buf_gp;
    std::optional<arch_nav::vehicle::Kinematics> buf_kin;
    std::optional<arch_nav::vehicle::VehicleStatus> buf_status;
    std::optional<arch_nav::constants::ArmState> buf_arm_state;
    std::optional<arch_nav::constants::ControlState> buf_control_state;

    const auto position_handle = telemetry.subscribe_position(
        [&buf_mutex, &buf_gp](Telemetry::Position pos) {
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_gp.emplace(
              pos.latitude_deg, pos.longitude_deg,
              static_cast<double>(pos.absolute_altitude_m));
        });

    const auto velocity_ned_handle = telemetry.subscribe_velocity_ned(
        [&buf_mutex, &buf_kin, &telemetry](Telemetry::VelocityNed vel) {
          auto pos = telemetry.position();
          auto heading = telemetry.heading();
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_kin.emplace(
              0.0, 0.0, 0.0,
              vel.north_m_s, vel.east_m_s, vel.down_m_s,
              0.0, 0.0, 0.0,
              pos.latitude_deg, pos.longitude_deg,
              static_cast<double>(pos.absolute_altitude_m),
              heading.heading_deg);
        });

    const auto armed_handle = telemetry.subscribe_armed(
        [this, &buf_mutex, &buf_status, &buf_arm_state, &buf_control_state](bool armed) {
          // Ensure landing completion is signaled before propagating DISARMED.
          if (!armed && internals_) {
            internals_->dispatcher.notify_landing_complete_if_pending();
          }
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_arm_state = armed ? arch_nav::constants::ArmState::ARMED
                                : arch_nav::constants::ArmState::DISARMED;
          if (buf_control_state.has_value()) {
            buf_status.emplace(*buf_control_state, *buf_arm_state);
          }
        });

    const auto flight_mode_handle = telemetry.subscribe_flight_mode(
        [&buf_mutex, &buf_status, &buf_arm_state, &buf_control_state](
            Telemetry::FlightMode mode) {
          std::lock_guard<std::mutex> lock(buf_mutex);
          buf_control_state = control_state_from_flight_mode(mode);
          if (buf_arm_state.has_value()) {
            buf_status.emplace(*buf_control_state, *buf_arm_state);
          }
        });

    while (running_) {
      std::this_thread::sleep_for(update_period);

      std::optional<arch_nav::vehicle::GlobalPosition> gp;
      std::optional<arch_nav::vehicle::Kinematics> kin;
      std::optional<arch_nav::vehicle::VehicleStatus> status;
      {
        std::lock_guard<std::mutex> lock(buf_mutex);
        gp.swap(buf_gp);
        kin.swap(buf_kin);
        status.swap(buf_status);
      }

      if (gp) context.update(*gp);
      if (kin) context.update(*kin);
      if (status) context.update(*status);
    }

    telemetry.unsubscribe_position(position_handle);
    telemetry.unsubscribe_velocity_ned(velocity_ned_handle);
    telemetry.unsubscribe_armed(armed_handle);
    telemetry.unsubscribe_flight_mode(flight_mode_handle);
  });
}

void MavsdkPlatformDriver::stop() {
  running_ = false;

  if (internals_) {
    internals_->dispatcher.stop();
  }

  if (telemetry_thread_.joinable()) {
    telemetry_thread_.join();
  }

  internals_.reset();
  system_.reset();
  mavsdk_.reset();
}

}  // namespace arch_nav_mavsdk
