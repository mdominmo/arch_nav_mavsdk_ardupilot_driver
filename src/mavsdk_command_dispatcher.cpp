#include "mavsdk_command_dispatcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

using namespace mavsdk;
using arch_nav::constants::CommandResponse;
using arch_nav::constants::ReferenceFrame;

namespace arch_nav_mavsdk {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kYawToleranceRad = 3.0 * kPi / 180.0;
constexpr std::chrono::seconds kYawOperationTimeout{45};

double normalize_angle_0_2pi(double angle_rad) {
  angle_rad = std::fmod(angle_rad, 2.0 * kPi);
  if (angle_rad < 0.0) angle_rad += 2.0 * kPi;
  return angle_rad;
}

double shortest_angular_error(double target_rad, double current_rad) {
  return std::atan2(
      std::sin(target_rad - current_rad),
      std::cos(target_rad - current_rad));
}

}  // namespace

MavsdkCommandDispatcher::MavsdkCommandDispatcher(
    std::shared_ptr<System> system,
    const MavsdkConfig& config)
    : action_(std::make_unique<Action>(system)),
      mavlink_passthrough_(std::make_unique<MavlinkPassthrough>(system)),
      mission_(std::make_unique<Mission>(system)),
      param_(std::make_unique<Param>(system)),
      telemetry_(std::make_unique<Telemetry>(system)),
      config_(config) {
  param_->set_param_int("MIS_TKO_LAND_REQ", 0);
}

MavsdkCommandDispatcher::~MavsdkCommandDispatcher() {
  stop();
}

void MavsdkCommandDispatcher::clear_subscriptions() {
  if (flight_mode_handle_) {
    telemetry_->unsubscribe_flight_mode(*flight_mode_handle_);
    flight_mode_handle_.reset();
  }
  if (landed_state_handle_) {
    telemetry_->unsubscribe_landed_state(*landed_state_handle_);
    landed_state_handle_.reset();
  }
  if (mission_progress_handle_) {
    mission_->unsubscribe_mission_progress(*mission_progress_handle_);
    mission_progress_handle_.reset();
  }
}

CommandResponse MavsdkCommandDispatcher::execute_arm() {
  auto result = action_->arm();
  if (result != Action::Result::Success) return CommandResponse::DENIED;
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_disarm() {
  auto result = action_->disarm();
  if (result != Action::Result::Success) return CommandResponse::DENIED;
  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_takeoff(
    double height, ReferenceFrame frame,
    std::function<void()> on_complete,
    arch_nav::execution::TakeoffExecutionState& state) {
  if (frame != ReferenceFrame::LOCAL_NED) return CommandResponse::DENIED;

  stop();

  const auto pos = telemetry_->position();
  if (!std::isfinite(pos.latitude_deg) ||
      !std::isfinite(pos.longitude_deg) ||
      !std::isfinite(pos.relative_altitude_m)) {
    return CommandResponse::DENIED;
  }

  // ArduPilot Guided takeoff may interpret altitude as ABOVE_TERRAIN when
  // rangefinder terrain source is active. Force WPNav rangefinder usage off
  // for takeoff so command altitude stays in the home-relative frame.
  const auto wpnav_rfnd_res = param_->set_param_int("WPNAV_RFND_USE", 0);
  const auto wp_rfnd_res = param_->set_param_int("WP_RFND_USE", 0);
  const auto is_param_ok = [](Param::Result r) {
    return r == Param::Result::Success || r == Param::Result::DoesNotExist ||
           r == Param::Result::WrongType || r == Param::Result::TypeMismatch ||
           r == Param::Result::TypeUnsupported;
  };
  if (!is_param_ok(wpnav_rfnd_res) || !is_param_ok(wp_rfnd_res)) {
    std::cerr << "[arch_nav_mavsdk] warning: could not force RFND takeoff frame "
              << "(WPNAV_RFND_USE=" << static_cast<int>(wpnav_rfnd_res)
              << ", WP_RFND_USE=" << static_cast<int>(wp_rfnd_res) << ")"
              << std::endl;
  }

  // ArduCopter TAKEOFF commands are expected while in GUIDED control.
  // Set mode explicitly to GUIDED before sending NAV_TAKEOFF.
  constexpr float kMavModeFlagCustomModeEnabled = 1.0f;
  constexpr float kCopterGuidedModeNumber = 4.0f;
  MavlinkPassthrough::CommandLong set_mode_cmd{};
  set_mode_cmd.target_sysid = mavlink_passthrough_->get_target_sysid();
  set_mode_cmd.target_compid = mavlink_passthrough_->get_target_compid();
  set_mode_cmd.command = MAV_CMD_DO_SET_MODE;
  set_mode_cmd.param1 = kMavModeFlagCustomModeEnabled;
  set_mode_cmd.param2 = kCopterGuidedModeNumber;
  set_mode_cmd.param3 = 0.0f;
  set_mode_cmd.param4 = 0.0f;
  set_mode_cmd.param5 = 0.0f;
  set_mode_cmd.param6 = 0.0f;
  set_mode_cmd.param7 = 0.0f;
  const auto mode_result = mavlink_passthrough_->send_command_long(set_mode_cmd);
  if (mode_result != MavlinkPassthrough::Result::Success) {
    return CommandResponse::DENIED;
  }

  MavlinkPassthrough::CommandInt takeoff_cmd{};
  takeoff_cmd.target_sysid = mavlink_passthrough_->get_target_sysid();
  takeoff_cmd.target_compid = mavlink_passthrough_->get_target_compid();
  takeoff_cmd.command = MAV_CMD_NAV_TAKEOFF;
  takeoff_cmd.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  takeoff_cmd.param1 = 0.0f;
  takeoff_cmd.param2 = 0.0f;
  takeoff_cmd.param3 = 0.0f;
  takeoff_cmd.param4 = std::numeric_limits<float>::quiet_NaN();
  takeoff_cmd.x = static_cast<int32_t>(std::lround(pos.latitude_deg * 1e7));
  takeoff_cmd.y = static_cast<int32_t>(std::lround(pos.longitude_deg * 1e7));
  // Command takeoff as a delta from current relative altitude so a request
  // "10 m" means climb 10 m from current position.
  const double target_rel_alt_m = static_cast<double>(pos.relative_altitude_m) + height;
  takeoff_cmd.z = static_cast<float>(target_rel_alt_m);

  auto takeoff_result = mavlink_passthrough_->send_command_int(takeoff_cmd);
  if (takeoff_result != MavlinkPassthrough::Result::Success) {
    return CommandResponse::DENIED;
  }

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this, &state, height, pos] {
    constexpr double kTakeoffToleranceM = 0.15;
    const double target_rel_alt_m = static_cast<double>(pos.relative_altitude_m) + height;
    int stable_samples = 0;
    while (!stop_requested_) {
      auto pos = telemetry_->position();
      const double relative_altitude = static_cast<double>(pos.relative_altitude_m);
      state.current_altitude.store(relative_altitude);

      if (std::isfinite(relative_altitude) &&
          std::fabs(relative_altitude - target_rel_alt_m) <= kTakeoffToleranceM) {
        ++stable_samples;
      } else {
        stable_samples = 0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (stable_samples >= 3) {
        break;
      }
    }

    // Phase 1: release driver resources
    clear_subscriptions();
    resources_released_ = true;

    // Phase 2: notify external consumer
    if (!stop_requested_) {
      std::function<void()> cb;
      {
        std::lock_guard<std::mutex> lock(complete_mutex_);
        cb = std::move(on_complete_);
      }
      if (cb) cb();
    }
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_land(
    std::function<void()> on_complete) {
  stop();

  auto result = action_->land();
  if (result != Action::Result::Success) return CommandResponse::DENIED;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;
  land_in_progress_ = true;
  land_completion_notified_ = false;
  land_on_ground_detected_ = false;

  monitor_thread_ = std::thread([this] {
    wait_for_landed_and_notify();
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_change_yaw(
    double new_yaw,
    ReferenceFrame frame,
    std::function<void()> on_complete) {
  stop();

  // Heading can be temporarily unavailable right after mode transitions.
  // Retry briefly before rejecting the command.
  double current_heading_rad = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 30; ++i) {
    const auto heading = telemetry_->heading();
    if (std::isfinite(heading.heading_deg)) {
      current_heading_rad = heading.heading_deg * kPi / 180.0;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!std::isfinite(current_heading_rad)) {
    std::cerr << "[arch_nav_mavsdk] change_yaw denied: heading is not finite"
              << std::endl;
    return CommandResponse::DENIED;
  }

  double target_heading_rad = current_heading_rad;
  bool relative_yaw = false;
  double requested_relative_yaw_rad = 0.0;

  switch (frame) {
    case ReferenceFrame::LOCAL_NED:
      target_heading_rad = normalize_angle_0_2pi(new_yaw);
      break;
    case ReferenceFrame::LOCAL_ENU:
      target_heading_rad = normalize_angle_0_2pi(kPi / 2.0 - new_yaw);
      break;
    case ReferenceFrame::BODY_FCS:
      relative_yaw = true;
      requested_relative_yaw_rad = new_yaw;
      target_heading_rad = normalize_angle_0_2pi(current_heading_rad + new_yaw);
      break;
    default:
      return CommandResponse::DENIED;
  }

  // Ensure Guided mode so Copter processes CONDITION_YAW reliably.
  constexpr float kMavModeFlagCustomModeEnabled = 1.0f;
  constexpr float kCopterGuidedModeNumber = 4.0f;
  MavlinkPassthrough::CommandLong set_mode_cmd{};
  set_mode_cmd.target_sysid = mavlink_passthrough_->get_target_sysid();
  set_mode_cmd.target_compid = mavlink_passthrough_->get_target_compid();
  set_mode_cmd.command = MAV_CMD_DO_SET_MODE;
  set_mode_cmd.param1 = kMavModeFlagCustomModeEnabled;
  set_mode_cmd.param2 = kCopterGuidedModeNumber;
  set_mode_cmd.param3 = 0.0f;
  set_mode_cmd.param4 = 0.0f;
  set_mode_cmd.param5 = 0.0f;
  set_mode_cmd.param6 = 0.0f;
  set_mode_cmd.param7 = 0.0f;
  const auto mode_result = mavlink_passthrough_->send_command_long(set_mode_cmd);
  if (mode_result != MavlinkPassthrough::Result::Success) {
    std::cerr << "[arch_nav_mavsdk] change_yaw denied: set_mode GUIDED failed with result="
              << static_cast<int>(mode_result) << std::endl;
    return CommandResponse::DENIED;
  }

  const auto send_condition_yaw_to_heading = [this](
      double heading_rad,
      bool is_relative,
      double relative_delta_rad) {
    const float target_yaw_deg = static_cast<float>(
        normalize_angle_0_2pi(heading_rad) * 180.0 / kPi);

    MavlinkPassthrough::CommandLong yaw_cmd{};
    yaw_cmd.target_sysid = mavlink_passthrough_->get_target_sysid();
    yaw_cmd.target_compid = mavlink_passthrough_->get_target_compid();
    yaw_cmd.command = MAV_CMD_CONDITION_YAW;
    if (is_relative) {
      yaw_cmd.param1 = static_cast<float>(std::fabs(relative_delta_rad) * 180.0 / kPi);
      yaw_cmd.param3 = (relative_delta_rad >= 0.0) ? 1.0f : -1.0f;
      yaw_cmd.param4 = 1.0f;  // relative heading mode
    } else {
      yaw_cmd.param1 = target_yaw_deg;  // absolute heading [deg]
      yaw_cmd.param3 = 0.0f;            // shortest path
      yaw_cmd.param4 = 0.0f;            // absolute heading mode
    }
    yaw_cmd.param2 = 0.0f;            // use default slew rate
    yaw_cmd.param5 = 0.0f;
    yaw_cmd.param6 = 0.0f;
    yaw_cmd.param7 = 0.0f;

    const auto cmd_result = mavlink_passthrough_->send_command_long(yaw_cmd);
    if (cmd_result != MavlinkPassthrough::Result::Success) {
      std::cerr << "[arch_nav_mavsdk] change_yaw denied: CONDITION_YAW failed with result="
                << static_cast<int>(cmd_result) << std::endl;
      return false;
    }
    return true;
  };
  if (!send_condition_yaw_to_heading(
          target_heading_rad, relative_yaw, requested_relative_yaw_rad)) {
    return CommandResponse::DENIED;
  }

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread(
      [this,
       target_heading_rad,
       relative_yaw,
       requested_relative_yaw_rad] {
    // Prevent immediate callback re-entrancy during task start.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto started_at = std::chrono::steady_clock::now();
    bool completed = false;
    double accumulated_turn_rad = 0.0;
    double last_heading_rad = std::numeric_limits<double>::quiet_NaN();
    while (!stop_requested_) {
      if (std::chrono::steady_clock::now() - started_at > kYawOperationTimeout) {
        std::cerr << "[arch_nav_mavsdk] change_yaw timeout after "
                  << kYawOperationTimeout.count() << "s" << std::endl;
        break;
      }

      const auto heading = telemetry_->heading();
      if (std::isfinite(heading.heading_deg)) {
        const double current_heading_rad =
            heading.heading_deg * kPi / 180.0;

        if (relative_yaw) {
          if (std::isfinite(last_heading_rad)) {
            accumulated_turn_rad += std::fabs(
                shortest_angular_error(current_heading_rad, last_heading_rad));
          }
          last_heading_rad = current_heading_rad;
          if (accumulated_turn_rad + kYawToleranceRad >=
              std::fabs(requested_relative_yaw_rad)) {
            completed = true;
            break;
          }
        } else {
          const double error = std::fabs(shortest_angular_error(
              target_heading_rad, current_heading_rad));
          if (error <= kYawToleranceRad) {
            completed = true;
            break;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!completed && !stop_requested_) {
      std::cerr << "[arch_nav_mavsdk] change_yaw finished without reaching target"
                << std::endl;
    }

    clear_subscriptions();
    resources_released_ = true;

    if (!stop_requested_) {
      std::function<void()> cb;
      {
        std::lock_guard<std::mutex> lock(complete_mutex_);
        cb = std::move(on_complete_);
      }
      if (cb) cb();
    }
  });

  return CommandResponse::ACCEPTED;
}

void MavsdkCommandDispatcher::complete_landing_if_pending() {
  if (!land_in_progress_.load() ||
      land_completion_notified_.load() ||
      !land_on_ground_detected_.load()) {
    return;
  }

  std::function<void()> cb;
  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    if (!land_in_progress_.load() ||
        land_completion_notified_.load() ||
        !land_on_ground_detected_.load()) {
      return;
    }

    // Phase 1: release driver resources
    land_completion_notified_ = true;
    land_in_progress_ = false;
    resources_released_ = true;

    // Phase 2: prepare external notification
    cb = std::move(on_complete_);
  }
  if (cb) cb();
}

void MavsdkCommandDispatcher::notify_landing_complete_if_pending() {
  complete_landing_if_pending();
}

CommandResponse MavsdkCommandDispatcher::execute_waypoint_following(
    std::vector<arch_nav::vehicle::Waypoint> waypoints,
    ReferenceFrame frame,
    std::function<void()> on_complete,
    arch_nav::execution::WaypointExecutionState& state) {
  if (frame != ReferenceFrame::GLOBAL_WGS84) return CommandResponse::DENIED;

  stop();

  Mission::MissionPlan plan;
  for (const auto& wp : waypoints) {
    Mission::MissionItem item{};
    item.latitude_deg        = wp.lat;
    item.longitude_deg       = wp.lon;
    item.relative_altitude_m = static_cast<float>(wp.alt);
    item.speed_m_s           = config_.default_speed_m_s;
    item.is_fly_through      = true;
    plan.mission_items.push_back(item);
  }

  if (!plan.mission_items.empty()) {
    plan.mission_items.back().is_fly_through = false;
  }

  auto upload_result = mission_->upload_mission(plan);
  if (upload_result != Mission::Result::Success) return CommandResponse::DENIED;

  std::this_thread::sleep_for(std::chrono::milliseconds(config_.mission_upload_delay_ms));

  auto start_result = mission_->start_mission();
  if (start_result != Mission::Result::Success) return CommandResponse::DENIED;

  {
    std::lock_guard<std::mutex> lock(complete_mutex_);
    on_complete_ = std::move(on_complete);
  }
  stop_requested_ = false;

  monitor_thread_ = std::thread([this, &state] {
    auto progress_updated = std::make_shared<std::atomic<bool>>(false);
    auto last_current = std::make_shared<std::atomic<int>>(0);
    auto last_total = std::make_shared<std::atomic<int>>(0);

    mission_progress_handle_ = mission_->subscribe_mission_progress(
        [progress_updated, last_current, last_total](Mission::MissionProgress progress) {
          last_current->store(progress.current);
          last_total->store(progress.total);
          progress_updated->store(true);
        });

    while (!stop_requested_) {
      if (progress_updated->exchange(false)) {
        state.current_waypoint.store(last_current->load());
        state.total_waypoints.store(last_total->load());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto finished = mission_->is_mission_finished();
      if (finished.first == Mission::Result::Success && finished.second) {
        // Phase 1: release driver resources
        clear_subscriptions();
        resources_released_ = true;

        // Phase 2: notify external consumer
        if (!stop_requested_) {
          std::function<void()> cb;
          {
            std::lock_guard<std::mutex> lock(complete_mutex_);
            cb = std::move(on_complete_);
          }
          if (cb) cb();
        }
        break;
      }
    }
  });

  return CommandResponse::ACCEPTED;
}

CommandResponse MavsdkCommandDispatcher::execute_trajectory(
    std::vector<arch_nav::vehicle::TrajectoryPoint> /*trajectory*/,
    ReferenceFrame /*frame*/,
    std::function<void()> /*on_complete*/,
    arch_nav::execution::TrajectoryExecutionState& /*state*/) {
  return CommandResponse::NOT_SUPPORTED;
}

void MavsdkCommandDispatcher::stop() {
  stop_requested_ = true;
  clear_subscriptions();

  if (monitor_thread_.joinable()) {
    const bool is_monitor_thread =
        monitor_thread_.get_id() == std::this_thread::get_id();
    if (is_monitor_thread || resources_released_.load())
      monitor_thread_.detach();
    else
      monitor_thread_.join();
  }

  resources_released_ = false;
  std::lock_guard<std::mutex> lock(complete_mutex_);
  on_complete_ = nullptr;
  land_in_progress_ = false;
  land_completion_notified_ = false;
  land_on_ground_detected_ = false;
}

void MavsdkCommandDispatcher::wait_for_landed_and_notify() {
  landed_state_handle_ = telemetry_->subscribe_landed_state(
      [this](Telemetry::LandedState landed_state) {
        if (landed_state == Telemetry::LandedState::OnGround) {
          land_on_ground_detected_ = true;
        }
      });

  while (!stop_requested_) {
    if (land_on_ground_detected_.load()) {
      complete_landing_if_pending();
      clear_subscriptions();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  clear_subscriptions();
}

}  // namespace arch_nav_mavsdk
