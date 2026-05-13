#ifndef ARCH_NAV_MAVSDK__MAVSDK_COMMAND_DISPATCHER_HPP_
#define ARCH_NAV_MAVSDK__MAVSDK_COMMAND_DISPATCHER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "mavsdk_config.hpp"

namespace arch_nav_mavsdk {

class MavsdkCommandDispatcher : public arch_nav::platform::ICommandDispatcher {
 public:
  explicit MavsdkCommandDispatcher(
      std::shared_ptr<mavsdk::System> system,
      const MavsdkConfig& config);

  ~MavsdkCommandDispatcher() override;

  arch_nav::constants::CommandResponse execute_takeoff(
      double height, arch_nav::constants::ReferenceFrame frame,
      std::function<void()> on_complete,
      arch_nav::report::TakeoffDriverOperationData& driver_data) override;
  arch_nav::constants::CommandResponse execute_land(
      std::function<void()> on_complete) override;
  arch_nav::constants::CommandResponse execute_change_yaw(
      double new_yaw, arch_nav::constants::ReferenceFrame frame,
      std::function<void()> on_complete) override;
  arch_nav::constants::CommandResponse execute_waypoint_following(
      std::vector<arch_nav::vehicle::Waypoint> waypoints,
      arch_nav::constants::ReferenceFrame frame,
      std::function<void()> on_complete,
      arch_nav::report::WaypointDriverOperationData& driver_data) override;
  arch_nav::constants::CommandResponse execute_trajectory(
      std::vector<arch_nav::vehicle::TrajectoryPoint> trajectory,
      arch_nav::constants::ReferenceFrame frame,
      std::function<void()> on_complete) override;
  arch_nav::constants::CommandResponse execute_arm() override;
  arch_nav::constants::CommandResponse execute_disarm() override;
  void stop() override;
  void notify_landing_complete_if_pending();

 private:
  void complete_landing_if_pending();
  void wait_for_landed_and_notify();
  void clear_subscriptions();

  std::unique_ptr<mavsdk::Action>    action_;
  std::unique_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough_;
  std::unique_ptr<mavsdk::Mission>   mission_;
  std::unique_ptr<mavsdk::Param>     param_;
  std::unique_ptr<mavsdk::Telemetry> telemetry_;
  MavsdkConfig config_;

  std::atomic<bool>      stop_requested_{false};
  std::atomic<bool>      resources_released_{false};
  std::function<void()>  on_complete_;
  std::mutex             complete_mutex_;
  std::thread            monitor_thread_;

  std::optional<mavsdk::Telemetry::FlightModeHandle>      flight_mode_handle_;
  std::optional<mavsdk::Telemetry::LandedStateHandle>     landed_state_handle_;
  std::optional<mavsdk::Mission::MissionProgressHandle>    mission_progress_handle_;
  std::atomic<bool>      land_in_progress_{false};
  std::atomic<bool>      land_completion_notified_{false};
  std::atomic<bool>      land_on_ground_detected_{false};
};

}  // namespace arch_nav_mavsdk

#endif  // ARCH_NAV_MAVSDK__DISPATCHERS__MAVSDK_COMMAND_DISPATCHER_HPP_
