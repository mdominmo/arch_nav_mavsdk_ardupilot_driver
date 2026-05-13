#ifndef ARCH_NAV_MAVSDK__MAVSDK_PLATFORM_DRIVER_HPP_
#define ARCH_NAV_MAVSDK__MAVSDK_PLATFORM_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <mavsdk/mavsdk.h>

#include "arch_nav/driver/i_platform_driver.hpp"
#include "mavsdk_config.hpp"

namespace arch_nav_mavsdk {

class MavsdkPlatformDriver : public arch_nav::platform::IPlatformDriver {
 public:
  explicit MavsdkPlatformDriver(const MavsdkConfig& config);
  ~MavsdkPlatformDriver() override;

  arch_nav::platform::ICommandDispatcher& dispatcher() override;

  void start(arch_nav::context::VehicleContext& context,
             std::chrono::milliseconds update_period) override;

  void stop() override;

 private:
  MavsdkConfig config_;

  std::unique_ptr<mavsdk::Mavsdk>  mavsdk_;
  std::shared_ptr<mavsdk::System>  system_;

  struct Internals;
  std::unique_ptr<Internals> internals_;

  std::atomic<bool> running_{false};
  std::thread       telemetry_thread_;
};

}  // namespace arch_nav_mavsdk

#endif  // ARCH_NAV_MAVSDK__MAVSDK_PLATFORM_DRIVER_HPP_
