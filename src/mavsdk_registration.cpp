#include "arch_nav/driver/driver_registry.hpp"
#include "mavsdk_config_loader.hpp"
#include "mavsdk_platform_driver.hpp"

namespace {

struct MavsdkDriverRegistration {
  MavsdkDriverRegistration() {
    arch_nav::platform::DriverRegistry::instance().register_driver(
        "mavsdk",
        [](const std::string& config_path) {
          auto config = config_path.empty()
              ? arch_nav_mavsdk::MavsdkConfig{}
              : arch_nav_mavsdk::MavsdkConfigLoader::load_from(config_path);
          return std::make_unique<arch_nav_mavsdk::MavsdkPlatformDriver>(config);
        });
  }
};

static MavsdkDriverRegistration registration;

}  // namespace
