#ifndef ARCH_NAV_MAVSDK__MAVSDK_CONFIG_LOADER_HPP_
#define ARCH_NAV_MAVSDK__MAVSDK_CONFIG_LOADER_HPP_

#include <string>

#include "mavsdk_config.hpp"

namespace arch_nav_mavsdk {

class MavsdkConfigLoader {
 public:
  static MavsdkConfig load_from(const std::string& path);
};

}  // namespace arch_nav_mavsdk

#endif  // ARCH_NAV_MAVSDK__MAVSDK_CONFIG_LOADER_HPP_
