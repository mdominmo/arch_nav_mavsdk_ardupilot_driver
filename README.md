# arch_nav_mavsdk_driver

![License](https://img.shields.io/github/license/mdominmo/arch_nav_mavsdk_driver) ![Version](https://img.shields.io/github/v/release/mdominmo/arch_nav_mavsdk_driver)

[arch-nav](https://github.com/mdominmo/arch-nav) driver that connects to MAVLink-compatible autopilots (PX4, ArduPilot) using [MAVSDK](https://mavsdk.mavlink.io).

## Features

- **Takeoff / Land** via MAVSDK Action plugin
- **Waypoint following** via MAVSDK Mission plugin
- **Arm / Disarm** control
- **Telemetry** feedback (position, kinematics, vehicle status) streamed to the arch-nav kernel
- Pure C++17, no ROS dependency

## Configuration

The driver reads an optional YAML config file:

```yaml
# mavsdk_config.yaml
connection_url: "udp://localhost:14540"
discover_timeout_s: 15.0
default_altitude_m: 10.0
default_speed_m_s: 5.0
```

Set the config path in `arch_nav_config.yaml`:

```yaml
driver: "mavsdk"
driver_config_path: "/path/to/mavsdk_config.yaml"
```

If `driver_config_path` is empty, the driver uses its built-in defaults.

## Prerequisites

- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html) (C++ library)
- [arch-nav](https://github.com/mdominmo/arch-nav) kernel installed

## Build

This driver is compiled together with the arch-nav kernel. See the [arch-nav README](https://github.com/mdominmo/arch-nav#build-and-install) for build instructions.

## Writing your own driver

See the [arch-nav driver guide](https://github.com/mdominmo/arch-nav#writing-a-driver) for how to implement and register a custom driver.
