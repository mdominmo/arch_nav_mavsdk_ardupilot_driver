# arch_nav_mavsdk_ardupilot_driver

![License](https://img.shields.io/github/license/mdominmo/arch_nav_mavsdk_ardupilot_driver) ![Version](https://img.shields.io/github/v/release/mdominmo/arch_nav_mavsdk_ardupilot_driver)

[arch-nav](https://github.com/mdominmo/arch-nav) driver that connects to ArduPilot-based autopilots using [MAVSDK](https://mavsdk.mavlink.io).

## Features

- **Takeoff / Land** via MAVSDK Action plugin
- **Waypoint following** via MAVSDK Mission plugin
- **Yaw control** (absolute LOCAL_NED/LOCAL_ENU and relative BODY_FCS)
- **Arm / Disarm** control
- **Telemetry** feedback (position, kinematics, vehicle status) streamed to the arch-nav kernel
- Pure C++17, no ROS dependency

## ArduPilot-specific behaviour

- Automatically sets GUIDED mode before takeoff and yaw commands (required by ArduPilot).
- Disables rangefinder-based altitude (`WPNAV_RFND_USE`, `WP_RFND_USE`) on startup to prevent altitude confusion.
- Disables mandatory takeoff before land (`MIS_TKO_LAND_REQ=0`) on startup.

## Supported reference frames

| Command | Frames |
|---------|--------|
| Takeoff | `LOCAL_NED` |
| Change yaw | `LOCAL_NED`, `LOCAL_ENU`, `BODY_FCS` |
| Waypoint following | `GLOBAL_WGS84` |

## Configuration

The driver reads an optional YAML config file:

```yaml
# mavsdk_config.yaml
connection_url: "tcp://127.0.0.1:14570"
discover_timeout_s: 5.0
default_speed_m_s: 5.0
mission_upload_delay_ms: 1000
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `connection_url` | string | `"udp://localhost:14581"` | MAVSDK connection string to the autopilot |
| `discover_timeout_s` | double | `15.0` | Seconds to wait for autopilot discovery after connection |
| `default_speed_m_s` | float | `5.0` | Cruise speed used for waypoint missions (m/s) |
| `mission_upload_delay_ms` | int | `1000` | Delay (ms) between mission upload and mission start |

Set the config path in `arch_nav_config.yaml`:

```yaml
driver: "mavsdk"
driver_config_path: "/path/to/mavsdk_config.yaml"
```

If `driver_config_path` is empty, the driver uses its built-in defaults.

### Connection URL examples

| Scenario | `connection_url` |
|----------|-----------------|
| ArduPilot SITL (TCP, default) | `tcp://127.0.0.1:14570` |
| ArduPilot SITL (UDP) | `udp://localhost:14551` |
| Serial link (hardware) | `serial:///dev/ttyUSB0:921600` |

## Prerequisites

- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html) C++ library
- [arch-nav](https://github.com/mdominmo/arch-nav) kernel installed

## Build

This driver is compiled together with the arch-nav kernel. See the [arch-nav build instructions](https://github.com/mdominmo/arch-nav#build-and-install).

The build produces a shared library (`libarch_nav_mavsdk.so`) installed to `<prefix>/lib/arch_nav/drivers/`, which arch-nav loads dynamically at startup.

## Writing your own driver

See the [arch-nav driver guide](https://github.com/mdominmo/arch-nav#writing-a-driver) for how to implement and register a custom driver.
