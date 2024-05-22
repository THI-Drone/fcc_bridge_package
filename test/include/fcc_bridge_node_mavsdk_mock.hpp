//
// Created by Johan <job8197@thi.de> on 20.05.2024.
//

#ifndef THI_DRONE_WS_FCC_BRIDGE_NODE_MAVSDK_MOCK_HPP
#define THI_DRONE_WS_FCC_BRIDGE_NODE_MAVSDK_MOCK_HPP

// Libc header
#include <optional>

// Mavsdk header
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace fcc_bridge::test {

extern std::optional<mavsdk::Telemetry::GpsInfo> fake_gps_info;

extern std::optional<mavsdk::Telemetry::Position> fake_gps_position;

}  // namespace fcc_bridge::test

#endif  // THI_DRONE_WS_FCC_BRIDGE_NODE_MAVSDK_MOCK_HPP
