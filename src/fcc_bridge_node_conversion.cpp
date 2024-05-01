//
// Created by Johan on 01.05.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {
interfaces::msg::GPSPosition::_fix_type_type
FCCBridgeNode::fix_type_mavsdk_to_ros(
    const mavsdk::Telemetry::FixType &fix_type) {
    switch (fix_type) {
        case mavsdk::Telemetry::FixType::NoFix:
            return interfaces::msg::GPSPosition::NO_FIX;
        case mavsdk::Telemetry::FixType::NoGps:
            return interfaces::msg::GPSPosition::NO_GPS;
        case mavsdk::Telemetry::FixType::Fix2D:
            return interfaces::msg::GPSPosition::FIX_2D;
        case mavsdk::Telemetry::FixType::Fix3D:
            return interfaces::msg::GPSPosition::FIX_3D;
        case mavsdk::Telemetry::FixType::FixDgps:
            return interfaces::msg::GPSPosition::FIX_DGPS;
        case mavsdk::Telemetry::FixType::RtkFloat:
            return interfaces::msg::GPSPosition::RTK_FLOAT;
        case mavsdk::Telemetry::FixType::RtkFixed:
            return interfaces::msg::GPSPosition::RTK_FIXED;
        default:
            throw std::runtime_error(
                std::string("Got invalid fix type ") +
                std::to_string(static_cast<int>(fix_type)));
    }
}
}  // namespace fcc_bridge
