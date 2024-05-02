//
// Created by Johan <job8197@thi.de> on 01.05.2024.
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

interfaces::msg::FlightState::_flight_mode_type
FCCBridgeNode::flight_mode_mavsdk_to_ros(
    const mavsdk::Telemetry::FlightMode &flight_mode) {
    switch (flight_mode) {
        case mavsdk::Telemetry::FlightMode::Unknown:
            return interfaces::msg::FlightState::UNKNOWN;
        case mavsdk::Telemetry::FlightMode::Ready:
            return interfaces::msg::FlightState::READY;
        case mavsdk::Telemetry::FlightMode::Takeoff:
            return interfaces::msg::FlightState::TAKEOFF;
        case mavsdk::Telemetry::FlightMode::Hold:
            return interfaces::msg::FlightState::HOLD;
        case mavsdk::Telemetry::FlightMode::Mission:
            return interfaces::msg::FlightState::MISSION;
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            return interfaces::msg::FlightState::RETURN_TO_LAUNCH;
        case mavsdk::Telemetry::FlightMode::Land:
            return interfaces::msg::FlightState::LAND;
        case mavsdk::Telemetry::FlightMode::Offboard:
            return interfaces::msg::FlightState::OFFBOARD;
        case mavsdk::Telemetry::FlightMode::FollowMe:
            return interfaces::msg::FlightState::FOLLOW_ME;
        case mavsdk::Telemetry::FlightMode::Manual:
            return interfaces::msg::FlightState::MANUAL;
        case mavsdk::Telemetry::FlightMode::Altctl:
            return interfaces::msg::FlightState::ALTITUDE_CONTROL;
        case mavsdk::Telemetry::FlightMode::Posctl:
            return interfaces::msg::FlightState::POSITION_CONTROL;
        case mavsdk::Telemetry::FlightMode::Acro:
            return interfaces::msg::FlightState::ACRO;
        case mavsdk::Telemetry::FlightMode::Stabilized:
            return interfaces::msg::FlightState::STABILIZED;
        case mavsdk::Telemetry::FlightMode::Rattitude:
            return interfaces::msg::FlightState::RATTITUDE;
        default:
            throw std::runtime_error(
                std::string("Got invalid flight mode ") +
                std::to_string(static_cast<int>(flight_mode)));
    }
}
}  // namespace fcc_bridge
