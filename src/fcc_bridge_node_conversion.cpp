//
// Created by Johan <job8197@thi.de> on 01.05.2024.
//

#include "fcc_bridge_node.hpp"

// Helper define to create a switch case to turn an enum meber to a string
#define ENUM_TO_STR(parent_namespace, member) \
    case parent_namespace::member:            \
        return #parent_namespace "::" #member

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
                std::string("Got invalid mavsdk::Telemetry::FixType value ") +
                std::to_string(static_cast<int>(fix_type)));
    }
}

interfaces::msg::FlightState::_mode_type::_mode_type
FCCBridgeNode::flight_mode_mavsdk_to_ros(
    const mavsdk::Telemetry::FlightMode &flight_mode) {
    switch (flight_mode) {
        case mavsdk::Telemetry::FlightMode::Unknown:
            return interfaces::msg::FlightState::_mode_type::UNKNOWN;
        case mavsdk::Telemetry::FlightMode::Ready:
            return interfaces::msg::FlightState::_mode_type::READY;
        case mavsdk::Telemetry::FlightMode::Takeoff:
            return interfaces::msg::FlightState::_mode_type::TAKEOFF;
        case mavsdk::Telemetry::FlightMode::Hold:
            return interfaces::msg::FlightState::_mode_type::HOLD;
        case mavsdk::Telemetry::FlightMode::Mission:
            return interfaces::msg::FlightState::_mode_type::MISSION;
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            return interfaces::msg::FlightState::_mode_type::RETURN_TO_LAUNCH;
        case mavsdk::Telemetry::FlightMode::Land:
            return interfaces::msg::FlightState::_mode_type::LAND;
        case mavsdk::Telemetry::FlightMode::Offboard:
            return interfaces::msg::FlightState::_mode_type::OFFBOARD;
        case mavsdk::Telemetry::FlightMode::FollowMe:
            return interfaces::msg::FlightState::_mode_type::FOLLOW_ME;
        case mavsdk::Telemetry::FlightMode::Manual:
            return interfaces::msg::FlightState::_mode_type::MANUAL;
        case mavsdk::Telemetry::FlightMode::Altctl:
            return interfaces::msg::FlightState::_mode_type::ALTITUDE_CONTROL;
        case mavsdk::Telemetry::FlightMode::Posctl:
            return interfaces::msg::FlightState::_mode_type::POSITION_CONTROL;
        case mavsdk::Telemetry::FlightMode::Acro:
            return interfaces::msg::FlightState::_mode_type::ACRO;
        case mavsdk::Telemetry::FlightMode::Stabilized:
            return interfaces::msg::FlightState::_mode_type::STABILIZED;
        case mavsdk::Telemetry::FlightMode::Rattitude:
            return interfaces::msg::FlightState::_mode_type::RATTITUDE;
        default:
            throw std::runtime_error(
                std::string(
                    "Got invalid mavsdk::Telemetry::FlightMode value ") +
                std::to_string(static_cast<int>(flight_mode)));
    }
}

interfaces::msg::FlightState::_state_type::_state_type
FCCBridgeNode::landed_state_mavsdk_to_ros(
    const mavsdk::Telemetry::LandedState &landed_state) {
    switch (landed_state) {
        case mavsdk::Telemetry::LandedState::Unknown:
            return interfaces::msg::FlightState::_state_type::UNKNOWN;
        case mavsdk::Telemetry::LandedState::OnGround:
            return interfaces::msg::FlightState::_state_type::ON_GROUND;
        case mavsdk::Telemetry::LandedState::InAir:
            return interfaces::msg::FlightState::_state_type::IN_AIR;
        case mavsdk::Telemetry::LandedState::TakingOff:
            return interfaces::msg::FlightState::_state_type::TAKING_OFF;
        case mavsdk::Telemetry::LandedState::Landing:
            return interfaces::msg::FlightState::_state_type::LANDING;
        default:
            throw std::runtime_error(
                std::string(
                    "Got invalid mavsdk::Telemetry::LandedState value ") +
                std::to_string(static_cast<int>(landed_state)));
    }
}

const char *FCCBridgeNode::mavsdk_connection_result_to_str(
    const mavsdk::ConnectionResult &result) {
    switch (result) {
        ENUM_TO_STR(mavsdk::ConnectionResult, Success);
        ENUM_TO_STR(mavsdk::ConnectionResult, Timeout);
        ENUM_TO_STR(mavsdk::ConnectionResult, SocketError);
        ENUM_TO_STR(mavsdk::ConnectionResult, BindError);
        ENUM_TO_STR(mavsdk::ConnectionResult, SocketConnectionError);
        ENUM_TO_STR(mavsdk::ConnectionResult, ConnectionError);
        ENUM_TO_STR(mavsdk::ConnectionResult, NotImplemented);
        ENUM_TO_STR(mavsdk::ConnectionResult, SystemNotConnected);
        ENUM_TO_STR(mavsdk::ConnectionResult, SystemBusy);
        ENUM_TO_STR(mavsdk::ConnectionResult, CommandDenied);
        ENUM_TO_STR(mavsdk::ConnectionResult, DestinationIpUnknown);
        ENUM_TO_STR(mavsdk::ConnectionResult, ConnectionsExhausted);
        ENUM_TO_STR(mavsdk::ConnectionResult, ConnectionUrlInvalid);
        ENUM_TO_STR(mavsdk::ConnectionResult, BaudrateUnknown);
        default:
            throw std::runtime_error(
                std::string("Got invalid mavsdk::ConnectionResult value ") +
                std::to_string(static_cast<int>(result)));
    }
}

const char *FCCBridgeNode::mavsdk_flight_mode_to_str(
    const mavsdk::Telemetry::FlightMode &flight_mode) {
    switch (flight_mode) {
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Unknown);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Ready);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Takeoff);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Hold);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Mission);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, ReturnToLaunch);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Land);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Offboard);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, FollowMe);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Manual);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Altctl);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Posctl);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Acro);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Stabilized);
        ENUM_TO_STR(mavsdk::Telemetry::FlightMode, Rattitude);
        default:
            throw std::runtime_error(
                std::string(
                    "Got invalid mavsdk::Telemetry::FlightMode value ") +
                std::to_string(static_cast<int>(flight_mode)));
    }
}

const char *FCCBridgeNode::mavsdk_landed_state_to_str(
    const mavsdk::Telemetry::LandedState &landed_state) {
    switch (landed_state) {
        ENUM_TO_STR(mavsdk::Telemetry::LandedState, Unknown);
        ENUM_TO_STR(mavsdk::Telemetry::LandedState, OnGround);
        ENUM_TO_STR(mavsdk::Telemetry::LandedState, InAir);
        ENUM_TO_STR(mavsdk::Telemetry::LandedState, TakingOff);
        ENUM_TO_STR(mavsdk::Telemetry::LandedState, Landing);
        default:
            throw std::runtime_error(
                std::string(
                    "Got invalid mavsdk::Telemetry::LandedState value ") +
                std::to_string(static_cast<int>(landed_state)));
    }
}

const char *FCCBridgeNode::mavsdk_mission_result_to_str(
    const mavsdk::Mission::Result &result) {
    switch (result) {
        ENUM_TO_STR(mavsdk::Mission::Result, Unknown);
        ENUM_TO_STR(mavsdk::Mission::Result, Success);
        ENUM_TO_STR(mavsdk::Mission::Result, Error);
        ENUM_TO_STR(mavsdk::Mission::Result, TooManyMissionItems);
        ENUM_TO_STR(mavsdk::Mission::Result, Busy);
        ENUM_TO_STR(mavsdk::Mission::Result, Timeout);
        ENUM_TO_STR(mavsdk::Mission::Result, InvalidArgument);
        ENUM_TO_STR(mavsdk::Mission::Result, Unsupported);
        ENUM_TO_STR(mavsdk::Mission::Result, NoMissionAvailable);
        ENUM_TO_STR(mavsdk::Mission::Result, UnsupportedMissionCmd);
        ENUM_TO_STR(mavsdk::Mission::Result, TransferCancelled);
        ENUM_TO_STR(mavsdk::Mission::Result, NoSystem);
        ENUM_TO_STR(mavsdk::Mission::Result, Next);
        ENUM_TO_STR(mavsdk::Mission::Result, Denied);
        ENUM_TO_STR(mavsdk::Mission::Result, ProtocolError);
        ENUM_TO_STR(mavsdk::Mission::Result, IntMessagesNotSupported);
        default:
            throw std::runtime_error(
                std::string("Got invalid mavsdk::Mission::Result value: ") +
                std::to_string(static_cast<int>(result)));
    }
}

const char *FCCBridgeNode::internal_state_to_str() const {
    switch (this->get_internal_state()) {
        ENUM_TO_STR(INTERNAL_STATE, STARTING_UP);
        ENUM_TO_STR(INTERNAL_STATE, ROS_SET_UP);
        ENUM_TO_STR(INTERNAL_STATE, MAVSDK_SET_UP);
        ENUM_TO_STR(INTERNAL_STATE, WAITING_FOR_ARM);
        ENUM_TO_STR(INTERNAL_STATE, ARMED);
        ENUM_TO_STR(INTERNAL_STATE, WAITING_FOR_COMMAND);
        ENUM_TO_STR(INTERNAL_STATE, FLYING_ACTION);
        ENUM_TO_STR(INTERNAL_STATE, FLYING_MISSION);
        ENUM_TO_STR(INTERNAL_STATE, RETURN_TO_HOME);
        ENUM_TO_STR(INTERNAL_STATE, LANDED);
        ENUM_TO_STR(INTERNAL_STATE, ERROR);
        default:
            throw std::runtime_error(
                std::string("Got invalid INTERNAL_STATE value: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }
}

}  // namespace fcc_bridge
