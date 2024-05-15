//
// Created by Johan <job8197@thi.de> on 04.05.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {

void FCCBridgeNode::check_telemetry_result(
    const mavsdk::Telemetry::Result &result, const char *const telemetry_type) {
    if (result != mavsdk::Telemetry::Result::Success) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Failed to set rate for %s with result: %s! Exiting...",
                     telemetry_type,
                     FCCBridgeNode::mavsdk_telemetry_result_to_str(result));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }
    RCLCPP_DEBUG(this->get_safety_logger(), "Successfully set rate for %s",
                 telemetry_type);
}

void FCCBridgeNode::mavsdk_rth_cb(const mavsdk::Action::Result &result) {
    RCLCPP_DEBUG(this->get_safety_logger(),
                 "Return to launch action callback triggered");

    if (result != mavsdk::Action::Result::Success) {
        // In this case something went wrong. Nothing left but to exit.
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Return to launch failed! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    RCLCPP_INFO(this->get_safety_logger(), "Return to home successful!");
    this->set_internal_state(INTERNAL_STATE::LANDED);

    // TODO: Disarm
}

void FCCBridgeNode::validate_safety_limits() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Validating safety limits");
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Safety limit validation is not fully implemented");

    if (!this->safety_limits.has_value()) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "Safety limits is not initialized");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Check speed limit
    if (this->safety_limits->max_speed_mps <= 0 ||
        SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS <
            this->safety_limits->max_speed_mps) {
        RCLCPP_WARN(
            this->get_safety_logger(),
            "Got invalid speed: %fm/s outside of range (0;%f]. Using Internal "
            "limit: %f",
            static_cast<double>(this->safety_limits->max_speed_mps),
            static_cast<double>(SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS),
            static_cast<double>(SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS));
        this->safety_limits->max_speed_mps =
            SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS;
    }

    // Check minimum state of charge
    if (this->safety_limits->min_soc < SafetyLimits::HARD_MIN_SOC) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Got invalid minimum state of charge: %f%%, Using Internal "
                    "limit: %f",
                    static_cast<double>(this->safety_limits->min_soc),
                    static_cast<double>(SafetyLimits::HARD_MIN_SOC));
        this->safety_limits->min_soc = SafetyLimits::HARD_MIN_SOC;
    }

    // Check maximum altitude
    if (SafetyLimits::HARD_MAX_HEIGHT_M < this->safety_limits->max_height_m) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Got invalid maximum height: %fm, Using Internal limit: %f",
                    static_cast<double>(this->safety_limits->max_height_m),
                    static_cast<double>(SafetyLimits::HARD_MAX_HEIGHT_M));
        this->safety_limits->max_height_m = SafetyLimits::HARD_MAX_HEIGHT_M;
    }

    // Check geofence
    if (this->safety_limits->geofence.get_polygon_point_count() < 3) {
        RCLCPP_ERROR(
            this->get_safety_logger(),
            "Got an effective polygon with less then 3 points! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_DEBUG(this->get_safety_logger(),
                 "Validating safety limits successful");
}

void FCCBridgeNode::check_gps_state() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Checking GPS state");

    // Ensuring there is valid gps info present
    if (!this->last_fcc_gps_info.has_value()) {
        RCLCPP_DEBUG(
            this->get_safety_logger(),
            "No cached GPS info found, getting an update from the FCC");
        this->get_gps_telemetry();
    }

    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
            // This function should never be called in these states
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(
                this->get_internal_state_logger(),
                "In an invalid state for a gps state check! Exiting...");
            this->exit_process_on_error();
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Verify that there is at least a 2D fix. (The No GPS antenna case
            // will be handled in the fallthrough cases)
            if (this->last_fcc_gps_info->fix_type ==
                mavsdk::Telemetry::FixType::NoFix) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Lost GPS fix in armed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            [[fallthrough]];
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::LANDED:
            // Verify that the FCC has an GPS installed
            if (this->last_fcc_gps_info->fix_type ==
                mavsdk::Telemetry::FixType::NoGps) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_safety_logger(),
                             "The FCC has no GPS installed! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
            // This function should never be called in these states
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "In an invalid state to check GPS! Exiting...");
            this->exit_process_on_error();
        case INTERNAL_STATE::MAVSDK_SET_UP:
            break;
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::LANDED:
            // Check that the current coordinate is inside the geofence.
            if (!this->check_point_in_geofence(
                    this->last_fcc_position.value())) {
                // The current point is outside the geofence.
                // TODO
            }
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    RCLCPP_INFO(this->get_safety_logger(), "GPS state is O.K.");
}

void FCCBridgeNode::check_flight_state() {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Flight State check not fully implemented!");
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
            // This function should never be called in these states
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "In an invalid state to check GPS! Exiting...");
            this->exit_process_on_error();
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::LANDED:
            if (!this->last_fcc_landed_state.has_value()) {
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got no cached landed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Checking Landed state value: %s",
                         FCCBridgeNode::mavsdk_landed_state_to_str(
                             this->last_fcc_landed_state.value()));
            switch (this->last_fcc_landed_state.value()) {
                case mavsdk::Telemetry::LandedState::Unknown:
                case mavsdk::Telemetry::LandedState::OnGround:
                    // Unknown is acceptable as we should still be on ground
                    break;
                case mavsdk::Telemetry::LandedState::TakingOff:
                case mavsdk::Telemetry::LandedState::InAir:
                case mavsdk::Telemetry::LandedState::Landing:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV is airborne while in state %s! "
                                 "Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                default:
                    throw std::runtime_error(
                        std::string("Got unknown "
                                    "mavsdk::Telemetry::LandedState value: ") +
                        std::to_string(static_cast<int>(this->last_fcc_landed_state.value())));
            }
            break;
        case INTERNAL_STATE::ARMED:
            if (!this->last_fcc_landed_state.has_value()) {
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got no cached landed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Checking Landed state value: %s",
                         FCCBridgeNode::mavsdk_landed_state_to_str(
                             this->last_fcc_landed_state.value()));
            switch (this->last_fcc_landed_state.value()) {
                case mavsdk::Telemetry::LandedState::Unknown:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV does not know if it is airborne while in state %s! Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                case mavsdk::Telemetry::LandedState::OnGround:
                    // Only OnGround is acceptable
                    break;
                case mavsdk::Telemetry::LandedState::TakingOff:
                case mavsdk::Telemetry::LandedState::InAir:
                case mavsdk::Telemetry::LandedState::Landing:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV is airborne while in state %s! "
                                 "Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                default:
                    throw std::runtime_error(
                        std::string("Got unknown "
                                    "mavsdk::Telemetry::LandedState value: ") +
                        std::to_string(static_cast<int>(
                            this->last_fcc_landed_state.value())));
            }
            break;
        case INTERNAL_STATE::TAKING_OFF:
            if (!this->last_fcc_landed_state.has_value()) {
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got no cached landed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Checking Landed state value: %s",
                         FCCBridgeNode::mavsdk_landed_state_to_str(
                             this->last_fcc_landed_state.value()));
            switch (this->last_fcc_landed_state.value()) {
                case mavsdk::Telemetry::LandedState::Unknown:
                case mavsdk::Telemetry::LandedState::Landing:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV does not know if it is landing while in state %s! Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                case mavsdk::Telemetry::LandedState::OnGround:
                case mavsdk::Telemetry::LandedState::TakingOff:
                case mavsdk::Telemetry::LandedState::InAir:
                    // Only OnGround TakingOff and InAir are acceptable
                    break;
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "Drone is not in a valid LandedState while in state %s! Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                default:
                    throw std::runtime_error(
                        std::string("Got unknown "
                                    "mavsdk::Telemetry::LandedState value: ") +
                        std::to_string(static_cast<int>(
                            this->last_fcc_landed_state.value())));
            }
            break;
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
            if (!this->last_fcc_landed_state.has_value()) {
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got no cached landed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Checking Landed state value: %s",
                         FCCBridgeNode::mavsdk_landed_state_to_str(
                             this->last_fcc_landed_state.value()));
            switch (this->last_fcc_landed_state.value()) {
                case mavsdk::Telemetry::LandedState::Unknown:
                case mavsdk::Telemetry::LandedState::Landing:
                case mavsdk::Telemetry::LandedState::TakingOff:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV is not fully airborne while in state %s! Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                case mavsdk::Telemetry::LandedState::OnGround:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV is on ground while in state %s! Exiting...",
                                 this->internal_state_to_str());
                    this->set_internal_state(INTERNAL_STATE::ERROR);
                    this->exit_process_on_error();
                case mavsdk::Telemetry::LandedState::InAir:
                    // Only InAir is acceptable
                    break;
                default:
                    throw std::runtime_error(
                        std::string("Got unknown "
                                    "mavsdk::Telemetry::LandedState value: ") +
                        std::to_string(static_cast<int>(
                            this->last_fcc_landed_state.value())));
            }
            break;
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            if (!this->last_fcc_landed_state.has_value()) {
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got no cached landed state! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Checking Landed state value: %s",
                         FCCBridgeNode::mavsdk_landed_state_to_str(
                             this->last_fcc_landed_state.value()));
            switch (this->last_fcc_landed_state.value()) {
                case mavsdk::Telemetry::LandedState::Unknown:
                case mavsdk::Telemetry::LandedState::TakingOff:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "UAV is not in a valid LandedState while in state %s! Triggering RTH...",
                                 this->internal_state_to_str());
                    this->trigger_rth();
                    return;
                case mavsdk::Telemetry::LandedState::OnGround:
                case mavsdk::Telemetry::LandedState::InAir:
                case mavsdk::Telemetry::LandedState::Landing:
                    // Only OnGround InAir Landing is acceptable
                    break;
                default:
                    throw std::runtime_error(
                        std::string("Got unknown "
                                    "mavsdk::Telemetry::LandedState value: ") +
                        std::to_string(static_cast<int>(
                            this->last_fcc_landed_state.value())));
            }
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    RCLCPP_INFO(this->get_safety_logger(), "FlightState check successful");
}

void FCCBridgeNode::check_battery_state() {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Battery State check not implemented!");
}

void FCCBridgeNode::check_rc_state() {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "RC State check not implemented!");
}

void FCCBridgeNode::check_uav_health() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Checking UAV health state");

    // Ensuring there is valid uav health present
    if (!this->last_fcc_health.has_value()) {
        RCLCPP_DEBUG(
            this->get_safety_logger(),
            "No cached uav health found, getting an update from the FCC");
        this->get_uav_health();
    }
    switch (this->get_internal_state()) {
            // This function should never be called in these states
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(
                this->get_internal_state_logger(),
                "In an invalid state for a uav health check! Exiting...");
            this->exit_process_on_error();
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Verify that UAV position and home position are ok
            if (!this->last_fcc_health->is_local_position_ok ||
                !this->last_fcc_health->is_global_position_ok ||
                !this->last_fcc_health->is_home_position_ok) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_safety_logger(),
                             "FCC cannot determine its own position any more "
                             "or has lost its home position! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            [[fallthrough]];
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::LANDED:
            // Verify that the base state of the drone is ok
            if (!this->last_fcc_health->is_gyrometer_calibration_ok ||
                !this->last_fcc_health->is_accelerometer_calibration_ok ||
                !this->last_fcc_health->is_magnetometer_calibration_ok) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_safety_logger(),
                             "UAV sensors are not calibrated! Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    RCLCPP_INFO(this->get_safety_logger(), "UAV health is O.K.");
}

bool FCCBridgeNode::check_point_in_geofence(const double latitude_deg,
                                            const double longitude_deg,
                                            const float relative_altitude_m) {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Geofence check fully not implemented!");
    RCLCPP_DEBUG(this->get_safety_logger(),
                 "Checking whether point (lat: %f°\tlon: %f°\trel alt: %fm) is "
                 "inside the geofence",
                 latitude_deg, longitude_deg,
                 static_cast<double>(relative_altitude_m));
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            // In these states a geofence should have been configured. This is a
            // sanity check that it was actually configured
            if (this->safety_limits.has_value()) {
                // Every thing ok proceed to the actual geofence check
                break;
            }
            RCLCPP_WARN(this->get_safety_logger(),
                        "Safety limits not configured!");
            [[fallthrough]];
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
            // In these states there is no geofence configured
            RCLCPP_ERROR(this->get_safety_logger(),
                         "Tried to check if point is inside geofence, while no "
                         "geofence was configured");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            return false;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Check that the waypoint values are finite floats
    if (!std::isfinite(latitude_deg)) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Latitude is not a finite number");
        return false;
    }
    if (!std::isfinite(longitude_deg)) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Longitude is not a finite number");
        return false;
    }
    if (!std::isfinite(relative_altitude_m)) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Altitude is not a finite number");
        return false;
    }

    if (!this->safety_limits->geofence.isIn({latitude_deg, longitude_deg})) {
        RCLCPP_WARN(this->get_safety_logger(), "Point is not inside geofence!");
        return false;
    }

    if (this->safety_limits->max_height_m < relative_altitude_m) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Point is above maximum altitude!");
        return false;
    }

    RCLCPP_DEBUG(this->get_safety_logger(), "Waypoint passed check");

    return true;
}

bool FCCBridgeNode::check_speed(const float speed_mps) {
    (void)speed_mps;
    RCLCPP_WARN_ONCE(this->get_safety_logger(), "Speed check not implemented!");
    // Check that the speed is a finite float
    if (!std::isfinite(speed_mps)) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Latitude is not a finite number");
        return false;
    }
    return true;
}

void FCCBridgeNode::check_last_mission_control_heartbeat() {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Heartbeat check not implemented!");
    // TODO: Check if last heartbeat is not too old.
}

}  // namespace fcc_bridge
