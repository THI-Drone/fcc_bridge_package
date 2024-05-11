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
        safety_limits::HARD_MAX_SPEED_LIMIT_MPS <
            this->safety_limits->max_speed_mps) {
        RCLCPP_WARN(
            this->get_safety_logger(),
            "Got invalid speed: %f outside of range (0;%f]. Using Internal "
            "limit: %f",
            static_cast<double>(this->safety_limits->max_speed_mps),
            static_cast<double>(safety_limits::HARD_MAX_SPEED_LIMIT_MPS),
            static_cast<double>(safety_limits::HARD_MAX_SPEED_LIMIT_MPS));
        this->safety_limits->max_speed_mps =
            safety_limits::HARD_MAX_SPEED_LIMIT_MPS;
    }
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
                "In an invalid state for a uav health check! Exiting...");
            this->exit_process_on_error();
        case INTERNAL_STATE::ARMED:
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
                     "Flight State check not implemented!");
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

    return true;  // TODO: implement actual geofence check
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
