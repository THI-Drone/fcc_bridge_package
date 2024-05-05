//
// Created by Johan <job8197@thi.de> on 04.05.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {

void FCCBridgeNode::check_gps_state() {
    RCLCPP_DEBUG(this->get_logger(), "Checking GPS state");

    // Ensuring there is valid gps info present
    if (!this->last_fcc_gps_info.has_value()) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "No cached GPS info found, getting an update from the FCC");
        this->get_gps_telemetry();
    }

    switch (this->get_internal_state()) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            throw std::runtime_error(std::string(__func__) +
                                     " was called in an invalid state: " +
                                     this->internal_state_to_str());
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Verify that there is at least a 2D fix. (The No GPS antenna case
            // will be handled in the fallthrough cases)
            if (this->last_fcc_gps_info->fix_type ==
                mavsdk::Telemetry::FixType::NoFix) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_logger(),
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
                RCLCPP_FATAL(this->get_logger(),
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

    // Check that the current coordinate is inside the geofence.
    if (!this->check_point_in_geofence(this->last_fcc_position.value())) {
        // The current point is outside the geofence. If we are in
    }

    RCLCPP_INFO(this->get_logger(), "GPS state is O.K.");
}

void FCCBridgeNode::check_flight_state() {
    RCLCPP_WARN_ONCE(this->get_logger(), "Flight State check not implemented!");
}

void FCCBridgeNode::check_battery_state() {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Battery State check not implemented!");
}

void FCCBridgeNode::check_rc_state() {
    RCLCPP_WARN_ONCE(this->get_logger(), "RC State check not implemented!");
}

void FCCBridgeNode::check_uav_health() {
    RCLCPP_DEBUG(this->get_logger(), "Checking UAV health state");

    // Ensuring there is valid uav health present
    if (!this->last_fcc_health.has_value()) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "No cached uav health found, getting an update from the FCC");
        this->get_uav_health();
    }
    switch (this->get_internal_state()) {
            // This function should never be called in these states
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            throw std::runtime_error(std::string(__func__) +
                                     " was called in an invalid state: " +
                                     this->internal_state_to_str());
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Verify that UAV position and home position are ok
            if (!this->last_fcc_health->is_local_position_ok ||
                !this->last_fcc_health->is_global_position_ok ||
                !this->last_fcc_health->is_home_position_ok) {
                // This is unrecoverable. If this happens on the ground nothing
                // there is no danger. If the drone is airborne manual control
                // is required.
                RCLCPP_FATAL(this->get_logger(),
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
                RCLCPP_FATAL(this->get_logger(),
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

    RCLCPP_INFO(this->get_logger(), "UAV health is O.K.");
}

bool FCCBridgeNode::check_point_in_geofence(const double latitude_deg,
                                            const double longitude_deg,
                                            const float relative_altitude_m) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Geofence check fully not implemented!");
    RCLCPP_DEBUG(this->get_logger(),
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
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
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
            RCLCPP_ERROR(this->get_logger(),
                         "Tried to check if point is inside geofence, while no "
                         "geofence was configured");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            return false;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    return true;  // TODO: implement actual geofence check
}

bool FCCBridgeNode::check_speed(const float speed_mps) {
    (void)speed_mps;
    RCLCPP_WARN_ONCE(this->get_logger(), "Speed check not implemented!");
    return true;
}

}  // namespace fcc_bridge