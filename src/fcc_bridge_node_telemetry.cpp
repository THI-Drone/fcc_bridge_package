//
// Created by Johan <job8197@thi.de> on 05.05.2024.
//

// FCC Bridge header
#include "fcc_bridge/fcc_bridge_node.hpp"

namespace fcc_bridge {

namespace {

constexpr std::chrono::milliseconds MISSION_PROGRESS_DELAY{1000};

}

void FCCBridgeNode::send_gps_telemetry() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated GPS telemetry and publishing the update");

    this->get_gps_telemetry();

    // Verify the GPS Fix type depending on the internal state
    this->check_gps_state();

    // In this case retrieving the gps telemetry was successful meaning that it
    // can be safely accessed.
    interfaces::msg::GPSPosition gps_msg;
    gps_msg.time_stamp = this->now();
    gps_msg.sender_id = this->get_name();
    gps_msg.fix_type =
        FCCBridgeNode::fix_type_mavsdk_to_ros(last_fcc_gps_info->fix_type);
    gps_msg.latitude_deg = last_fcc_position->latitude_deg;
    gps_msg.longitude_deg = last_fcc_position->longitude_deg;
    gps_msg.relative_altitude_m = last_fcc_position->relative_altitude_m;

    // Publish the message
    this->gps_position_publisher->publish(gps_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current GPS position");
}

void FCCBridgeNode::send_flight_state() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated flight state and publishing the update");

    // Update flight state
    this->get_flight_state();

    // Verify the Flight State depending on the internal state
    this->check_flight_state();

    // If we are in the WAITING_FOR_ARM state and the drone is READY send
    // MissionStart and wait for takeoff
    if (this->get_internal_state() == INTERNAL_STATE::WAITING_FOR_ARM &&
        this->last_fcc_armed_state.value() &&
        this->last_fcc_flight_mode.value() ==
            mavsdk::Telemetry::FlightMode::Hold) {
        RCLCPP_INFO(this->get_internal_state_logger(),
                    "Drone is armed and ready for takeoff. Switching to ARMED "
                    "state and sending MissionStart message");

        this->set_internal_state(INTERNAL_STATE::ARMED);
        interfaces::msg::MissionStart mission_start_msg;
        mission_start_msg.sender_id = this->get_name();
        this->mission_start_publisher->publish(mission_start_msg);

        RCLCPP_DEBUG(this->get_ros_interface_logger(),
                     "MissionStart message send");
    }

    // In this case retrieving the flight state was successful meaning that it
    // can be safely accessed.
    interfaces::msg::FlightState flight_state_msg;
    flight_state_msg.time_stamp = this->now();
    flight_state_msg.sender_id = this->get_name();
    flight_state_msg.mode.mode = FCCBridgeNode::flight_mode_mavsdk_to_ros(
        this->last_fcc_flight_mode.value());
    flight_state_msg.state.state = FCCBridgeNode::landed_state_mavsdk_to_ros(
        this->last_fcc_landed_state.value());
    flight_state_msg.armed = this->last_fcc_armed_state.value();

    // Publish the message
    this->flight_state_publisher->publish(flight_state_msg);

    if (this->get_internal_state() == INTERNAL_STATE::RETURN_TO_HOME &&
        this->last_fcc_landed_state.value() ==
            mavsdk::Telemetry::LandedState::OnGround) {
        RCLCPP_INFO(this->get_safety_logger(),
                    "LandedState indicates on ground. RTH successful");
        this->set_internal_state(INTERNAL_STATE::LANDED);
        this->disarm();
        this->shutdown_timer = this->create_wall_timer(
            std::chrono::seconds{10},
            std::bind(&FCCBridgeNode::shutdown_node, this));
        RCLCPP_WARN(this->get_safety_logger(),
                    "Node shutdown will occur in 10 seconds");
    }

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current flight state");
}

void FCCBridgeNode::send_battery_state() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated battery state and publishing the update");

    // Update battery state
    this->get_battery_state();

    // Verify the Battery State depending on the internal state
    this->check_battery_state();

    // In this case retrieving the battery state was successful meaning that it
    // can be safely accessed.
    interfaces::msg::BatteryState battery_state_msg;
    battery_state_msg.time_stamp = this->now();
    battery_state_msg.sender_id = this->get_name();
    battery_state_msg.id = this->last_fcc_battery_state->id;
    battery_state_msg.temperature_degc =
        this->last_fcc_battery_state->temperature_degc;
    battery_state_msg.voltage_v = this->last_fcc_battery_state->voltage_v;
    battery_state_msg.current_battery_a =
        this->last_fcc_battery_state->current_battery_a;
    battery_state_msg.capacity_consumed_ah =
        this->last_fcc_battery_state->capacity_consumed_ah;
    battery_state_msg.remaining_percent =
        this->last_fcc_battery_state->remaining_percent;

    // Publish the message
    this->battery_state_publisher->publish(battery_state_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current battery state");
}

void FCCBridgeNode::send_rc_state() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated RC state and publishing the update");

    // Update RC state
    this->get_rc_state();

    // Verify the RC State depending on the internal state
    this->check_rc_state();

    // In this case retrieving the RC state was successful meaning that it
    // can be safely accessed.
    interfaces::msg::RCState rc_state_msg;
    rc_state_msg.time_stamp = this->now();
    rc_state_msg.sender_id = this->get_name();
    rc_state_msg.was_available_once =
        this->last_fcc_rc_state->was_available_once;
    rc_state_msg.is_available = this->last_fcc_rc_state->is_available;
    rc_state_msg.signal_strength_percent =
        this->last_fcc_rc_state->signal_strength_percent;

    // Publish the message
    this->rc_state_publisher->publish(rc_state_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current RC state");
}

void FCCBridgeNode::send_euler_angle() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated euler angle and publishing the update");

    // Update euler angle
    this->get_euler_angle();

    // In this case retrieving the euler angle was successful meaning that it
    // can be safely accessed.
    interfaces::msg::Pose euler_angle_msg;
    euler_angle_msg.time_stamp = this->now();
    euler_angle_msg.sender_id = this->get_name();
    euler_angle_msg.roll_deg = this->last_fcc_euler_angle->roll_deg;
    euler_angle_msg.pitch_deg = this->last_fcc_euler_angle->pitch_deg;
    euler_angle_msg.yaw_deg = this->last_fcc_euler_angle->yaw_deg;

    // Publish the message
    this->euler_angle_publisher->publish(euler_angle_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current euler angle");
}

void FCCBridgeNode::send_mission_progress() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated mission progress and publishing the update");

    // Update mission progress
    this->get_mission_progress();

    const std::chrono::milliseconds time_since_mission_start =
        (this->now() - this->last_mission_active_time)
            .to_chrono<std::chrono::milliseconds>();

    if (MISSION_PROGRESS_DELAY < time_since_mission_start) {
        RCLCPP_INFO(this->get_safety_logger(),
                    "Skipping mission progress because mission start is less "
                    "then %" PRId64 "ms ago. Current time since: %" PRId64 "ms",
                    MISSION_PROGRESS_DELAY.count(),
                    time_since_mission_start.count());
        return;
    }

    // Verify of getting the mission progress was successful
    if (this->last_mission_progress->first !=
        mavsdk::Mission::Result::Success) {
        if (this->is_airborne()) {
            // Trigger an RTH on any error
            RCLCPP_ERROR(this->get_safety_logger(),
                         "Failed to get mission progress with error: %s! "
                         "Triggering RTH...",
                         FCCBridgeNode::mavsdk_mission_result_to_str(
                             this->last_mission_progress->first));
            this->trigger_rth();
            return;
        } else {
            // Exit on any error
            RCLCPP_FATAL(
                this->get_safety_logger(),
                "Failed to get mission progress with error: %s! Exiting...",
                FCCBridgeNode::mavsdk_mission_result_to_str(
                    this->last_mission_progress->first));
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    // In this case retrieving the mission progress was successful meaning that
    // it can be safely accessed.
    interfaces::msg::MissionProgress mission_progress_msg;
    mission_progress_msg.time_stamp = this->now();
    mission_progress_msg.sender_id = this->get_name();
    // Check if the mission is finished
    if (this->last_mission_progress->second) {
        // Switch to the waiting for command state
        switch (this->get_internal_state()) {
            case INTERNAL_STATE::ERROR:
                // This should never happen, as the process exits on ERROR state
                throw invalid_state_error(std::string(__func__) +
                                          " called while in ERROR state");
            case INTERNAL_STATE::STARTING_UP:
            case INTERNAL_STATE::ROS_SET_UP:
            case INTERNAL_STATE::MAVSDK_SET_UP:
            case INTERNAL_STATE::WAITING_FOR_ARM:
            case INTERNAL_STATE::ARMED:
            case INTERNAL_STATE::WAITING_FOR_COMMAND:
            case INTERNAL_STATE::RETURN_TO_HOME:
            case INTERNAL_STATE::LANDED:
                RCLCPP_ERROR(this->get_internal_state_logger(),
                             "Getting the mission progress in a non mission "
                             "mode is invalid! Triggering RTH...");
                this->trigger_rth();
                return;
            case INTERNAL_STATE::TAKING_OFF:
            case INTERNAL_STATE::FLYING_MISSION:
                // This means we are ready for the next command
                this->set_internal_state(INTERNAL_STATE::WAITING_FOR_COMMAND);
                break;
            case INTERNAL_STATE::LANDING:
                // This means we landed
                RCLCPP_INFO(this->get_safety_logger(), "Landing complete");
                this->set_internal_state(INTERNAL_STATE::LANDED);
                this->disarm();
                this->shutdown_timer = this->create_wall_timer(
                    std::chrono::seconds{10},
                    std::bind(&FCCBridgeNode::shutdown_node, this));
                RCLCPP_WARN(this->get_safety_logger(),
                            "Node shutdown will occur in 10 seconds");
                break;
            default:
                throw unknown_enum_value_error(
                    std::string("Got invalid value for internal_state: ") +
                    std::to_string(
                        static_cast<int>(this->get_internal_state())));
        }
        mission_progress_msg.progress = 1.0;
        RCLCPP_INFO(this->get_internal_state_logger(),
                    "Mission finished successfully");
    } else {
        mission_progress_msg.progress = 0.0;
    }

    // Publish the message
    this->mission_progress_publisher->publish(mission_progress_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current mission progress");
}

void FCCBridgeNode::send_uav_health() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Getting updated uav health and publishing the update");

    // Update UAV health
    this->get_uav_health();

    // Verify the health depending on the internal state
    this->check_uav_health();

    // In this case retrieving the UAV health was successful meaning that it
    // can be safely accessed.
    interfaces::msg::UAVHealth uav_health_msg;
    uav_health_msg.time_stamp = this->now();
    uav_health_msg.sender_id = this->get_name();
    uav_health_msg.is_gyrometer_calibration_ok =
        this->last_fcc_health->is_gyrometer_calibration_ok;
    uav_health_msg.is_accelerometer_calibration_ok =
        this->last_fcc_health->is_accelerometer_calibration_ok;
    uav_health_msg.is_magnetometer_calibration_ok =
        this->last_fcc_health->is_magnetometer_calibration_ok;
    uav_health_msg.is_local_position_ok =
        this->last_fcc_health->is_local_position_ok;
    uav_health_msg.is_global_position_ok =
        this->last_fcc_health->is_global_position_ok;
    uav_health_msg.is_home_position_ok =
        this->last_fcc_health->is_home_position_ok;
    uav_health_msg.is_armable = this->last_fcc_health->is_armable;

    // Publish the message
    this->uav_health_publisher->publish(uav_health_msg);

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Published current UAV health");
}

}  // namespace fcc_bridge
