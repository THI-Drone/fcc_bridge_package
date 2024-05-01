//
// Created by Johan on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <chrono>

namespace fcc_bridge {
FCCBridgeNode::FCCBridgeNode()
    : CommonNode("fcc_bridge"), internal_state(INTERNAL_STATE::STARTING_UP) {
    // Pre-populate the last mission control heartbeat
    this->last_mission_control_heatbeat.time_stamp = this->now();

    // Setup ROS objects such as timer, publishers etc.
    this->setup_ros();
    if (this->internal_state == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup ROS! Exiting...");
        std::exit(EXIT_FAILURE);
    }
    this->internal_state = INTERNAL_STATE::ROS_SET_UP;
    RCLCPP_INFO(this->get_logger(), "Transitioning into ROS_SET_UP state");

    // Setup MAVSDk objects such as system, telemetry etc.
    this->setup_mavsdk();
    if (this->internal_state == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup MAVSDK! Exiting...");
        std::exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->get_logger(), "Transitioning into MAVSDK_SET_UP state");
    this->internal_state = INTERNAL_STATE::MAVSDK_SET_UP;

    // Activating node to signal that it is ready for the safety limits
    this->activate();

    RCLCPP_INFO(this->get_logger(), "Successfully created %s instance",
                this->get_name());
}

void FCCBridgeNode::setup_ros() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up ROS");
    if (this->internal_state != INTERNAL_STATE::STARTING_UP) {
        RCLCPP_ERROR(this->get_logger(),
                     "Repeated try to setup ros components");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

    // Setup publisher
    this->gps_position_publisher =
        this->create_publisher<interfaces::msg::GPSPosition>("uav_gps_position",
                                                             1);

    // Setup subscriber
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>
        subscription_options;
    subscription_options.content_filter_options.filter_expression =
        "sender_id = \"mission_control\"";

    this->mission_control_heartbeat_subscriber =
        this->create_subscription<interfaces::msg::Heartbeat>(
            "heartbeat", 1,
            std::bind(&FCCBridgeNode::mission_control_heartbeat_subscriber_cb,
                      this, std::placeholders::_1),
            subscription_options);

    // Setup 5Hz timer to get telemetry from the FCC
    this->fcc_telemetry_timer_5hz = this->create_wall_timer(
        std::chrono::milliseconds{200},
        std::bind(&FCCBridgeNode::fcc_telemetry_timer_5hz_cb, this));

    // Setup 10Hz timer to get telemetry from the FCC
    this->fcc_telemetry_timer_5hz = this->create_wall_timer(
        std::chrono::milliseconds{100},
        std::bind(&FCCBridgeNode::fcc_telemetry_timer_10hz_cb, this));
}

void FCCBridgeNode::mission_control_heartbeat_subscriber_cb(
    const interfaces::msg::Heartbeat &msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received heartbeat from mission control");
    if (msg.tick <= this->last_mission_control_heatbeat.tick) {
        RCLCPP_ERROR(this->get_logger(),
                     "The received heartbeat is not newer than the last one!");
        this->trigger_rth();
    }
    if (!msg.active) {
    }
}

void FCCBridgeNode::fcc_telemetry_timer_5hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "5Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heatbeat();
}

void FCCBridgeNode::fcc_telemetry_timer_10hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "10Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heatbeat();
    if (!this->get_gps_telemetry()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to get GPS_Telemetry");
        std::exit(EXIT_FAILURE);
    } else {
        // In this case retrieving the gps telemetry was successful and it can
        // be safely accessed.
        interfaces::msg::GPSPosition gps_msg;
        gps_msg.time_stamp = this->now();
        gps_msg.sender_id = this->get_name();
        gps_msg.fix_type =
            FCCBridgeNode::fix_type_mavsdk_to_ros(last_fcc_gps_info->fix_type);
        gps_msg.latitude_deg = last_fcc_position->latitude_deg;
        gps_msg.longitude_deg = last_fcc_position->longitude_deg;
        gps_msg.relative_altitude_m = last_fcc_position->relative_altitude_m;
        this->gps_position_publisher->publish(gps_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published current GPS position");
    }
}

void FCCBridgeNode::check_last_mission_control_heatbeat() {
    // TODO: Check if last heartbeat is not too old.
}
}  // namespace fcc_bridge
