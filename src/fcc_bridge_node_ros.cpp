//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <chrono>

namespace fcc_bridge {
FCCBridgeNode::FCCBridgeNode(const std::string &name/*,
                             const rclcpp::NodeOptions &node_options*/)
    : CommonNode(name), internal_state(INTERNAL_STATE::STARTING_UP) {
    // Pre-populate the last mission control heartbeat
    this->last_mission_control_heartbeat.time_stamp = this->now();

    // Setup ROS objects such as timer, publishers etc.
    this->setup_ros();
    if (this->internal_state == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup ROS! Exiting...");
        this->exit_process_on_error();
    }
    this->internal_state = INTERNAL_STATE::ROS_SET_UP;
    RCLCPP_INFO(this->get_logger(), "Transitioning into ROS_SET_UP state");

    // Setup MAVSDk objects such as system, telemetry etc.
    this->setup_mavsdk();
    if (this->internal_state == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup MAVSDK! Exiting...");
        this->exit_process_on_error();
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
    // Check if the node is in the correct state
    if (this->internal_state != INTERNAL_STATE::STARTING_UP) {
        RCLCPP_ERROR(this->get_logger(),
                     "Repeated try to setup ros components");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

    // Setup publisher

    // Create GPSInfo & Position publisher
    this->gps_position_publisher =
        this->create_publisher<interfaces::msg::GPSPosition>("uav_gps_position",
                                                             1);
    // Create FlightState publisher
    this->flight_state_publisher =
        this->create_publisher<interfaces::msg::FlightState>("uav_flight_state",
                                                             1);

    // Create BatteryState publisher
    this->battery_state_publisher =
        this->create_publisher<interfaces::msg::BatteryState>(
            "uav_battery_state", 1);

    // Create RCState publisher
    this->rc_state_publisher =
        this->create_publisher<interfaces::msg::RCState>("uav_rc_state", 1);

    // Setup subscriber

    // Create subscription options for heartbeat to only receive mission control
    // heartbeats
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.content_filter_options.filter_expression =
        "sender_id = 'mission_control'";

    // Create Heartbeat subscription
    this->mission_control_heartbeat_subscriber =
        this->create_subscription<interfaces::msg::Heartbeat>(
            "heartbeat", 1,
            std::bind(&FCCBridgeNode::mission_control_heartbeat_subscriber_cb,
                      this, std::placeholders::_1),
            subscription_options);

    // Ensure that the filter is enabled. (Some DDS versions do not support it)
    if (!this->mission_control_heartbeat_subscriber->is_cft_enabled()) {
        RCLCPP_FATAL(this->get_logger(), "Content filtering is not enabled!");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

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

    // Check if the heartbeat tick has increased since last time
    if (msg.tick <= this->last_mission_control_heartbeat.tick) {
        RCLCPP_ERROR(this->get_logger(),
                     "The received heartbeat is not newer than the last one! "
                     "Triggering RTH...");
        this->trigger_rth();
    }

    // Check if mission control is still active
    if (!msg.active) {
        RCLCPP_ERROR(this->get_logger(),
                     "Mission control node is inactive! Triggering RTH...");
        this->trigger_rth();
    }

    // Set the cached heartbeat message to the current one
    this->last_mission_control_heartbeat = msg;

    // Verify that the received heartbeat is not too old
    this->check_last_mission_control_heatbeat();

    RCLCPP_INFO(this->get_logger(), "Mission control is alive and ok");
}

void FCCBridgeNode::fcc_telemetry_timer_5hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "5Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heatbeat();
    switch (this->internal_state) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            RCLCPP_WARN(this->get_logger(),
                        "5Hz Telemetry callback function was called in an "
                        "invalid state");
            return;
        default:
            break;
    }

    // Update flight state
    this->get_flight_state();
    interfaces::msg::FlightState flight_state_msg;
    flight_state_msg.time_stamp = this->now();
    flight_state_msg.sender_id = this->get_name();
    flight_state_msg.flight_mode = FCCBridgeNode::flight_mode_mavsdk_to_ros(
        this->last_fcc_flight_state.value());
    this->flight_state_publisher->publish(flight_state_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published current Flight State");

    // Update battery state
    this->get_battery_state();
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
    this->battery_state_publisher->publish(battery_state_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published current battery state");

    // Update RC state
    this->get_rc_state();
    interfaces::msg::RCState rc_state_msg;
    rc_state_msg.time_stamp = this->now();
    rc_state_msg.sender_id = this->get_name();
    rc_state_msg.was_available_once =
        this->last_fcc_rc_state->was_available_once;
    rc_state_msg.is_available = this->last_fcc_rc_state->is_available;
    rc_state_msg.signal_strength_percent =
        this->last_fcc_rc_state->signal_strength_percent;
    this->rc_state_publisher->publish(rc_state_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published current RC state");
}

void FCCBridgeNode::fcc_telemetry_timer_10hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "10Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heatbeat();
    switch (this->internal_state) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            RCLCPP_WARN(this->get_logger(),
                        "10Hz Telemetry callback function was called in an "
                        "invalid state");
            return;
        default:
            break;
    }
    this->get_gps_telemetry();
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
    this->gps_position_publisher->publish(gps_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published current GPS position");
}

void FCCBridgeNode::check_last_mission_control_heatbeat() {
    // TODO: Check if last heartbeat is not too old.
}
}  // namespace fcc_bridge
