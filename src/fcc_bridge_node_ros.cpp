//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <chrono>
#include <cinttypes>

// CommonLib header
#include "common_package/topic_names.hpp"

namespace fcc_bridge {

namespace {

// Timer periods
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_5HZ{
    200}; /**< The period of the 5Hz telemetry timer */
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_10HZ{
    100}; /**< The period of the 10Hz telemetry timer */

// Mission control node name
constexpr char const *const MISSION_CONTROL_NODE_NAME = "mission_control";

// Limits
constexpr std::chrono::seconds MAX_UAV_COMMAND_AGE{
    1}; /**< Maximum age of a received UAV command */

}  // namespace

void FCCBridgeNode::set_internal_state(
    const fcc_bridge::FCCBridgeNode::INTERNAL_STATE new_state) {
    // Actually set the state.
    this->internal_state = new_state;

    RCLCPP_INFO(this->get_internal_state_logger(),
                "Switching to new internal_state: %s",
                this->internal_state_to_str());
}

void FCCBridgeNode::setup_ros() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(), "Setting up ROS");
    // Check if the node is in the correct state
    if (this->get_internal_state() != INTERNAL_STATE::STARTING_UP) {
        RCLCPP_ERROR(this->get_internal_state_logger(),
                     "Attempted to setup ROS components more than once");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Setup publisher

    // Create GPSInfo & Position publisher
    this->gps_position_publisher =
        this->create_publisher<interfaces::msg::GPSPosition>(
            common_lib::topic_names::GPSPosition, 1);
    // Create FlightState publisher
    this->flight_state_publisher =
        this->create_publisher<interfaces::msg::FlightState>(
            common_lib::topic_names::FlightState, 1);

    // Create BatteryState publisher
    this->battery_state_publisher =
        this->create_publisher<interfaces::msg::BatteryState>(
            common_lib::topic_names::BatteryState, 1);

    // Create RCState publisher
    this->rc_state_publisher = this->create_publisher<interfaces::msg::RCState>(
        common_lib::topic_names::RCState, 1);

    // Create euler angle publisher
    this->euler_angle_publisher = this->create_publisher<interfaces::msg::Pose>(
        common_lib::topic_names::Pose, 1);

    // Create mission progress publisher
    this->mission_progress_publisher =
        this->create_publisher<interfaces::msg::MissionProgress>(
            common_lib::topic_names::MissionProgress, 1);

    // Create UAV health publisher
    this->uav_health_publisher =
        this->create_publisher<interfaces::msg::UAVHealth>(
            common_lib::topic_names::UAVHealth, 1);

    // Create mission start publisher
    this->mission_start_publisher =
        this->create_publisher<interfaces::msg::MissionStart>(
            common_lib::topic_names::MissionStart, 1);

    // Setup subscriber

    // Create subscription options for heartbeat to only receive mission control
    // heartbeats
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.content_filter_options.filter_expression =
        "sender_id = 'mission_control'";

    // Create Heartbeat subscription
    this->mission_control_heartbeat_subscriber =
        this->create_subscription<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 1,
            std::bind(&FCCBridgeNode::mission_control_heartbeat_subscriber_cb,
                      this, std::placeholders::_1),
            subscription_options);

    // Ensure that the filter is enabled. (Some DDS versions do not support it)
    if (!this->mission_control_heartbeat_subscriber->is_cft_enabled()) {
        RCLCPP_FATAL(this->get_ros_interface_logger(),
                     "Content filtering is not enabled!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Create UAV command subscriber
    this->uav_command_subscriber =
        this->create_subscription<interfaces::msg::UAVCommand>(
            common_lib::topic_names::UAVCommand, 10,
            std::bind(&FCCBridgeNode::uav_command_subscriber_cb, this,
                      std::placeholders::_1));

    // Create UAV waypoint command subscriber
    this->uav_waypoint_command_subscriber =
        this->create_subscription<interfaces::msg::UAVWaypointCommand>(
            common_lib::topic_names::UAVWaypointCommand, 10,
            std::bind(&FCCBridgeNode::uav_waypoint_command_subscriber_cb, this,
                      std::placeholders::_1));

    // Create MissionFinished subscriber
    this->mission_finished_subscriber =
        this->create_subscription<interfaces::msg::MissionFinished>(
            common_lib::topic_names::MissionFinished, 10,
            std::bind(&FCCBridgeNode::mission_finished_cb, this,
                      std::placeholders::_1));

    // Create SafetyLimits subscriber
    this->safety_limits_subscriber =
        this->create_subscription<interfaces::msg::SafetyLimits>(
            common_lib::topic_names::SafetyLimits, 10,
            std::bind(&FCCBridgeNode::safety_limits_cb, this,
                      std::placeholders::_1));

    // Setup 5Hz timer to get telemetry from the FCC
    this->fcc_telemetry_timer_5hz = this->create_wall_timer(
        FCC_TELEMETRY_PERIOD_5HZ,
        std::bind(&FCCBridgeNode::fcc_telemetry_timer_5hz_cb, this));

    // Setup 10Hz timer to get telemetry from the FCC
    this->fcc_telemetry_timer_10hz = this->create_wall_timer(
        FCC_TELEMETRY_PERIOD_10HZ,
        std::bind(&FCCBridgeNode::fcc_telemetry_timer_10hz_cb, this));
}

void FCCBridgeNode::mission_control_heartbeat_subscriber_cb(
    const interfaces::msg::Heartbeat &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received heartbeat from mission control");

    // Check if the heartbeat tick has increased since last time or in the case
    // of an overflow that is equal to 0
    if (msg.tick <= this->last_mission_control_heartbeat.tick &&
        msg.tick != 0) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "The received heartbeat is not newer than the last one! "
                     "Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Set the cached heartbeat message to the current one
    this->last_mission_control_heartbeat = msg;

    // Verify that the received heartbeat is not too old
    this->check_last_mission_control_heartbeat();

    RCLCPP_INFO(this->get_safety_logger(), "Mission control is alive and ok");
}

void FCCBridgeNode::uav_command_subscriber_cb(
    const interfaces::msg::UAVCommand &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received a new UAVCommand message");

    // Ensure that the message is not too old
    if (this->now() - msg.time_stamp > rclcpp::Duration(MAX_UAV_COMMAND_AGE)) {
        RCLCPP_ERROR(
            this->get_safety_logger(),
            "Received a UAVCommand that is too old! Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // TODO: Do check of sender

    switch (msg.type) {
        case interfaces::msg::UAVCommand::TAKE_OFF:
            this->initiate_takeoff(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::FLY_TO_WAYPOINT:
            this->start_flying_to_waypoint(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::LAND:
            this->initiate_land(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::RTH:
            this->initiate_rth();
            break;
        default:
            RCLCPP_ERROR(this->get_safety_logger(),
                         "Got unknown UAVCommand type %d", msg.type);
            switch (this->get_internal_state()) {
                case INTERNAL_STATE::ERROR:
                    // This should never happen as the process should have
                    // exited before
                    throw std::runtime_error(
                        "Received unknown command while in ERROR state");
                case INTERNAL_STATE::STARTING_UP:
                case INTERNAL_STATE::ROS_SET_UP:
                case INTERNAL_STATE::MAVSDK_SET_UP:
                case INTERNAL_STATE::WAITING_FOR_ARM:
                case INTERNAL_STATE::ARMED:
                case INTERNAL_STATE::LANDED:
                    // In these cases the UAV is not airborne so the process
                    // will exit
                    RCLCPP_FATAL(this->get_safety_logger(),
                                 "Drone is still on ground. Exiting...");
                    this->set_internal_state(INTERNAL_STATE::ERROR);
                    this->exit_process_on_error();
                case INTERNAL_STATE::WAITING_FOR_COMMAND:
                case INTERNAL_STATE::FLYING_MISSION:
                case INTERNAL_STATE::LANDING:
                case INTERNAL_STATE::RETURN_TO_HOME:
                    RCLCPP_ERROR(this->get_safety_logger(),
                                 "Triggering RTH...");
                    this->trigger_rth();
                    return;
                default:
                    throw std::runtime_error(
                        std::string("Got invalid value for internal_state: ") +
                        std::to_string(
                            static_cast<int>(this->get_internal_state())));
            }
    }

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Finished handling command message");
}

void FCCBridgeNode::uav_waypoint_command_subscriber_cb(
    const interfaces::msg::UAVWaypointCommand &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received a new UAVWaypointCommand message");

    // Repackage message
    interfaces::msg::UAVCommand uav_command_msg;
    uav_command_msg.type = interfaces::msg::UAVCommand::FLY_TO_WAYPOINT;
    uav_command_msg.sender_id = msg.sender_id;
    uav_command_msg.time_stamp = msg.time_stamp;
    uav_command_msg.speed_m_s = msg.speed_m_s;
    uav_command_msg.waypoint = msg.waypoint;

    // Call the actual command handling
    this->uav_command_subscriber_cb(uav_command_msg);
}

void FCCBridgeNode::mission_finished_cb(
    const interfaces::msg::MissionFinished &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received a new MissionFinished message with sender_id: %s, "
                 "error_code: %" PRIu8 ", reason: %s",
                 msg.sender_id.c_str(), msg.error_code, msg.reason.c_str());

    if (msg.sender_id != MISSION_CONTROL_NODE_NAME) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "Received a MissionFinished message from a non mission "
                     "control node! Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // TODO: Check if mission control is active

    // Verify that we are in the right state
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
            // The UAV has not yet taken off
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "Received a MissionFinished while the UAV has not yet "
                         "taken off! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // The UAV is airborne. This will result in an RTH
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "Received a MissionFinished message while airborne. "
                        "Triggering an RTH...");
            this->trigger_rth();
            return;
        case INTERNAL_STATE::LANDED:
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // This means we have landed
    if (msg.error_code == 0) {
        // Expected mission end
        RCLCPP_INFO(this->get_internal_state_logger(),
                    "Ended mission with success. Ending process in 5 seconds.");
        // TODO: trigger callback to end node in 5 seconds
    } else {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Ended mission with error code set! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }
}

void FCCBridgeNode::safety_limits_cb(const interfaces::msg::SafetyLimits &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received a new SafetyLimits message");
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Safety limits not fully implemented");
    // TODO: Sender check

    // Initialize safety limits
    this->safety_limits.emplace();

    this->safety_limits->max_speed_mps = msg.max_speed_m_s;

    this->validate_safety_limits();

    RCLCPP_INFO(this->get_safety_logger(), "Set safety limits");

    // Go into WAITING_FOR_ARM state
    this->set_internal_state(INTERNAL_STATE::WAITING_FOR_ARM);
}

void FCCBridgeNode::fcc_telemetry_timer_5hz_cb() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "5Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heartbeat();
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "5Hz Telemetry callback function was called in an "
                        "invalid state");
            return;

        // Remaining cases that are allowed:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Send out flight state
    this->send_flight_state();

    // Send out battery state
    this->send_battery_state();

    // Send out RC state
    this->send_rc_state();

    // Check if there is currently a mission running
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            RCLCPP_INFO(this->get_internal_state_logger(),
                        "Not in a state to publish mission progress");
            break;
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
            // Send out mission progress
            this->send_mission_progress();
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Send out UAV health
    this->send_uav_health();

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "5Hz telemetry timer callback successful");
}

void FCCBridgeNode::fcc_telemetry_timer_10hz_cb() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "10Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heartbeat();
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw std::runtime_error(std::string(__func__) +
                                     " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "10Hz Telemetry callback function was called in an "
                        "invalid state");
            return;

        // Remaining cases that are allowed
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Send out GPS Telemetry
    this->send_gps_telemetry();

    // Send out euler angle
    this->send_euler_angle();

    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "10Hz telemetry timer callback successful");
}

FCCBridgeNode::FCCBridgeNode(const std::string &name,
                             const rclcpp::NodeOptions &node_options)
    : CommonNode(name, node_options),
      internal_state(INTERNAL_STATE::STARTING_UP) {
    // Pre-populate the last mission control heartbeat
    this->last_mission_control_heartbeat.time_stamp = this->now();
    this->last_mission_control_heartbeat.tick = 0;

    // Setup ROS objects such as timer, publishers etc.
    this->setup_ros();
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Failed to setup ROS! Exiting...");
        this->exit_process_on_error();
    }

    this->set_internal_state(INTERNAL_STATE::ROS_SET_UP);

    // Setup MAVSDK objects such as system, telemetry etc.
    this->setup_mavsdk();
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Failed to setup MAVSDK! Exiting...");
        this->exit_process_on_error();
    }

    this->set_internal_state(INTERNAL_STATE::MAVSDK_SET_UP);

    // Activating node to signal that it is ready for the safety limits
    this->activate();

    RCLCPP_INFO(this->get_logger(), "Successfully created %s instance",
                this->get_name());
}

}  // namespace fcc_bridge
