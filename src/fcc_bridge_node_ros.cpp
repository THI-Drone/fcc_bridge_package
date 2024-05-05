//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <chrono>

namespace fcc_bridge {

namespace {

// Timer periods
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_5HZ{
    200}; /**< The period of the 5Hz telemetry timer */
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_10HZ{
    100}; /**< The period of the 10Hz telemetry timer */

// Publisher topic names
constexpr char const *const GPS_POSITION_TOPIC_NAME =
    "uav_gps_position"; /**< Topic name for the GPS telemetry */
constexpr char const *const FLIGHT_STATE_TOPIC_NAME =
    "uav_flight_state"; /**< Topic name for the UAV flight state telemetry */
constexpr char const *const BATTERY_STATE_TOPIC_NAME =
    "uav_battery_state"; /**< Topic name for the UAV battery state telemetry */
constexpr char const *const RC_STATE_TOPIC_NAME =
    "uav_rc_state"; /**< Topic name for the RC state telemetry */
constexpr char const *const EULER_ANGLE_TOPIC_NAME =
    "uav_pose"; /**< Topic name for the UAV euler angle telemetry */
constexpr char const *const MISSION_PROGRESS_TOPIC_NAME =
    "uav_mission_progress"; /**< Topic name for the mission progress telemetry
                             */
constexpr char const *const UAV_HEALTH_TOPIC_NAME =
    "uav_health"; /**< Topic name for the uav health telemetry */
constexpr char const *const MISSION_START_TOPIC_NAME =
    "mission_start"; /**< Topic name for the mission start signal send out when
                        the FCC gets armed */

// Subscriber topic names
constexpr char const *const HEARTBEAT_TOPIC_NAME =
    "heartbeat"; /**< Topic name for the heartbeat */
constexpr char const *const UAV_COMMAND_TOPIC_NAME =
    "uav_command"; /**< Topic name for uav commands */

constexpr std::chrono::seconds MAX_UAV_COMMAND_AGE{
    1}; /**< Maximum age of a received UAV command */

}  // namespace

void FCCBridgeNode::set_internal_state(
    const fcc_bridge::FCCBridgeNode::INTERNAL_STATE new_state) {
    // Placeholder for the name of the enum entry name of new state
    const char *state_name = nullptr;

    // Helper define to cut down on boilerplate code
#define HELPER(enum_entry)           \
    case INTERNAL_STATE::enum_entry: \
        state_name = #enum_entry;    \
        break

    // Set state_name to enum entry name of new_state
    switch (new_state) {
        HELPER(STARTING_UP);
        HELPER(ROS_SET_UP);
        HELPER(MAVSDK_SET_UP);
        HELPER(WAITING_FOR_ARM);
        HELPER(ARMED);
        HELPER(WAITING_FOR_COMMAND);
        HELPER(FLYING_ACTION);
        HELPER(FLYING_MISSION);
        HELPER(RETURN_TO_HOME);
        HELPER(LANDED);
        HELPER(ERROR);
        default:
            throw std::runtime_error("Got unexpected state!");
    }

        // Undefine HELPER as it is no longer needed
#undef HELPER

    RCLCPP_INFO(this->get_logger(),
                "Switching to new internal_state: INTERNAL_STATE::%s",
                state_name);

    // Actually set the state.
    this->internal_state = new_state;
}

void FCCBridgeNode::setup_ros() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up ROS");
    // Check if the node is in the correct state
    if (this->get_internal_state() != INTERNAL_STATE::STARTING_UP) {
        RCLCPP_ERROR(this->get_logger(),
                     "Attempted to setup ROS components more than once");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Setup publisher

    // Create GPSInfo & Position publisher
    this->gps_position_publisher =
        this->create_publisher<interfaces::msg::GPSPosition>(
            GPS_POSITION_TOPIC_NAME, 1);
    // Create FlightState publisher
    this->flight_state_publisher =
        this->create_publisher<interfaces::msg::FlightState>(
            FLIGHT_STATE_TOPIC_NAME, 1);

    // Create BatteryState publisher
    this->battery_state_publisher =
        this->create_publisher<interfaces::msg::BatteryState>(
            BATTERY_STATE_TOPIC_NAME, 1);

    // Create RCState publisher
    this->rc_state_publisher = this->create_publisher<interfaces::msg::RCState>(
        RC_STATE_TOPIC_NAME, 1);

    // Create euler angle publisher
    this->euler_angle_publisher = this->create_publisher<interfaces::msg::Pose>(
        EULER_ANGLE_TOPIC_NAME, 1);

    // Create mission progress publisher
    this->mission_progress_publisher =
        this->create_publisher<interfaces::msg::MissionProgress>(
            MISSION_PROGRESS_TOPIC_NAME, 1);

    // Create UAV health publisher
    this->uav_health_publisher =
        this->create_publisher<interfaces::msg::UAVHealth>(
            UAV_HEALTH_TOPIC_NAME, 1);

    // Create mission start publisher
    this->mission_start_publisher =
        this->create_publisher<interfaces::msg::MissionStart>(
            MISSION_START_TOPIC_NAME, 1);

    // Setup subscriber

    // Create subscription options for heartbeat to only receive mission control
    // heartbeats
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.content_filter_options.filter_expression =
        "sender_id = 'mission_control'";

    // Create Heartbeat subscription
    this->mission_control_heartbeat_subscriber =
        this->create_subscription<interfaces::msg::Heartbeat>(
            HEARTBEAT_TOPIC_NAME, 1,
            std::bind(&FCCBridgeNode::mission_control_heartbeat_subscriber_cb,
                      this, std::placeholders::_1),
            subscription_options);

    // Ensure that the filter is enabled. (Some DDS versions do not support it)
    if (!this->mission_control_heartbeat_subscriber->is_cft_enabled()) {
        RCLCPP_FATAL(this->get_logger(), "Content filtering is not enabled!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Create UAV command subscriber
    this->uav_command_subscriber =
        this->create_subscription<interfaces::msg::UAVCommand>(
            UAV_COMMAND_TOPIC_NAME, 10,
            std::bind(&FCCBridgeNode::uav_command_subscriber_cb, this,
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
    RCLCPP_DEBUG(this->get_logger(), "Received heartbeat from mission control");

    // Check if the heartbeat tick has increased since last time or in the case
    // of an overflow that is equal to 0
    if (msg.tick <= this->last_mission_control_heartbeat.tick &&
        msg.tick != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "The received heartbeat is not newer than the last one! "
                     "Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Set the cached heartbeat message to the current one
    this->last_mission_control_heartbeat = msg;

    // Verify that the received heartbeat is not too old
    this->check_last_mission_control_heartbeat();

    RCLCPP_INFO(this->get_logger(), "Mission control is alive and ok");
}

void FCCBridgeNode::uav_command_subscriber_cb(
    const interfaces::msg::UAVCommand &msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received a new UAVCommand message");

    // Ensure that the message is not too old
    if (this->now() - msg.time_stamp > rclcpp::Duration(MAX_UAV_COMMAND_AGE)) {
        RCLCPP_ERROR(
            this->get_logger(),
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
            break;
        case interfaces::msg::UAVCommand::RTH:
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Got unknown UAVCommand type %d",
                         msg.type);
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
                    RCLCPP_FATAL(this->get_logger(),
                                 "Drone is still on ground. Exiting...");
                    this->set_internal_state(INTERNAL_STATE::ERROR);
                    this->exit_process_on_error();
                case INTERNAL_STATE::WAITING_FOR_COMMAND:
                case INTERNAL_STATE::FLYING_ACTION:
                case INTERNAL_STATE::FLYING_MISSION:
                case INTERNAL_STATE::RETURN_TO_HOME:
                    RCLCPP_ERROR(this->get_logger(), "Triggering RTH...");
                    this->trigger_rth();
                    return;
                default:
                    throw std::runtime_error(
                        std::string("Got invalid value for internal_state: ") +
                        std::to_string(
                            static_cast<int>(this->get_internal_state())));
            }
    }

    RCLCPP_DEBUG(this->get_logger(), "Finished handling command");
}

void FCCBridgeNode::fcc_telemetry_timer_5hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "5Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heartbeat();
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            RCLCPP_WARN(this->get_logger(),
                        "5Hz Telemetry callback function was called in an "
                        "invalid state");
            return;

        // Remaining cases that are allowed:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
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
    if (this->get_internal_state() == INTERNAL_STATE::FLYING_MISSION) {
        // Send out mission progress
        this->send_mission_progress();
    }

    // Send out UAV health
    this->send_uav_health();
}

void FCCBridgeNode::fcc_telemetry_timer_10hz_cb() {
    RCLCPP_DEBUG(this->get_logger(),
                 "10Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heartbeat();
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::ERROR:
            RCLCPP_WARN(this->get_logger(),
                        "10Hz Telemetry callback function was called in an "
                        "invalid state");
            return;

        // Remaining cases that are allowed
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
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

    RCLCPP_DEBUG(this->get_logger(),
                 "10Hz telemetry timer callback successful");
}

FCCBridgeNode::FCCBridgeNode(const std::string &name/*,
                               const rclcpp::NodeOptions &node_options*/)
    : CommonNode(name), internal_state(INTERNAL_STATE::STARTING_UP) {
    // Pre-populate the last mission control heartbeat
    this->last_mission_control_heartbeat.time_stamp = this->now();
    this->last_mission_control_heartbeat.tick = 0;

    // Setup ROS objects such as timer, publishers etc.
    this->setup_ros();
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup ROS! Exiting...");
        this->exit_process_on_error();
    }
    this->set_internal_state(INTERNAL_STATE::ROS_SET_UP);
    RCLCPP_INFO(this->get_logger(), "Transitioning into ROS_SET_UP state");

    // Setup MAVSDK objects such as system, telemetry etc.
    this->setup_mavsdk();
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup MAVSDK! Exiting...");
        this->exit_process_on_error();
    }
    RCLCPP_INFO(this->get_logger(), "Transitioning into MAVSDK_SET_UP state");
    this->set_internal_state(INTERNAL_STATE::MAVSDK_SET_UP);

    // Activating node to signal that it is ready for the safety limits
    this->activate();

    RCLCPP_INFO(this->get_logger(), "Successfully created %s instance",
                this->get_name());
}

}  // namespace fcc_bridge
