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

constexpr float WAYPOINT_ACCEPTANCE_RADIUS_M =
    .5; /**< Acceptance radius for waypoints */

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

void FCCBridgeNode::initiate_takeoff(const interfaces::msg::Waypoint &waypoint,
                                     const float speed_mps) {
    RCLCPP_INFO(this->get_logger(),
                "Received a command to takeoff to lat: %f°\tlon: %f°\trel alt: "
                "%fm with speed: %fm/s",
                waypoint.latitude_deg, waypoint.longitude_deg,
                waypoint.relative_altitude_m, static_cast<double>(speed_mps));

    // Verify that the uav is in the right state
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen as the process should have exited before
            throw std::runtime_error(
                "Received command to take of while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::LANDED:
            // The UAV is still or again on the ground so the only thing left is
            // to kill the process
            RCLCPP_FATAL(
                this->get_logger(),
                "Received a takeoff command in an invalid state! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // If the UAV is airborne and another takeoff command is received
            // something must have gone wrong so an RTH is triggered
            RCLCPP_ERROR(this->get_logger(),
                         "Received a takeoff command while airborne! "
                         "Triggering RTH...");
            this->trigger_rth();
            return;
        case INTERNAL_STATE::ARMED:
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Check that the speed is valid
    if (!this->check_speed(speed_mps)) {
        if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
            // This means an unrecoverable error has occurred such as missing
            // safety limits
            RCLCPP_FATAL(
                this->get_logger(),
                "There has been an unrecoverable error checking if the target "
                "speed for takeoff was inside the safety limits! Exiting...");
        } else {
            // In this case the target takeoff speed is outside the limits
            // deemed safe. Because the UAV is on the ground this results in a
            // process exit.
            RCLCPP_FATAL(
                this->get_logger(),
                "The target takeoff speed is outside the speed "
                "safety limits (%f is not in (%f;%f]). The UAV should "
                "not be airborne! Exiting...",
                static_cast<double>(speed_mps),
                static_cast<double>(safety_limits::MIN_SPEED_LIMIT_MPS),
                static_cast<double>(this->safety_limits->max_speed_mps));
            this->set_internal_state(INTERNAL_STATE::ERROR);
        }
        // Exit the process
        this->exit_process_on_error();
    }

    // Check that the waypoint is valid
    if (waypoint.relative_altitude_m ==
        interfaces::msg::Waypoint::INVALID_ALTITUDE) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Got an invalid waypoint for a takeoff command! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    // Verify that the waypoint is inside the geofence
    if (!this->check_point_in_geofence(waypoint)) {
        // Something went wrong with the check
        if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
            // This means an unrecoverable error has occurred such as missing
            // safety limits
            RCLCPP_FATAL(
                this->get_logger(),
                "There has been an unrecoverable error checking if the target "
                "waypoint for takeoff was inside the geofence! Exiting...");
        } else {
            // The waypoint is outside the geofence. The UAV is still on the
            // ground so the process will exit.
            RCLCPP_FATAL(
                this->get_logger(),
                "Got an waypoint with lat: %f°\tlon: %f°\trel. alt. %fm "
                "for a takeoff that is outside the geofence! Exiting...",
                waypoint.latitude_deg, waypoint.longitude_deg,
                waypoint.relative_altitude_m);
            this->set_internal_state(INTERNAL_STATE::ERROR);
        }
        // Exit the process
        this->exit_process_on_error();
    }

    RCLCPP_INFO(this->get_logger(), "Clear for takeoff");

    // Mission item that describes the takeoff action
    mavsdk::Mission::MissionItem takeoff_mission_item;

    // Set all the relevant values
    takeoff_mission_item.latitude_deg = waypoint.latitude_deg;
    takeoff_mission_item.longitude_deg = waypoint.longitude_deg;
    takeoff_mission_item.relative_altitude_m = waypoint.relative_altitude_m;
    takeoff_mission_item.speed_m_s = speed_mps;
    takeoff_mission_item.is_fly_through = false;
    takeoff_mission_item.acceptance_radius_m = WAYPOINT_ACCEPTANCE_RADIUS_M;
    takeoff_mission_item.vehicle_action =
        mavsdk::Mission::MissionItem::VehicleAction::Takeoff;

    // Mission plan that stores the mission items. In this case only one
    mavsdk::Mission::MissionPlan takeoff_mission_plan;

    // Insert takeoff action into mission plan
    takeoff_mission_plan.mission_items.push_back(takeoff_mission_item);

    // Execute mission
    if (!this->execute_mission_plan(takeoff_mission_plan)) {
        RCLCPP_FATAL(
            this->get_logger(),
                     "There was an error triggering the waypoint mission! "
                     "Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Set state to waiting for mission
    this->set_internal_state(INTERNAL_STATE::FLYING_MISSION);

    RCLCPP_INFO(
        this->get_logger(),
        "Successfully triggered takeoff. Waiting for mission to complete.");
}

void FCCBridgeNode::start_flying_to_waypoint(
    const interfaces::msg::Waypoint &waypoint, const float speed_mps) {
    RCLCPP_INFO(this->get_logger(),
                "Received a command to fly to a waypoint at lat: %f°\tlon: "
                "%f°\trel alt: %fm with speed: %fm/s",
                waypoint.latitude_deg, waypoint.longitude_deg,
                waypoint.relative_altitude_m, static_cast<double>(speed_mps));

    // Verify that the uav is in the right state
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen as the process should have exited before
            throw std::runtime_error(
                "Received command to take of while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::LANDED:
        case INTERNAL_STATE::ARMED:
            // The UAV is still or again on the ground so the only thing left is
            // to kill the process
            RCLCPP_FATAL(
                this->get_logger(),
                "Received a takeoff command in an invalid state! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // If the UAV is already waiting on a UAVCommand and another
            // waypoint command is received something must have gone wrong so an
            // RTH is triggered
            RCLCPP_ERROR(this->get_logger(),
                         "Received a waypoint command while there is already a "
                         "command in progress! Triggering RTH...");
            this->trigger_rth();
            return;
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
            break;
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Check that the speed is valid
    if (!this->check_speed(speed_mps)) {
        if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
            // This means an unrecoverable error has occurred such as missing
            // safety limits
            RCLCPP_FATAL(
                this->get_logger(),
                "There has been an unrecoverable error checking if the target "
                "speed for waypoint was inside the safety limits! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // In this case the target waypoint speed is outside the limits
            // deemed safe. This will result in an RTH
            RCLCPP_ERROR(
                this->get_logger(),
                "The target waypoint speed is outside the speed safety limits "
                "(%f is not in (%f;%f]). Triggering RTH...",
                static_cast<double>(speed_mps),
                static_cast<double>(safety_limits::MIN_SPEED_LIMIT_MPS),
                static_cast<double>(this->safety_limits->max_speed_mps));
            this->trigger_rth();
            return;
        }
    }

    // Check that the waypoint is valid
    if (waypoint.relative_altitude_m ==
        interfaces::msg::Waypoint::INVALID_ALTITUDE) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Got an invalid waypoint for a takeoff command! Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Verify that the waypoint is inside the geofence
    if (!this->check_point_in_geofence(waypoint)) {
        // Something went wrong with the check
        if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
            // This means an unrecoverable error has occurred such as missing
            // safety limits
            RCLCPP_FATAL(
                this->get_logger(),
                "There has been an unrecoverable error checking if the target "
                "waypoint was inside the geofence! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // The waypoint is outside the geofence. This will trigger an RTH.
            RCLCPP_FATAL(
                this->get_logger(),
                "Got an waypoint with lat: %f°\tlon: %f°\trel. alt. %fm to fly "
                "to that is outside the geofence! Triggering RTH...",
                waypoint.latitude_deg, waypoint.longitude_deg,
                waypoint.relative_altitude_m);
            this->trigger_rth();
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Clear for fly to waypoint");

    // Mission item that describes the fly to waypoint action
    mavsdk::Mission::MissionItem fly_to_waypoint_mission_item;

    // Set all the relevant values
    fly_to_waypoint_mission_item.latitude_deg = waypoint.latitude_deg;
    fly_to_waypoint_mission_item.longitude_deg = waypoint.longitude_deg;
    fly_to_waypoint_mission_item.relative_altitude_m =
        fly_to_waypoint_mission_item.relative_altitude_m;
    fly_to_waypoint_mission_item.speed_m_s = speed_mps;
    fly_to_waypoint_mission_item.is_fly_through = false;
    fly_to_waypoint_mission_item.acceptance_radius_m =
        WAYPOINT_ACCEPTANCE_RADIUS_M;
    fly_to_waypoint_mission_item.vehicle_action =
        mavsdk::Mission::MissionItem::VehicleAction::None;

    // Mission plan that stores the mission items. In this case only one
    mavsdk::Mission::MissionPlan fly_to_waypoint_mission_plan;

    // Insert fly to waypoint action into mission plan
    fly_to_waypoint_mission_plan.mission_items.push_back(
        fly_to_waypoint_mission_item);

    // Execute mission
    if (!this->execute_mission_plan(fly_to_waypoint_mission_plan)) {
        RCLCPP_FATAL(
            this->get_logger(),
            "There was an error triggering the takeoff mission! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    // Set state to waiting for mission
    this->set_internal_state(INTERNAL_STATE::FLYING_MISSION);

    RCLCPP_INFO(this->get_logger(),
                "Successfully triggered fly to waypoint. Waiting for mission "
                "to complete.");
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

void FCCBridgeNode::check_last_mission_control_heartbeat() {
    // TODO: Check if last heartbeat is not too old.
}

void FCCBridgeNode::send_gps_telemetry() {
    RCLCPP_DEBUG(this->get_logger(),
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

    RCLCPP_DEBUG(this->get_logger(), "Published current GPS position");
}

void FCCBridgeNode::send_flight_state() {
    RCLCPP_DEBUG(this->get_logger(),
                 "Getting updated flight state and publishing the update");

    // Update flight state
    this->get_flight_state();

    // Verify the Flight State depending on the internal state
    this->check_flight_state();

    // If we are in the WAITING_FOR_ARM state and the drone is READY send
    // MissionStart and wait for takeoff
    if (this->get_internal_state() == INTERNAL_STATE::WAITING_FOR_ARM &&
        this->last_fcc_flight_mode.value() ==
            mavsdk::Telemetry::FlightMode::Ready) {
        RCLCPP_INFO(this->get_logger(),
                    "Drone is armed and ready for takeoff. Switching to ARMED "
                    "state and sending MissionStart message");
        this->set_internal_state(INTERNAL_STATE::ARMED);
        interfaces::msg::MissionStart mission_start_msg;
        mission_start_msg.sender_id = this->get_name();
        this->mission_start_publisher->publish(mission_start_msg);
        RCLCPP_DEBUG(this->get_logger(), "MissionStart message send");
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

    // Publish the message
    this->flight_state_publisher->publish(flight_state_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published current flight state");
}

void FCCBridgeNode::send_battery_state() {
    RCLCPP_DEBUG(this->get_logger(),
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

    RCLCPP_DEBUG(this->get_logger(), "Published current battery state");
}

void FCCBridgeNode::send_rc_state() {
    RCLCPP_DEBUG(this->get_logger(),
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

    RCLCPP_DEBUG(this->get_logger(), "Published current RC state");
}

void FCCBridgeNode::send_euler_angle() {
    RCLCPP_DEBUG(this->get_logger(),
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

    RCLCPP_DEBUG(this->get_logger(), "Published current euler angle");
}

void FCCBridgeNode::send_mission_progress() {
    RCLCPP_DEBUG(this->get_logger(),
                 "Getting updated mission progress and publishing the update");

    // Update mission progress
    this->get_mission_progress();

    // Verify of getting the mission progress was successful
    if (this->last_mission_progress->first !=
        mavsdk::Mission::Result::Success) {
        // Trigger an RTH on any error
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to get mission progress! Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // In this case retrieving the mission progress was successful meaning that
    // it can be safely accessed.
    interfaces::msg::MissionProgress mission_progress_msg;
    mission_progress_msg.time_stamp = this->now();
    mission_progress_msg.sender_id = this->get_name();
    // Check if the mission is finished
    if (this->last_mission_progress->second) {
        // Switch to the waiting for command state
        this->set_internal_state(INTERNAL_STATE::WAITING_FOR_COMMAND);
        mission_progress_msg.progress = 1.0;
    } else {
        mission_progress_msg.progress = 0.0;
    }

    // Publish the message
    this->mission_progress_publisher->publish(mission_progress_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published current mission progress");
}

void FCCBridgeNode::send_uav_health() {
    RCLCPP_DEBUG(this->get_logger(),
                 "Getting updated uav health and publishing the update");

    // Update UAV health
    this->get_uav_health();

    // Verify the health depending on the internal state
    this->check_uav_health();

    RCLCPP_INFO(this->get_logger(), "UAV config is ok in the current state %s",
                this->internal_state_to_str());

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

    RCLCPP_DEBUG(this->get_logger(), "Published current UAV health");
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
