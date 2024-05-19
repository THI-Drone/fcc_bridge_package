//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

// Libc header
#include <chrono>
#include <cinttypes>

// CommonLib header
#include <common_package/node_names.hpp>
#include <common_package/topic_names.hpp>

// FCC Bridge header
#include "fcc_bridge_node.hpp"

namespace fcc_bridge {

namespace {

// Timer periods
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_5HZ{
    200}; /**< The period of the 5Hz telemetry timer */
constexpr std::chrono::milliseconds FCC_TELEMETRY_PERIOD_10HZ{
    100}; /**< The period of the 10Hz telemetry timer */

// Limits
constexpr std::chrono::seconds MAX_UAV_COMMAND_AGE{
    1}; /**< Maximum age of a received UAV command */

}  // namespace

void FCCBridgeNode::set_internal_state(
    const fcc_bridge::FCCBridgeNode::INTERNAL_STATE new_state) {
    char const *const old_state = this->internal_state_to_str();
    // Actually set the state.
    this->internal_state = new_state;

    RCLCPP_INFO(this->get_internal_state_logger(),
                "Switching old internal state: %s to new internal_state: %s",
                old_state, this->internal_state_to_str());
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

    // Create Heartbeat subscription
    this->mission_control_heartbeat_subscriber =
        this->create_subscription<interfaces::msg::Heartbeat>(
            common_lib::topic_names::Heartbeat, 1,
            std::bind(&FCCBridgeNode::mission_control_heartbeat_subscriber_cb,
                      this, std::placeholders::_1));

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

    // Create Control subscriber
    this->control_subscriber =
        this->create_subscription<interfaces::msg::Control>(
            common_lib::topic_names::Control, 10,
            std::bind(&FCCBridgeNode::control_cb, this, std::placeholders::_1));

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

    if (msg.sender_id != common_lib::node_names::MISSION_CONTROL) {
        return;
    }

    if (this->last_mission_control_heartbeat.has_value()) {
        // Check if the heartbeat tick has increased since last time or in the
        // case of an overflow that is equal to 0
        if (msg.tick <= this->last_mission_control_heartbeat->tick &&
            msg.tick == 0 && this->last_mission_control_heartbeat->tick == 0) {
            RCLCPP_ERROR(
                this->get_safety_logger(),
                "The received heartbeat is not newer than the last one! "
                "Triggering RTH...");
            this->trigger_rth();
            return;
        }
    } else {
        RCLCPP_INFO(this->get_ros_interface_logger(),
                    "Got the first heartbeat from mission control. Activating "
                    "node to signal to mission control that the node is ready "
                    "for the safety limits");
        // Activating node to signal that it is ready for the safety limits
        this->activate();
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

    switch (msg.type) {
        case interfaces::msg::UAVCommand::TAKE_OFF:
            if (!this->check_sender(msg.sender_id,
                                    common_lib::node_names::MISSION_CONTROL)) {
                // RTH should have been triggered
                RCLCPP_ERROR(this->get_ros_interface_logger(),
                             "Take off command sender was invalid!");
                return;
            }
            this->initiate_takeoff(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::FLY_TO_WAYPOINT:
            if (!this->check_sender(msg.sender_id,
                                    common_lib::node_names::WAYPOINT)) {
                // RTH should have been triggered
                RCLCPP_ERROR(this->get_ros_interface_logger(),
                             "Fly to waypoint command sender was invalid!");
                return;
            }
            this->start_flying_to_waypoint(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::LAND:
            if (!this->check_sender(msg.sender_id,
                                    common_lib::node_names::MISSION_CONTROL)) {
                // RTH should have been triggered
                RCLCPP_ERROR(this->get_ros_interface_logger(),
                             "LAND command sender was invalid!");
                return;
            }
            this->initiate_land(msg.waypoint, msg.speed_m_s);
            break;
        case interfaces::msg::UAVCommand::RTH:
            if (!this->check_sender(msg.sender_id,
                                    common_lib::node_names::MISSION_CONTROL)) {
                // RTH should have been triggered
                RCLCPP_ERROR(this->get_ros_interface_logger(),
                             "Return to home command sender was invalid!");
                return;
            }
            this->initiate_rth();
            break;
        default:
            if (this->is_airborne()) {
                RCLCPP_ERROR(this->get_safety_logger(),
                             "Got unknown UAVCommand type %" PRIu8
                             "! Triggering RTH...",
                             msg.type);
                this->trigger_rth();
                return;
            } else {
                // In these cases the UAV is not airborne so the process
                // will exit
                RCLCPP_FATAL(this->get_safety_logger(),
                             "Got unknown UAVCommand type %" PRIu8
                             "!Drone is still on ground. Exiting...",
                             msg.type);
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
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

    if (msg.sender_id != common_lib::node_names::MISSION_CONTROL) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "Received a MissionFinished message from a non mission "
                     "control node! Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Ignoring whether mission control is active

    // Ignoring mission finished if we are in RTH and no error is indicated

    if (this->get_internal_state() != INTERNAL_STATE::RETURN_TO_HOME ||
        msg.error_code != EXIT_SUCCESS) {
        if (this->is_airborne()) {
            // The UAV is airborne. This will result in an RTH
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "Received a MissionFinished message while airborne. "
                        "Triggering an RTH...");
            this->trigger_rth();
        } else {
            // The UAV has not yet taken off
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "Received a MissionFinished while the UAV has not yet "
                         "taken off! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    RCLCPP_INFO(this->get_ros_interface_logger(),
                "Ignored Mission finished message with no indicated error "
                "while in RTH state");
}

void FCCBridgeNode::safety_limits_cb(const interfaces::msg::SafetyLimits &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Received a new SafetyLimits message");

    if (this->get_internal_state() != INTERNAL_STATE::MAVSDK_SET_UP) {
        if (this->is_airborne()) {
            // The UAV is airborne. This will result in an RTH
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "Received a SafetyLimits message while airborne. "
                        "Triggering an RTH...");
            this->trigger_rth();
            return;
        } else {
            // The UAV has not yet taken off
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "Received SafetyLimits in an invalid state while the "
                         "UAV has not yet taken off! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    // Ignoring whether mission control is active

    if (msg.sender_id != common_lib::node_names::MISSION_CONTROL) {
        RCLCPP_FATAL(this->get_ros_interface_logger(),
                     "Got safety limits from invalid sender: %s (Expected: "
                     "%s)! Exiting...",
                     msg.sender_id.c_str(),
                     common_lib::node_names::MISSION_CONTROL);
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    GeofenceInstace::PolygonType geofence_polygon;

    for (const interfaces::msg::Waypoint &point : msg.geofence_points) {
        RCLCPP_DEBUG(this->get_ros_interface_logger(),
                     "Got geofence point lat: %f°\tlon: %f°",
                     point.latitude_deg, point.longitude_deg);
        if (!std::isfinite(point.latitude_deg) ||
            !std::isfinite(point.longitude_deg)) {
            RCLCPP_FATAL(this->get_ros_interface_logger(),
                         "One of the geofence point contains non finite "
                         "coordinates! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
        geofence_polygon.push_back({point.latitude_deg, point.longitude_deg});
    }

    // Initialize safety limits
    this->safety_limits.emplace(msg.max_speed_m_s, msg.min_soc,
                                msg.max_height_m, geofence_polygon);

    this->validate_safety_limits();

    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        RCLCPP_FATAL(this->get_ros_interface_logger(),
                     "Failed to validate safety limits! Exiting...");
        this->exit_process_on_error();
    }

    RCLCPP_INFO(this->get_ros_interface_logger(), "Set safety limits");

    // Disable safety limits subscriber
    this->safety_limits_subscriber.reset();

    // Go into WAITING_FOR_ARM state
    this->set_internal_state(INTERNAL_STATE::WAITING_FOR_ARM);
}

void FCCBridgeNode::control_cb(const interfaces::msg::Control &msg) {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Got new control message with target: %s and active: %s",
                 msg.target_id.c_str(), msg.active ? "true" : "false");

    if (common_lib::node_names::VALID_CONTROL_NODE_NAMES.find(msg.target_id) ==
        common_lib::node_names::VALID_CONTROL_NODE_NAMES.end()) {
        if (this->is_airborne()) {
            // UAV is airborne. RTH is triggered
            RCLCPP_ERROR(this->get_ros_interface_logger(),
                         "Received a control message with an invalid "
                         "target id %s! UAV is airborne. Triggering RTH...",
                         msg.target_id.c_str());
            this->trigger_rth();
            return;
        } else {
            // UAV is on ground exiting
            RCLCPP_FATAL(this->get_ros_interface_logger(),
                         "Received a control message with an invalid "
                         "target id %s! UAV is on ground. Exiting...",
                         msg.target_id.c_str());
            this->exit_process_on_error();
        }
    }

    if (msg.active) {
        if (this->active_node.has_value()) {
            RCLCPP_WARN(
                this->get_ros_interface_logger(),
                "Already got an active node: %s. Switching anyway to: %s",
                this->active_node->c_str(), msg.target_id.c_str());
        }
        this->active_node = msg.target_id;
    } else {
        if (!this->active_node.has_value()) {
            RCLCPP_WARN(this->get_ros_interface_logger(),
                        "Got a control message to deactivate node %s while "
                        "there was no active node configured",
                        msg.target_id.c_str());
        } else if (this->active_node.value() != msg.target_id) {
            RCLCPP_WARN(this->get_ros_interface_logger(),
                        "Got a control message to deactivate node %s while "
                        "node %s is active. Ignoring...",
                        msg.target_id.c_str(), this->active_node->c_str());
        } else {
            this->active_node.reset();
        }
    }
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "Handling of control message complete");
}

void FCCBridgeNode::fcc_telemetry_timer_5hz_cb() {
    RCLCPP_DEBUG(this->get_ros_interface_logger(),
                 "5Hz telemetry timer callback was triggered");
    this->check_last_mission_control_heartbeat();
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw invalid_state_error(std::string(__func__) +
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
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            break;
        default:
            throw unknown_enum_value_error(
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
            RCLCPP_INFO(this->get_internal_state_logger(),
                        "Not in a state to publish mission progress");
            break;
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
            // Send out mission progress
            this->send_mission_progress();
            break;
        default:
            throw unknown_enum_value_error(
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
            throw invalid_state_error(std::string(__func__) +
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
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            break;
        default:
            throw unknown_enum_value_error(
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

    RCLCPP_INFO(this->get_logger(), "Successfully created %s instance",
                this->get_name());
}

}  // namespace fcc_bridge
