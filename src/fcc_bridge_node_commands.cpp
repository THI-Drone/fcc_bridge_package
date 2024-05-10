//
// Created by Johan <job8197@thi.de> on 05.05.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {

namespace {

constexpr float WAYPOINT_ACCEPTANCE_RADIUS_M =
    .5; /**< Acceptance radius for waypoints */

}

void FCCBridgeNode::initiate_takeoff(const interfaces::msg::Waypoint &waypoint,
                                     const float speed_mps) {
    RCLCPP_INFO(this->get_command_handler_logger(),
                "Received a command to takeoff to lat: %f°\tlon: %f°\trel alt: "
                "%fm with speed: %fm/s",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m),
                static_cast<double>(speed_mps));

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
                this->get_internal_state_logger(),
                "Received a takeoff command in an invalid state! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // If the UAV is airborne and another takeoff command is received
            // something must have gone wrong so an RTH is triggered
            RCLCPP_ERROR(this->get_internal_state_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "speed for takeoff was inside the safety limits! Exiting...");
        } else {
            // In this case the target takeoff speed is outside the limits
            // deemed safe. Because the UAV is on the ground this results in a
            // process exit.
            RCLCPP_FATAL(
                this->get_command_handler_logger(),
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
    if (0.f <= std::abs(waypoint.relative_altitude_m - interfaces::msg::Waypoint::INVALID_ALTITUDE)) {
        RCLCPP_FATAL(
            this->get_command_handler_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "waypoint for takeoff was inside the geofence! Exiting...");
        } else {
            // The waypoint is outside the geofence. The UAV is still on the
            // ground so the process will exit.
            RCLCPP_FATAL(
                this->get_command_handler_logger(),
                "Got an waypoint with lat: %f°\tlon: %f°\trel. alt. %fm "
                "for a takeoff that is outside the geofence! Exiting...",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m));
            this->set_internal_state(INTERNAL_STATE::ERROR);
        }
        // Exit the process
        this->exit_process_on_error();
    }

    RCLCPP_INFO(this->get_command_handler_logger(), "Clear for takeoff");

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
        RCLCPP_FATAL(this->get_command_handler_logger(),
                     "There was an error triggering the waypoint mission! "
                     "Triggering RTH...");
        this->trigger_rth();
        return;
    }

    // Set state to waiting for mission
    this->set_internal_state(INTERNAL_STATE::FLYING_MISSION);

    RCLCPP_INFO(
        this->get_command_handler_logger(),
        "Successfully triggered takeoff. Waiting for mission to complete.");
}

void FCCBridgeNode::start_flying_to_waypoint(
    const interfaces::msg::Waypoint &waypoint, const float speed_mps) {
    RCLCPP_INFO(this->get_command_handler_logger(),
                "Received a command to fly to a waypoint at lat: %f°\tlon: "
                "%f°\trel alt: %fm with speed: %fm/s",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m),
                static_cast<double>(speed_mps));

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
                this->get_internal_state_logger(),
                "Received a takeoff command in an invalid state! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // If the UAV is already waiting on a UAVCommand and another
            // waypoint command is received something must have gone wrong so an
            // RTH is triggered
            RCLCPP_ERROR(this->get_internal_state_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "speed for waypoint was inside the safety limits! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // In this case the target waypoint speed is outside the limits
            // deemed safe. This will result in an RTH
            RCLCPP_ERROR(
                this->get_command_handler_logger(),
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
    if (0.f <= std::abs(waypoint.relative_altitude_m - interfaces::msg::Waypoint::INVALID_ALTITUDE)) {
        RCLCPP_ERROR(
            this->get_command_handler_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "waypoint was inside the geofence! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // The waypoint is outside the geofence. This will trigger an RTH.
            RCLCPP_FATAL(
                this->get_command_handler_logger(),
                "Got an waypoint with lat: %f°\tlon: %f°\trel. alt. %fm to fly "
                "to that is outside the geofence! Triggering RTH...",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m));
            this->trigger_rth();
            return;
        }
    }

    RCLCPP_INFO(this->get_command_handler_logger(),
                "Clear for fly to waypoint");

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
        RCLCPP_FATAL(this->get_command_handler_logger(),
                     "There was an error triggering the fly to waypoint "
                     "mission! Exiting...");
        // TODO: Change to RTH
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    // Set state to waiting for mission
    this->set_internal_state(INTERNAL_STATE::FLYING_MISSION);

    RCLCPP_INFO(this->get_command_handler_logger(),
                "Successfully triggered fly to waypoint. Waiting for mission "
                "to complete.");
}

void FCCBridgeNode::initiate_land(const interfaces::msg::Waypoint &waypoint,
                                  const float speed_mps) {
    RCLCPP_INFO(this->get_command_handler_logger(),
                "Received a command to land at waypoint at lat: %f°\tlon: "
                "%f°\trel alt: %fm with speed: %fm/s",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m),
                static_cast<double>(speed_mps));

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
                this->get_internal_state_logger(),
                "Received a takeoff command in an invalid state! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // If the UAV is already waiting on a UAVCommand and another
            // command is received something must have gone wrong so an RTH is
            // triggered
            RCLCPP_ERROR(this->get_internal_state_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "speed for waypoint was inside the safety limits! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // In this case the target waypoint speed is outside the limits
            // deemed safe. This will result in an RTH
            RCLCPP_ERROR(
                this->get_command_handler_logger(),
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
    if (0.f >= std::abs(waypoint.relative_altitude_m -
        interfaces::msg::Waypoint::INVALID_ALTITUDE)) {
        RCLCPP_ERROR(
            this->get_command_handler_logger(),
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
                this->get_command_handler_logger(),
                "There has been an unrecoverable error checking if the target "
                "waypoint was inside the geofence! Exiting...");
            // Exit the process
            this->exit_process_on_error();
        } else {
            // The waypoint is outside the geofence. This will trigger an RTH.
            RCLCPP_FATAL(
                this->get_command_handler_logger(),
                "Got an waypoint with lat: %f°\tlon: %f°\trel. alt. %fm to fly "
                "to that is outside the geofence! Triggering RTH...",
                waypoint.latitude_deg, waypoint.longitude_deg,
                static_cast<double>(waypoint.relative_altitude_m));
            this->trigger_rth();
            return;
        }
    }

    RCLCPP_INFO(this->get_command_handler_logger(),
                "Clear for land at waypoint");

    // Mission item that describes the fly to waypoint action
    mavsdk::Mission::MissionItem land_at_waypoint_mission_item;

    // Set all the relevant values
    land_at_waypoint_mission_item.latitude_deg = waypoint.latitude_deg;
    land_at_waypoint_mission_item.longitude_deg = waypoint.longitude_deg;
    land_at_waypoint_mission_item.relative_altitude_m =
        land_at_waypoint_mission_item.relative_altitude_m;
    land_at_waypoint_mission_item.speed_m_s = speed_mps;
    land_at_waypoint_mission_item.is_fly_through = false;
    land_at_waypoint_mission_item.acceptance_radius_m =
        WAYPOINT_ACCEPTANCE_RADIUS_M;
    land_at_waypoint_mission_item.vehicle_action =
        mavsdk::Mission::MissionItem::VehicleAction::Land;

    // Mission plan that stores the mission items. In this case only one
    mavsdk::Mission::MissionPlan land_at_waypoint_mission_plan;

    // Insert fly to waypoint action into mission plan
    land_at_waypoint_mission_plan.mission_items.push_back(
        land_at_waypoint_mission_item);

    // Execute mission
    if (!this->execute_mission_plan(land_at_waypoint_mission_plan)) {
        RCLCPP_FATAL(
            this->get_command_handler_logger(),
            "There was an error triggering the land mission! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
        // TODO: Change to RTH
    }

    // Set state to waiting for mission
    this->set_internal_state(INTERNAL_STATE::LANDING);

    RCLCPP_INFO(this->get_command_handler_logger(),
                "Successfully triggered land at waypoint. Waiting for mission "
                "to complete.");
}

void FCCBridgeNode::initiate_rth() {
    RCLCPP_DEBUG(this->get_command_handler_logger(), "Initiating RTH");

    this->trigger_rth();

    RCLCPP_INFO(this->get_command_handler_logger(),
                "Successfully triggered an RTH");
}

}  // namespace fcc_bridge