//
// Created by Johan <job8197@thi.de> on 04.05.2024.
//

#include <map>
#include <set>

// FCC Bridge header
#include "fcc_bridge/fcc_bridge_node.hpp"

namespace fcc_bridge {

namespace {

using INTERNAL_STATE = FCCBridgeNode::INTERNAL_STATE;

using LandedState = mavsdk::Telemetry::LandedState;

using FlightMode = mavsdk::Telemetry::FlightMode;

template <typename T>
struct state_action {
    const std::set<T> ERROR_STATES;
    const std::set<T> RTH_STATES;
    const std::set<T> VALID_STATES;

    state_action(const std::initializer_list<T> &error_states,
                 const std::initializer_list<T> &rth_states,
                 const std::initializer_list<T> &valid_states)
        : ERROR_STATES(error_states),
          RTH_STATES(rth_states),
          VALID_STATES(valid_states) {}

    state_action(const std::set<T> &error_states,
                 const std::initializer_list<T> &rth_states,
                 const std::initializer_list<T> &valid_states)
        : ERROR_STATES(error_states),
          RTH_STATES(rth_states),
          VALID_STATES(valid_states) {}
};

const state_action<LandedState> uk_og_valid{
    {},
    {LandedState::TakingOff, LandedState::InAir, LandedState::Landing},
    {LandedState::Unknown, LandedState::OnGround}};

const state_action<LandedState> ia_valid{
    {LandedState::OnGround},
    {LandedState::Unknown, LandedState::TakingOff, LandedState::Landing},
    {LandedState::InAir}};

const state_action<LandedState> og_ia_la_valid{
    {},
    {LandedState::Unknown, LandedState::TakingOff},
    {LandedState::OnGround, LandedState::InAir, LandedState::Landing}};

// INTERNAL_STATE::ERROR -> Kill / INTERNAL_STATE::STARTING_UP &
// INTERNAL_STATE::ROS_SET_UP -> exit expected to be handled explicitly
const std::map<const INTERNAL_STATE, const state_action<LandedState>>
    landed_state_actions{
        {INTERNAL_STATE::MAVSDK_SET_UP, uk_og_valid},
        {INTERNAL_STATE::WAITING_FOR_ARM, uk_og_valid},
        {INTERNAL_STATE::ARMED,
         {{},
          {LandedState::Unknown, LandedState::TakingOff, LandedState::InAir,
           LandedState::Landing},
          {LandedState::OnGround}}},
        {INTERNAL_STATE::TAKING_OFF,
         {{},
          {LandedState::Unknown, LandedState::Landing},
          {LandedState::OnGround, LandedState::TakingOff, LandedState::InAir}}},
        {INTERNAL_STATE::WAITING_FOR_COMMAND, ia_valid},
        {INTERNAL_STATE::FLYING_MISSION, ia_valid},
        {INTERNAL_STATE::LANDING, og_ia_la_valid},
        {INTERNAL_STATE::RETURN_TO_HOME, og_ia_la_valid},
        {INTERNAL_STATE::LANDED, uk_og_valid}};

const std::set<FlightMode> exit_flight_modes{
    FlightMode::Offboard,   FlightMode::FollowMe, FlightMode::Manual,
    FlightMode::Altctl,     FlightMode::Posctl,   FlightMode::Acro,
    FlightMode::Stabilized, FlightMode::Rattitude};

// INTERNAL_STATE::ERROR -> Kill
// INTERNAL_STATE::STARTING_UP & INTERNAL_STATE::ROS_SET_UP -> exit
// INTERNAL_STATE::MAVSDK_SET_UP & INTERNAL_STATE::WAITING_FOR_ARM -> valid
// expected to be handled explicitly
const std::map<const INTERNAL_STATE, const state_action<FlightMode>>
    flight_mode_actions{
        {INTERNAL_STATE::ARMED,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::Mission, FlightMode::ReturnToLaunch, FlightMode::Land},
          {FlightMode::Hold}}},
        {INTERNAL_STATE::TAKING_OFF,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::ReturnToLaunch, FlightMode::Land},
          {FlightMode::Hold, FlightMode::Mission}}},
        {INTERNAL_STATE::WAITING_FOR_COMMAND,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::ReturnToLaunch, FlightMode::Land},
          {FlightMode::Hold, FlightMode::Mission}}},
        {INTERNAL_STATE::FLYING_MISSION,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::ReturnToLaunch, FlightMode::Land},
          {FlightMode::Hold, FlightMode::Mission}}},
        {INTERNAL_STATE::LANDING,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::ReturnToLaunch},
          {FlightMode::Hold, FlightMode::Mission, FlightMode::Land}}},
        {INTERNAL_STATE::RETURN_TO_HOME,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Ready, FlightMode::Takeoff,
           FlightMode::Land},
          {FlightMode::Hold, FlightMode::Mission, FlightMode::ReturnToLaunch}}},
        {INTERNAL_STATE::LANDED,
         {exit_flight_modes,
          {FlightMode::Unknown, FlightMode::Takeoff, FlightMode::Mission,
           FlightMode::ReturnToLaunch, FlightMode::Land},
          {FlightMode::Ready, FlightMode::Hold}}}};

constexpr std::chrono::milliseconds MAX_MISSION_CONTROL_HEARTBEAT_AGE{1000};

}  // namespace

bool FCCBridgeNode::check_sender(const std::string &actual_sender,
                                 const char *const expected_sender) {
    if (!this->active_node.has_value()) {
        RCLCPP_WARN(this->get_safety_logger(), "No active node is configured");
    } else if (this->active_node.value() != expected_sender) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "The currently active node %s is not the expected node %s",
                    this->active_node->c_str(), expected_sender);
    } else if (actual_sender != expected_sender) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "The actual sender %s does not match the expected %s",
                    actual_sender.c_str(), expected_sender);
    } else {
        RCLCPP_DEBUG(this->get_safety_logger(),
                     "Checking the sender %s matched the expected and "
                     "currently active node",
                     expected_sender);
        return true;
    }
    if (this->is_airborne()) {
        // The UAV is airborne. This will result in an RTH
        RCLCPP_WARN(this->get_internal_state_logger(),
                    "The UAV is airborne. Triggering an RTH...");
        this->trigger_rth();
        return false;
    } else {
        // The UAV has not yet taken off
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "The UAV has not yet taken off! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }
}

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
    if (!std::isfinite(this->safety_limits->max_speed_mps) ||
        this->safety_limits->max_speed_mps <= 0 ||
        SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS <
            this->safety_limits->max_speed_mps) {
        RCLCPP_WARN(
            this->get_safety_logger(),
            "Got invalid speed: %fm/s outside of range (0;%f]. Using Internal "
            "limit: %f",
            static_cast<double>(this->safety_limits->max_speed_mps),
            static_cast<double>(SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS),
            static_cast<double>(SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS));
        this->safety_limits->max_speed_mps =
            SafetyLimits::HARD_MAX_SPEED_LIMIT_MPS;
    }

    // Check minimum state of charge
    if (!std::isfinite(this->safety_limits->min_soc) ||
        this->safety_limits->min_soc < SafetyLimits::HARD_MIN_SOC) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Got invalid minimum state of charge: %f%%, Using Internal "
                    "limit: %f",
                    static_cast<double>(this->safety_limits->min_soc),
                    static_cast<double>(SafetyLimits::HARD_MIN_SOC));
        this->safety_limits->min_soc = SafetyLimits::HARD_MIN_SOC;
    }

    // Check maximum altitude
    if (!std::isfinite(this->safety_limits->max_height_m) ||
        SafetyLimits::HARD_MAX_HEIGHT_M < this->safety_limits->max_height_m) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Got invalid maximum height: %fm, Using Internal limit: %f",
                    static_cast<double>(this->safety_limits->max_height_m),
                    static_cast<double>(SafetyLimits::HARD_MAX_HEIGHT_M));
        this->safety_limits->max_height_m = SafetyLimits::HARD_MAX_HEIGHT_M;
    }

    // Check geofence
    // No need to check if any of the polygon points are not finite because
    // currently the geofence does not modify the points themselves
    if (this->safety_limits->geofence.get_polygon_point_count() < 3) {
        RCLCPP_ERROR(
            this->get_safety_logger(),
            "Got an effective polygon with less then 3 points! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_DEBUG(this->get_safety_logger(),
                 "Validating safety limits successful");
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
            throw invalid_state_error(std::string(__func__) +
                                      " called while in ERROR state");
            // This function should never be called in these states
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(
                this->get_internal_state_logger(),
                "In an invalid state for a gps state check! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Verify that there is at least a 2D fix. (The No GPS antenna case
            // will be handled in the fallthrough cases)
            if (this->last_fcc_gps_info->fix_type ==
                mavsdk::Telemetry::FixType::NoFix) {
                // This is unrecoverable. If this happens on the ground
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
            throw unknown_enum_value_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // In state MAVSDK set up no safety limits are configured so a geofence
    // cannot be enforced
    if (this->get_internal_state() != INTERNAL_STATE::MAVSDK_SET_UP) {
        // Check that the current coordinate is inside the geofence.
        if (!this->check_point_in_geofence(this->last_fcc_position.value())) {
            if (this->is_airborne()) {
                // The current point is outside the geofence and the UAV is
                // airborne
                RCLCPP_ERROR(this->get_safety_logger(),
                             "The current position is not inside the geofence! "
                             "UAV is airborne. Triggering RTH...");
                this->trigger_rth();
                return;
            } else {
                // The current point is outside the geofence and the UAV is not
                // airborne
                RCLCPP_FATAL(this->get_safety_logger(),
                             "The current position is not inside the geofence! "
                             "UAV is not airborne. Exiting...");
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
        } else {
            RCLCPP_DEBUG(this->get_safety_logger(),
                         "Current UAV position is inside the geofence");
        }
    } else {
        RCLCPP_WARN_ONCE(this->get_safety_logger(),
                         "Cannot check current UAV position as safety limits "
                         "are not configured in state: %s",
                         this->internal_state_to_str());
    }

    RCLCPP_INFO(this->get_safety_logger(), "GPS state is O.K.");
}

void FCCBridgeNode::check_flight_state() {
    RCLCPP_WARN_ONCE(this->get_safety_logger(),
                     "Flight State check not fully implemented!");

    if (!this->check_landed_state()) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "LandedState checking triggered an RTH");
        return;
    }

    if (!this->check_flight_mode()) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "FlightMode checking triggered an RTH");
        return;
    }

    RCLCPP_INFO(this->get_safety_logger(), "FlightState check successful");
}

bool FCCBridgeNode::check_landed_state() {
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        // This should never happen, as the process exits on ERROR state
        throw invalid_state_error(std::string(__func__) +
                                  " called while in ERROR state");
    }

    if (this->get_internal_state() == INTERNAL_STATE::STARTING_UP ||
        this->get_internal_state() == INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "In an invalid state to check LandedState! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    // Ensuring there is a valid LandedState present
    if (!this->last_fcc_landed_state.has_value()) {
        RCLCPP_DEBUG(
            this->get_safety_logger(),
            "No cached LandedState found, getting an update from the FCC");
        this->get_flight_state();
    }

    const auto entry = landed_state_actions.find(this->get_internal_state());
    if (entry == landed_state_actions.end()) {
        throw unknown_enum_value_error(
            std::string("Got invalid value for internal_state: ") +
            std::to_string(static_cast<int>(this->get_internal_state())));
    }

    const state_action<LandedState> &actions = entry->second;
    if (actions.ERROR_STATES.find(this->last_fcc_landed_state.value()) !=
        actions.ERROR_STATES.end()) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "LandedState %s while in %s! Exiting...",
                     FCCBridgeNode::mavsdk_landed_state_to_str(
                         this->last_fcc_landed_state.value()),
                     this->internal_state_to_str());
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    } else if (actions.RTH_STATES.find(this->last_fcc_landed_state.value()) !=
               actions.RTH_STATES.end()) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "LandedState %s while in %s! Triggering RTH...",
                     FCCBridgeNode::mavsdk_landed_state_to_str(
                         this->last_fcc_landed_state.value()),
                     this->internal_state_to_str());
        this->trigger_rth();
        return false;
    } else if (actions.VALID_STATES.find(this->last_fcc_landed_state.value()) !=
               actions.VALID_STATES.end()) {
        return true;
    } else {
        throw unknown_enum_value_error(
            std::string("Got unknown "
                        "mavsdk::Telemetry::LandedState value: ") +
            std::to_string(
                static_cast<int>(this->last_fcc_landed_state.value())));
    }
}

bool FCCBridgeNode::check_flight_mode() {
    if (this->get_internal_state() == INTERNAL_STATE::ERROR) {
        // This should never happen, as the process exits on ERROR state
        throw invalid_state_error(std::string(__func__) +
                                  " called while in ERROR state");
    }

    if (this->get_internal_state() == INTERNAL_STATE::STARTING_UP ||
        this->get_internal_state() == INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "In an invalid state to check FlightMode! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    if (this->get_internal_state() == INTERNAL_STATE::MAVSDK_SET_UP ||
        this->get_internal_state() == INTERNAL_STATE::WAITING_FOR_ARM) {
        return true;
    }

    // Ensuring there is a valid LandedState present
    if (!this->last_fcc_flight_mode.has_value()) {
        RCLCPP_DEBUG(
            this->get_safety_logger(),
            "No cached FlightMode found, getting an update from the FCC");
        this->get_flight_state();
    }

    const auto entry = flight_mode_actions.find(this->get_internal_state());
    if (entry == flight_mode_actions.end()) {
        throw unknown_enum_value_error(
            std::string("Got invalid value for internal_state: ") +
            std::to_string(static_cast<int>(this->get_internal_state())));
    }

    const state_action<FlightMode> &actions = entry->second;
    if (actions.ERROR_STATES.find(this->last_fcc_flight_mode.value()) !=
        actions.ERROR_STATES.end()) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "FlightMode %s while in %s! Exiting...",
                     FCCBridgeNode::mavsdk_flight_mode_to_str(
                         this->last_fcc_flight_mode.value()),
                     this->internal_state_to_str());
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    } else if (actions.RTH_STATES.find(this->last_fcc_flight_mode.value()) !=
               actions.RTH_STATES.end()) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "FlightMode %s while in %s! Triggering RTH...",
                     FCCBridgeNode::mavsdk_flight_mode_to_str(
                         this->last_fcc_flight_mode.value()),
                     this->internal_state_to_str());
        this->trigger_rth();
        return false;
    } else if (actions.VALID_STATES.find(this->last_fcc_flight_mode.value()) !=
               actions.VALID_STATES.end()) {
        return true;
    } else {
        throw unknown_enum_value_error(
            std::string("Got unknown "
                        "mavsdk::Telemetry::FlightMode value: ") +
            std::to_string(
                static_cast<int>(this->last_fcc_landed_state.value())));
    }
}

void FCCBridgeNode::check_battery_state() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Checking battery state");

    if (this->get_internal_state() == INTERNAL_STATE::STARTING_UP ||
        this->get_internal_state() == INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "Current state %s is an invalid state for a battery "
                     "check! Exiting..",
                     this->internal_state_to_str());
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    if (this->get_internal_state() == INTERNAL_STATE::MAVSDK_SET_UP) {
        // Acceptable. In this state no safety limits are configured so a
        // check is not possible
        RCLCPP_WARN_ONCE(this->get_safety_logger(),
                         "Cannot check battery state, as no safety limits are "
                         "configured. Ignoring in state: %s",
                         this->internal_state_to_str());
        return;
    }

    if (!this->safety_limits.has_value()) {
        if (this->is_airborne()) {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Found no safety limits! UAV is airborne. "
                         "Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(
                this->get_safety_logger(),
                "Found no safety limits! UAV is not airborne. Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    // Ensuring there is a valid BatteryState present
    if (!this->last_fcc_battery_state.has_value()) {
        RCLCPP_DEBUG(this->get_safety_logger(),
                     "No cached BatteryState found, getting an update "
                     "from the FCC");
        this->get_flight_state();
    }

    if (this->last_fcc_battery_state->id != 0) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Got an unknown battery id: %" PRIu32,
                    this->last_fcc_battery_state->id);
    }

    if (!std::isfinite(this->last_fcc_battery_state->remaining_percent)) {
        if (this->is_airborne()) {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Battery remaining percent is not finite! UAV is "
                         "airborne. Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Battery remaining percent is not finite! UAV is "
                         "not airborne. Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    if (this->last_fcc_battery_state->remaining_percent <
        this->safety_limits->min_soc) {
        if (this->is_airborne()) {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Battery remaining percent is below minimum state "
                         "of charge! UAV is airborne. Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Battery remaining percent is below minimum state "
                         "of charge! UAV is not airborne. Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    RCLCPP_INFO(this->get_safety_logger(), "Battery check successful");
}

void FCCBridgeNode::check_rc_state() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Checking RC state");

    if (this->get_internal_state() == INTERNAL_STATE::STARTING_UP ||
        this->get_internal_state() == INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "In an invalid state for a rc state check! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    if (!this->last_fcc_rc_state.has_value()) {
        RCLCPP_DEBUG(this->get_safety_logger(),
                     "No cached RC state found, getting an update "
                     "from the FCC");
        this->get_rc_state();
    }

    if (!this->last_fcc_rc_state->is_available) {
        if (this->is_airborne()) {
            RCLCPP_FATAL(this->get_logger(),
                         "UAV lost RC connection. UAV is airborne. "
                         "Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(
                this->get_logger(),
                "UAV lost RC connection. UAV is not airborne. Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    RCLCPP_INFO(this->get_safety_logger(), "RC state check successful");
}

void FCCBridgeNode::check_uav_health() {
    RCLCPP_DEBUG(this->get_safety_logger(), "Checking UAV health state");

    switch (this->get_internal_state()) {
            // This function should never be called in these states
        case INTERNAL_STATE::ERROR:
            // This should never happen, as the process exits on ERROR state
            throw invalid_state_error(std::string(__func__) +
                                      " called while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            RCLCPP_FATAL(
                this->get_internal_state_logger(),
                "In an invalid state for a uav health check! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
            // Ensuring there is valid uav health present
            if (!this->last_fcc_health.has_value()) {
                RCLCPP_DEBUG(this->get_safety_logger(),
                             "No cached uav health found, getting an update "
                             "from the FCC");
                this->get_uav_health();
            }
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
            // Ensuring there is valid uav health present
            if (!this->last_fcc_health.has_value()) {
                RCLCPP_DEBUG(this->get_safety_logger(),
                             "No cached uav health found, getting an update "
                             "from the FCC");
                this->get_uav_health();
            }
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
            throw unknown_enum_value_error(
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
            throw invalid_state_error(std::string(__func__) +
                                      " called while in ERROR state");
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
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
            RCLCPP_WARN(this->get_safety_logger(),
                        "Safety limits not configured!");
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
            throw unknown_enum_value_error(
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

    if (!this->safety_limits->geofence.isIn({latitude_deg, longitude_deg})) {
        RCLCPP_WARN(this->get_safety_logger(), "Point is not inside geofence!");
        return false;
    }

    if (this->safety_limits->max_height_m < relative_altitude_m) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Point is above maximum altitude!");
        return false;
    }

    RCLCPP_DEBUG(this->get_safety_logger(), "Waypoint passed check");

    return true;
}

bool FCCBridgeNode::check_speed(const float speed_mps) {
    if (!this->safety_limits.has_value()) {
        RCLCPP_WARN(this->get_safety_logger(), "No safety limits configured");
        return false;
    }
    // Check that the speed is a finite float
    if (!std::isfinite(speed_mps)) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Latitude is not a finite number");
        return false;
    }

    if (this->safety_limits->max_speed_mps < speed_mps) {
        RCLCPP_WARN(this->get_safety_logger(),
                    "Current speed: %fm/s exceeds max speed: %fm/s",
                    static_cast<double>(speed_mps),
                    static_cast<double>(this->safety_limits->max_speed_mps));
        return false;
    }

    return true;
}

void FCCBridgeNode::check_last_mission_control_heartbeat() {
    RCLCPP_DEBUG(this->get_safety_logger(),
                 "Checking last mission control heartbeat");

    if (this->get_internal_state() == INTERNAL_STATE::STARTING_UP ||
        this->get_internal_state() == INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_FATAL(this->get_internal_state_logger(),
                     "In an invalid state for a mission control heartbeat "
                     "check check! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    if (!this->last_mission_control_heartbeat.has_value()) {
        if (this->get_internal_state() == INTERNAL_STATE::MAVSDK_SET_UP) {
            RCLCPP_WARN(this->get_safety_logger(),
                        "Got no cached mission control heartbeat. "
                        "Acceptable while in state %s",
                        this->internal_state_to_str());
            return;
        } else if (this->is_airborne()) {
            RCLCPP_ERROR(
                this->get_safety_logger(),
                "Got no cached mission control heartbeat! Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(this->get_safety_logger(),
                         "Got no cached mission control heartbeat! EXITING...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    if (rclcpp::Duration(MAX_MISSION_CONTROL_HEARTBEAT_AGE) <
        (this->now() - this->last_mission_control_heartbeat->time_stamp)) {
        if (this->get_internal_state() == INTERNAL_STATE::RETURN_TO_HOME ||
            this->get_internal_state() == INTERNAL_STATE::LANDED) {
            RCLCPP_WARN(
                this->get_safety_logger(),
                "Mission control timed out while in state %s. Ignoring...",
                this->internal_state_to_str());
            return;
        }
        if (this->is_airborne()) {
            RCLCPP_FATAL(
                this->get_safety_logger(),
                "The last mission control heartbeat is older than: %" PRId64
                "ms. UAV is airborne. Triggering RTH...",
                MAX_MISSION_CONTROL_HEARTBEAT_AGE.count());
            this->trigger_rth();
            return;
        } else {
            RCLCPP_FATAL(
                this->get_safety_logger(),
                "The last mission control heartbeat is older than: %" PRId64
                "ms. UAV is not airborne. Exiting...",
                MAX_MISSION_CONTROL_HEARTBEAT_AGE.count());
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    RCLCPP_INFO(this->get_safety_logger(),
                "Mission control heartbeat check successful");
}

}  // namespace fcc_bridge
