//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <cinttypes>
#include <map>

namespace fcc_bridge {
namespace {
// Path to uart device on Raspberry PI
constexpr char const *const UART_DEVICE_PATH = "/dev/serial0";

// Baud rate to use for uart connection
constexpr int UART_BAUD_RATE = 57600;

// Simulator IP address
constexpr char const *const SIMULATOR_IP_ADDRESS = "127.0.0.1";
// Simulator UDP Port number
constexpr int SIMULATOR_PORT_NUMBER = 14550;

/**
 * @brief Struct to hold an entry in @ref fcc_bridge::SYS_ID_MAP
 */
struct sys_id_map_entry {
    const u8 MAVLINK_SYS_ID; /**< The MAVLink System ID to use */
    const bool
        IS_SIMULATOR_TARGET; /**< If the selected target is a simulator */
    /**
     * @brief Constructor to create an instance of this struct in a well defined
     * manner
     * @param MAVLINK_SYS_ID The MAVLink System ID to use
     * @param IS_SIMULATOR_TARGET If the selected target is a simulator
     */
    constexpr sys_id_map_entry(const u8 MAVLINK_SYS_ID,
                               const bool IS_SIMULATOR_TARGET)
        : MAVLINK_SYS_ID(MAVLINK_SYS_ID),
          IS_SIMULATOR_TARGET(IS_SIMULATOR_TARGET) {}
};

// Map to get the MAVLink System id for the selected
// {TEAM_ID : {MAVLINK_SYS_ID, IS_SIMULATOR_TARGET}}
const std::map<const std::string, const struct sys_id_map_entry> SYS_ID_MAP{
    {"UAV_TEAM_RED", {21, false}},
    {"UAV_TEAM_GREEN", {22, false}},
    {"UAV_TEAM_BLUE", {23, false}},
    {"SIMULATOR", {1, true}}};

// Timeout for the autopilot to be discovered
constexpr double AUTOPILOT_DISCOVERY_TIMEOUT_S = 1;
}  // namespace

void FCCBridgeNode::setup_mavsdk() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up MAVSDK");

    // Internal state check
    if (this->get_internal_state() != INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_ERROR(this->get_logger(),
                     "Repeated try to setup MAVSDK components");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }
    std::string uav_id;
    try {
        // Getting uav id to deduct MAVLink System ID
        uav_id = this->declare_parameter<std::string>("UAV_ID");
    } catch (
        const rclcpp::exceptions::UninitializedStaticallyTypedParameterException
            &e) {
        // Catch if parameter was not set
        RCLCPP_FATAL(this->get_logger(), "UAV_ID parameter was not set!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
        // Catch if parameter is not a string
        RCLCPP_FATAL(
            this->get_logger(),
            "UAV_ID parameter was of wrong type! It has to be a string!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Checking if there is an entry in the SYS_ID_MAP for the supplied UAV_ID
    const std::map<const std::string,
                   const struct sys_id_map_entry>::const_iterator
        uav_id_map_entry = SYS_ID_MAP.find(uav_id);
    if (uav_id_map_entry == SYS_ID_MAP.end()) {
        RCLCPP_FATAL(this->get_logger(), "Got unknown UAV_ID!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Creating a MAVSDK instance with the provided System ID as an onboard
    // computer
    this->mavsdk.emplace(
        mavsdk::Mavsdk::Configuration(uav_id_map_entry->second.MAVLINK_SYS_ID,
                                      MAV_COMP_ID_ONBOARD_COMPUTER, false));

    // The result of the connection attempt to the MAVLink network will be
    // stored here
    mavsdk::ConnectionResult connection_result;

    // Checking if the target UAV is a simulation target
    if (uav_id_map_entry->second.IS_SIMULATOR_TARGET) {
        RCLCPP_WARN(this->get_logger(), "Connecting to simulator!");
        // Trying to connect to the simulated MAVLink network
        connection_result = this->mavsdk->add_udp_connection(
            SIMULATOR_IP_ADDRESS, SIMULATOR_PORT_NUMBER,
            mavsdk::ForwardingOption::ForwardingOff);
    } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "Creating a connection to a real MAVLink network");
        // Trying to connect to the real MAVLink network
        connection_result = this->mavsdk->add_serial_connection(
            UART_DEVICE_PATH, UART_BAUD_RATE, false,
            mavsdk::ForwardingOption::ForwardingOff);
    }
    // Checking if connection succeeded
    if (connection_result != mavsdk::ConnectionResult::Success) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to establish MAVSDK connection");
        RCLCPP_DEBUG(
            this->get_logger(), "Error code: %s",
            FCCBridgeNode::mavsdk_connection_result_to_str(connection_result));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Trying to get an autopilot system. There should only be one.
    const std::optional<std::shared_ptr<mavsdk::System>> potential_system =
        this->mavsdk->first_autopilot(AUTOPILOT_DISCOVERY_TIMEOUT_S);
    // Checking if an autopilot was found
    if (!potential_system.has_value()) {
        RCLCPP_FATAL(this->get_logger(),
                     "Could not discover an autopilot within %f seconds",
                     AUTOPILOT_DISCOVERY_TIMEOUT_S);
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }
    this->mavsdk_system = potential_system.value();

    // Verifying that the system is connected
    if (!this->mavsdk_system->is_connected()) {
        RCLCPP_FATAL(this->get_logger(), "Connection to system has failed!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Verifying that the system has an autopilot
    if (!this->mavsdk_system->has_autopilot()) {
        RCLCPP_FATAL(this->get_logger(),
                     "The MAVSDK system does not seem to have an autopilot!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Setting up mavsdk telemetry
    this->mavsdk_telemtry.emplace(this->mavsdk_system);

    // Setting up mavsdk action
    this->mavsdk_action.emplace(this->mavsdk_system);

    // Setting up mavsdk mission
    this->mavsdk_mission.emplace(this->mavsdk_system);

    RCLCPP_INFO(this->get_logger(), "Successfully set up MAVSDK components");
}

void FCCBridgeNode::verify_mavsdk_connection() {
    RCLCPP_DEBUG(this->get_logger(), "Verifying MAVSDK connection");
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::LANDED:
            this->set_internal_state(INTERNAL_STATE::ERROR);
            [[fallthrough]];
        case INTERNAL_STATE::ERROR:
            RCLCPP_ERROR(this->get_logger(),
                         "MAVSDK is not set up! Exiting...");
            // Does not return
            this->exit_process_on_error();
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_ACTION:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::RETURN_TO_HOME:
            if (this->mavsdk_system->is_connected()) {
                RCLCPP_INFO(this->get_logger(), "MAVSDK state is good");
                return;
            }
            RCLCPP_FATAL(this->get_logger(),
                         "An MAVSDK error was encountered! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            // Does not return
            this->exit_process_on_error();
        default:
            throw std::runtime_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }
}

void FCCBridgeNode::get_gps_telemetry() {
    // Clear cached values
    this->last_fcc_gps_info = std::nullopt;
    this->last_fcc_position = std::nullopt;

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get GPSInfo
    RCLCPP_DEBUG(this->get_logger(), "Getting GPSInfo from FCC");
    this->last_fcc_gps_info = this->mavsdk_telemtry->gps_info();

    // Get GPS position
    RCLCPP_DEBUG(this->get_logger(), "Getting position from FCC");
    this->last_fcc_position = this->mavsdk_telemtry->position();

    RCLCPP_INFO(
        this->get_logger(),
        "Current position of UAV: Lat: %f°\tLon: %f°\tAbsolute "
        "attitude: %fm\trelative attitude: %fm\tNo. of satellites: %" PRId32,
        this->last_fcc_position->latitude_deg,
        this->last_fcc_position->longitude_deg,
        static_cast<double>(this->last_fcc_position->absolute_altitude_m),
        static_cast<double>(this->last_fcc_position->relative_altitude_m),
        this->last_fcc_gps_info->num_satellites);  // TODO: Print out fix type
}

void FCCBridgeNode::get_flight_state() {
    // Clear Cached values
    this->last_fcc_flight_mode = std::nullopt;
    this->last_fcc_landed_state = std::nullopt;

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get flight mode
    RCLCPP_DEBUG(this->get_logger(), "Getting FlightMode from FCC");
    this->last_fcc_flight_mode = this->mavsdk_telemtry->flight_mode();

    RCLCPP_INFO(this->get_logger(), "The current flight mode is: %s",
                FCCBridgeNode::mavsdk_flight_mode_to_str(
                    this->last_fcc_flight_mode.value()));

    // Get landed state
    RCLCPP_DEBUG(this->get_logger(), "Getting LandedState from FCC");
    this->last_fcc_landed_state = this->mavsdk_telemtry->landed_state();

    RCLCPP_INFO(
        this->get_logger(), "The current landed state is: %s",
        this->mavsdk_landed_state_to_str(this->last_fcc_landed_state.value()));
}

void FCCBridgeNode::get_battery_state() {
    // Clear cached values
    this->last_fcc_battery_state = std::nullopt;

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get battery state
    RCLCPP_DEBUG(this->get_logger(), "Getting Battery state from FCC");
    this->last_fcc_battery_state = this->mavsdk_telemtry->battery();

    RCLCPP_INFO(
        this->get_logger(),
        "The current FCC battery state: Battery id: %" PRIu32
        "\tTemperature: %f°C\tVoltage: %fV\tBattery current: "
        "%fA\tConsumed capacity: %fAh\tRemaining percent: %f%%",
        this->last_fcc_battery_state->id,
        static_cast<double>(this->last_fcc_battery_state->temperature_degc),
        static_cast<double>(this->last_fcc_battery_state->voltage_v),
        static_cast<double>(this->last_fcc_battery_state->current_battery_a),
        static_cast<double>(this->last_fcc_battery_state->capacity_consumed_ah),
        static_cast<double>(this->last_fcc_battery_state->remaining_percent));
}

void FCCBridgeNode::get_rc_state() {
    // Clear cached values
    this->last_fcc_rc_state = std::nullopt;

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get RC state
    RCLCPP_DEBUG(this->get_logger(), "Getting RC state from FCC");
    this->last_fcc_rc_state = this->mavsdk_telemtry->rc_status();

    RCLCPP_INFO(
        this->get_logger(),
        "The current FCC RC state: RC was available once: %s\tRC is "
        "available: %s\t Signal strength: %f%%",
        this->last_fcc_rc_state->was_available_once ? "true" : "false",
        this->last_fcc_rc_state->is_available ? "true" : "false",
        static_cast<double>(this->last_fcc_rc_state->signal_strength_percent));
}

void FCCBridgeNode::get_euler_angle() {
    // Clear cached values
    this->last_fcc_euler_angle = std::nullopt;

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get euler angle
    RCLCPP_DEBUG(this->get_logger(), "Getting euler angle from FCC");
    this->last_fcc_euler_angle = this->mavsdk_telemtry->attitude_euler();

    RCLCPP_INFO(this->get_logger(),
                "The current FCC euler angle: Roll %f° (Positive means banking "
                "to the right)\tPitch: %f° (Positive means Nose up)\tYaw: "
                "%f°(Clockwise from above)",
                static_cast<double>(this->last_fcc_euler_angle->roll_deg),
                static_cast<double>(this->last_fcc_euler_angle->pitch_deg),
                static_cast<double>(this->last_fcc_euler_angle->yaw_deg));
}

void FCCBridgeNode::get_mission_progress() {
    // Clear cached values
    this->last_mission_progress = std::nullopt;

    // Verify the MAVSDK connection
    this->verify_mavsdk_connection();

    // Get mission progress
    RCLCPP_DEBUG(this->get_logger(), "Getting mission progress");
    this->last_mission_progress = this->mavsdk_mission->is_mission_finished();

    RCLCPP_INFO(this->get_logger(),
                "The current mission is finished: %s\t Result code: %s",
                this->last_mission_progress->second ? "true" : "false",
                FCCBridgeNode::mavsdk_mission_result_to_str(
                    this->last_mission_progress->first));
}

void FCCBridgeNode::get_uav_health() {
    // Clear cached values
    this->last_fcc_health = std::nullopt;

    // Verify the MAVSDK connection
    this->verify_mavsdk_connection();

    // Get the UAV health
    RCLCPP_DEBUG(this->get_logger(), "Getting UAV health");
    this->last_fcc_health = this->mavsdk_telemtry->health();

    static auto bool_to_str = [](const bool b) -> const char * {
        return b ? "true" : "false";
    };

    RCLCPP_INFO(
        this->get_logger(),
        "Current UAV health: Gyrometer calibrated: %s\tAccelerometer "
        "calibrated: %s\tMagnetometer calibrated: %s\tLocal position ok: "
        "%s\tGlobal position ok: %s\tHome position ok: %s\tArmable: %s",
        bool_to_str(this->last_fcc_health->is_gyrometer_calibration_ok),
        bool_to_str(this->last_fcc_health->is_accelerometer_calibration_ok),
        bool_to_str(this->last_fcc_health->is_magnetometer_calibration_ok),
        bool_to_str(this->last_fcc_health->is_local_position_ok),
        bool_to_str(this->last_fcc_health->is_global_position_ok),
        bool_to_str(this->last_fcc_health->is_home_position_ok),
        bool_to_str(this->last_fcc_health->is_armable));
}

void FCCBridgeNode::trigger_rth() {
    RCLCPP_WARN(this->get_logger(), "Triggering RTH");

    this->set_internal_state(INTERNAL_STATE::RETURN_TO_HOME);

    this->deactivate();
}

void FCCBridgeNode::exit_process_on_error() {
    RCLCPP_WARN(this->get_logger(), "Exiting process with EXIT_FAILURE");
    std::exit(EXIT_FAILURE);
}
}  // namespace fcc_bridge