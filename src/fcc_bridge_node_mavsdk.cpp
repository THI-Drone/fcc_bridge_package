//
// Created by Johan on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

// Libc header
#include <map>

namespace fcc_bridge {
namespace {
// Path to uart device on Raspberry PI
constexpr char const *UART_DEVICE_PATH = "/dev/serial0";

// Baud rate to use for uart connection
constexpr int UART_BAUD_RATE = 112500;

// Simulator IP address
constexpr char const *SIMULATOR_IP_ADDRESS = "127.0.0.1";
// Simulator TCP Port number
constexpr int SIMULATOR_PORT_NUMBER = 4560;

// Map to get the MAVLink System id for the selected
// {TEAM_ID : {SYS_ID, IS_SIMULATOR_TARGET}}
const std::map<const std::string, std::pair<const u8, const bool>> SYS_ID_MAP{
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
    if (this->internal_state != INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_ERROR(this->get_logger(),
                     "Repeated try to setup MAVSDK components");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

    // Getting uav id to deduct MAVLink System ID
    const rclcpp::ParameterValue &uav_id_parameter =
        this->declare_parameter("UAV_ID", rclcpp::ParameterValue());
    // Checking if parameter was set
    if (uav_id_parameter.get_type() ==
        rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_FATAL(this->get_logger(), "UAV_ID parameter was not set!");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }
    // Checking type of parameter
    if (uav_id_parameter.get_type() !=
        rclcpp::ParameterType::PARAMETER_STRING) {
        RCLCPP_FATAL(this->get_logger(), "UAV_ID parameter was of wrong type!");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }
    // Checking of for the supplied uav id there is an entry in the SYS_ID_MAP
    const std::map<const std::string,
                   std::pair<const u8, const bool>>::const_iterator
        uav_id_map_entry = SYS_ID_MAP.find(uav_id_parameter.get<std::string>());
    if (uav_id_map_entry == SYS_ID_MAP.end()) {
        RCLCPP_FATAL(this->get_logger(), "Got unknown UAV_ID!");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

    // Creating a MAVSDK instance with the provided System ID as an onboard
    // computer
    this->mavsdk.emplace(mavsdk::Mavsdk::Configuration(
        std::get<const u8>(uav_id_map_entry->second),
        MAV_COMP_ID_ONBOARD_COMPUTER, false));

    // The result of the connection attempt to the MAVLink network will be
    // stored here
    mavsdk::ConnectionResult connection_result;

    // Checking if the target UAV is a simulation target
    if (std::get<const bool>(uav_id_map_entry->second)) {
        RCLCPP_WARN(this->get_logger(), "Connecting to simulator!");
        // Trying to connect to the simulated MAVLink network
        connection_result = this->mavsdk->add_tcp_connection(
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
        RCLCPP_DEBUG(this->get_logger(), "Error code: %d",
                     static_cast<int>(connection_result));
        this->internal_state = INTERNAL_STATE::ERROR;
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
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }
    this->mavsdk_system = potential_system.value();

    // Verifying that the system is connected
    if (!this->mavsdk_system->is_connected()) {
        RCLCPP_FATAL(this->get_logger(), "Connection to system has failed!");
        this->internal_state = INTERNAL_STATE::ERROR;
        return;
    }

    // Verifying that the system has an autopilot
    if (!this->mavsdk_system->has_autopilot()) {
        RCLCPP_FATAL(this->get_logger(),
                     "The MAVSDK system does not seem to have an autopilot!");
        this->internal_state = INTERNAL_STATE::ERROR;
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

void FCCBridgeNode::verify_connection() {
    RCLCPP_DEBUG(this->get_logger(), "Verifying MAVSDK connection");
    switch (this->internal_state) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::LANDED:
            this->internal_state = INTERNAL_STATE::ERROR;
            [[fallthrough]];
        case INTERNAL_STATE::ERROR:
            RCLCPP_ERROR(this->get_logger(),
                         "MAVSDK is not set up! Exiting...");
            std::exit(EXIT_FAILURE);
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
            this->internal_state = INTERNAL_STATE::ERROR;
            std::exit(EXIT_FAILURE);
    }
}

void FCCBridgeNode::get_gps_telemetry() {
    // Clearing cached values
    this->last_fcc_gps_info = std::nullopt;
    this->last_fcc_position = std::nullopt;

    this->verify_connection();

    RCLCPP_DEBUG(this->get_logger(), "Getting GPSInfo from FCC");
    this->last_fcc_gps_info = this->mavsdk_telemtry->gps_info();

    RCLCPP_DEBUG(this->get_logger(), "Getting position from FCC");
    this->last_fcc_position = this->mavsdk_telemtry->position();

    RCLCPP_INFO(this->get_logger(),
                "Current position of UAV: Lat: %f°\tLon: %f°\tAbsolute "
                "attitude: %fm\trelative attitude: %fm\tNo. of satellites: %d",
                this->last_fcc_position->latitude_deg,
                this->last_fcc_position->longitude_deg,
                this->last_fcc_position->absolute_altitude_m,
                this->last_fcc_position->relative_altitude_m,
                this->last_fcc_gps_info->num_satellites);
}

void FCCBridgeNode::trigger_rth() {
    RCLCPP_WARN(this->get_logger(), "Triggering RTH");
}
}  // namespace fcc_bridge