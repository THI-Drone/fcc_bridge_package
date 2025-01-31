//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

// Libc header
#include <cinttypes>
#include <map>
#include <thread>

// FCC bridge header
#include "fcc_bridge/fcc_bridge_node.hpp"

namespace fcc_bridge {
namespace {

constexpr char const *const UART_DEVICE_PATH =
    "/dev/fcc_uart"; /**< Path to uart device on Raspberry PI */

constexpr int UART_BAUD_RATE =
    115200; /**< Baud rate to use for uart connection */

constexpr char const *const SIMULATOR_IP_ADDRESS =
    "127.0.0.1"; /**< Simulator IP address */

constexpr int SIMULATOR_PORT_NUMBER = 14540; /**< Simulator UDP Port number */

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
     * @param mavlink_sys_id The MAVLink System ID to use
     * @param IS_SIMULATOR_TARGET If the selected target is a simulator
     */
    constexpr sys_id_map_entry(const u8 mavlink_sys_id,
                               const bool is_simulator_target)
        : MAVLINK_SYS_ID(mavlink_sys_id),
          IS_SIMULATOR_TARGET(is_simulator_target) {}
};

// Map to get the MAVLink System id for the selected
// {TEAM_ID : {MAVLINK_SYS_ID, IS_SIMULATOR_TARGET}}
const std::map<const std::string, const struct sys_id_map_entry> SYS_ID_MAP{
    {"UAV_FFI_1", {1, false}},       {"UAV_FFI_2", {2, false}},
    {"UAV_FFI_3", {3, false}},       {"UAV_FFI_4", {4, false}},
    {"UAV_FFI_5", {5, false}},       {"UAV_TEAM_RED", {21, false}},
    {"UAV_TEAM_GREEN", {22, false}}, {"UAV_TEAM_BLUE", {23, false}},
    {"SIMULATOR", {1, true}}};

constexpr double AUTOPILOT_DISCOVERY_TIMEOUT_S =
    5; /**< Timeout for the autopilot to be discovered */

constexpr std::chrono::milliseconds MAVSDK_WAIT_TIME{
    2500}; /**< Time to wait after MAVSDK has been set up and connected to the
              FCC to let the system collect all telemetery values */

constexpr u8 MAX_MISSION_START_RETRY_COUNT =
    3; /**< Max retry count for mission start if the FCC denies */

}  // namespace

void FCCBridgeNode::setup_mavsdk() {
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(), "Setting up MAVSDK");

    mavsdk::log::subscribe(std::bind(
        &FCCBridgeNode::mavsdk_log_callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Installed MAVSDK Log callback");

    // Internal state check
    if (this->get_internal_state() != INTERNAL_STATE::ROS_SET_UP) {
        RCLCPP_ERROR(this->get_mavsdk_interface_logger(),
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
        RCLCPP_FATAL(this->get_ros_interface_logger(),
                     "UAV_ID parameter was not set!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
        // Catch if parameter is not a string
        RCLCPP_FATAL(
            this->get_ros_interface_logger(),
            "UAV_ID parameter was of wrong type! It has to be a string!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Checking if there is an entry in the SYS_ID_MAP for the supplied UAV_ID
    const std::map<const std::string,
                   const struct sys_id_map_entry>::const_iterator
        uav_id_map_entry = SYS_ID_MAP.find(uav_id);
    if (uav_id_map_entry == SYS_ID_MAP.end()) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Got unknown UAV_ID!");
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
        RCLCPP_WARN(this->get_mavsdk_interface_logger(),
                    "Connecting to simulator!");
        // Trying to connect to the simulated MAVLink network
        connection_result = this->mavsdk->add_udp_connection(
            SIMULATOR_IP_ADDRESS, SIMULATOR_PORT_NUMBER,
            mavsdk::ForwardingOption::ForwardingOff);
    } else {
        RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                     "Creating a connection to a real MAVLink network");
        // Trying to connect to the real MAVLink network
        connection_result = this->mavsdk->add_serial_connection(
            UART_DEVICE_PATH, UART_BAUD_RATE, false,
            mavsdk::ForwardingOption::ForwardingOff);
    }
    // Checking if connection succeeded
    if (connection_result != mavsdk::ConnectionResult::Success) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Failed to establish MAVSDK connection");
        RCLCPP_DEBUG(
            this->get_mavsdk_interface_logger(), "Error code: %s",
            FCCBridgeNode::mavsdk_connection_result_to_str(connection_result));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Trying to get an autopilot system. There should only be one.
    const std::optional<std::shared_ptr<mavsdk::System>> potential_system =
        this->mavsdk->first_autopilot(AUTOPILOT_DISCOVERY_TIMEOUT_S);
    // Checking if an autopilot was found
    if (!potential_system.has_value()) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Could not discover an autopilot within %f seconds",
                     AUTOPILOT_DISCOVERY_TIMEOUT_S);
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }
    this->mavsdk_system = potential_system.value();

    // Verifying that the system is connected
    if (!this->mavsdk_system->is_connected()) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Connection to system has failed!");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    // Verifying that the system has an autopilot
    if (!this->mavsdk_system->has_autopilot()) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
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

    // Set Telemetry rates so that MAVSDK has fresh values in its cache

    // 10Hz Telemetry
    this->check_telemetry_result(this->mavsdk_telemtry->set_rate_gps_info(10.0),
                                 "GPSInfo");
    this->check_telemetry_result(this->mavsdk_telemtry->set_rate_position(10.0),
                                 "Position");
    this->check_telemetry_result(
        this->mavsdk_telemtry->set_rate_attitude_euler(10.0), "EulerAngle");

    // 5Hz Telemetry
    this->check_telemetry_result(this->mavsdk_telemtry->set_rate_battery(5.0),
                                 "BatteryState");
    this->check_telemetry_result(this->mavsdk_telemtry->set_rate_home(5.0),
                                 "Home");
    this->check_telemetry_result(this->mavsdk_telemtry->set_rate_in_air(5.0),
                                 "In Air");
    this->check_telemetry_result(
        this->mavsdk_telemtry->set_rate_landed_state(5.0), "LandedState");

    // Wait so that MAVSDK can get correct values
    std::this_thread::sleep_for(MAVSDK_WAIT_TIME);

    const mavsdk::Info info{this->mavsdk_system};

    const std::pair<mavsdk::Info::Result, mavsdk::Info::FlightInfo> flight_info{
        info.get_flight_information()};

    if (flight_info.first != mavsdk::Info::Result::Success) {
        RCLCPP_FATAL(
            this->get_mavsdk_interface_logger(),
            "Failed to get flight info from FCC with error: %s",
            FCCBridgeNode::mavsdk_info_result_to_str(flight_info.first));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "Got flight info from FCC: boot time: %" PRIu32
                "ms\tFlight uid: %" PRIu64,
                flight_info.second.time_boot_ms, flight_info.second.flight_uid);

    const std::pair<mavsdk::Info::Result, mavsdk::Info::Identification>
        identification{info.get_identification()};

    if (identification.first != mavsdk::Info::Result::Success) {
        RCLCPP_FATAL(
            this->get_mavsdk_interface_logger(),
            "Failed to get identification from FCC with error: %s",
            FCCBridgeNode::mavsdk_info_result_to_str(identification.first));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "Got identification from FCC: hardware uid: %s\t legacy uid: "
                "%#016" PRIX64,
                identification.second.hardware_uid.c_str(),
                identification.second.legacy_uid);

    const std::pair<mavsdk::Info::Result, mavsdk::Info::Product> product{
        info.get_product()};

    if (product.first != mavsdk::Info::Result::Success) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Failed to get product from FCC with error: %s",
                     FCCBridgeNode::mavsdk_info_result_to_str(product.first));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "Got product info from FCC: vendor id: %" PRId32
                "\tvendor name: %s\t product id: %" PRId32 "\tproduct name: %s",
                product.second.vendor_id, product.second.vendor_name.c_str(),
                product.second.product_id, product.second.product_name.c_str());

    const std::pair<mavsdk::Info::Result, mavsdk::Info::Version> version{
        info.get_version()};

    if (version.first != mavsdk::Info::Result::Success) {
        RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                     "Failed to get version from FCC with error: %s",
                     FCCBridgeNode::mavsdk_info_result_to_str(version.first));
        this->set_internal_state(INTERNAL_STATE::ERROR);
        return;
    }

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "Got version info from FCC: Flight SW major: %" PRId32
                "\tFlight SW minor: %" PRId32 "\tFlight SW patch: %" PRId32
                "\tFlight SW git hash: %s\tFlight SW type: %s\tFlight SW "
                "vendor major: %" PRId32 "\tFlight SW vendor minor: %" PRId32
                "\tFlight SW vendor patch: %" PRId32 "\tOS SW major: %" PRId32
                "\tOS SW minor: %" PRId32 "\tOS SW patch: %" PRId32
                "\tOs SW git hash: %s",
                version.second.flight_sw_major, version.second.flight_sw_minor,
                version.second.flight_sw_patch,
                version.second.flight_sw_git_hash.c_str(),
                FCCBridgeNode::mavsdk_info_version_flight_sw_version_to_str(
                    version.second.flight_sw_version_type),
                version.second.flight_sw_vendor_major,
                version.second.flight_sw_vendor_minor,
                version.second.flight_sw_vendor_patch,
                version.second.os_sw_major, version.second.os_sw_minor,
                version.second.os_sw_patch,
                version.second.os_sw_git_hash.c_str());

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "Successfully set up MAVSDK components");
}

void FCCBridgeNode::verify_mavsdk_connection() {
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Verifying MAVSDK connection");
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
            this->set_internal_state(INTERNAL_STATE::ERROR);
            [[fallthrough]];
        case INTERNAL_STATE::ERROR:
            RCLCPP_ERROR(this->get_mavsdk_interface_logger(),
                         "MAVSDK is not set up! Exiting...");
            // Does not return
            this->exit_process_on_error();
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
        case INTERNAL_STATE::RETURN_TO_HOME:
        case INTERNAL_STATE::LANDED:
            if (this->mavsdk_system->is_connected()) {
                RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                            "MAVSDK state is good");
                return;
            }
            RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                         "An MAVSDK error was encountered! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            // Does not return
            this->exit_process_on_error();
        default:
            throw unknown_enum_value_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }
}

void FCCBridgeNode::get_gps_telemetry() {
    // Clear cached values
    this->last_fcc_gps_info.reset();
    this->last_fcc_position.reset();

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get GPSInfo
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting GPSInfo from FCC");
    this->last_fcc_gps_info = this->mavsdk_telemtry->gps_info();

    // Get GPS position
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting position from FCC");
    this->last_fcc_position = this->mavsdk_telemtry->position();

    RCLCPP_INFO(
        this->get_fcc_telemetry_logger(),
        "Current position of UAV: Lat: %f°\tLon: %f°\tAbsolute attitude: "
        "%fm\trelative attitude: %fm\tNo. of satellites: %" PRId32
        "\tFix type: %s",
        this->last_fcc_position->latitude_deg,
        this->last_fcc_position->longitude_deg,
        static_cast<double>(this->last_fcc_position->absolute_altitude_m),
        static_cast<double>(this->last_fcc_position->relative_altitude_m),
        this->last_fcc_gps_info->num_satellites,
        FCCBridgeNode::mavsdk_fix_type_to_str(
            this->last_fcc_gps_info->fix_type));
}

void FCCBridgeNode::get_flight_state() {
    // Clear Cached values
    this->last_fcc_flight_mode.reset();
    this->last_fcc_landed_state.reset();
    this->last_fcc_armed_state.reset();

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get flight mode
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting FlightMode from FCC");
    this->last_fcc_flight_mode = this->mavsdk_telemtry->flight_mode();

    RCLCPP_INFO(this->get_fcc_telemetry_logger(),
                "The current flight mode is: %s",
                FCCBridgeNode::mavsdk_flight_mode_to_str(
                    this->last_fcc_flight_mode.value()));

    // Get landed state
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting LandedState from FCC");
    this->last_fcc_landed_state = this->mavsdk_telemtry->landed_state();

    RCLCPP_INFO(
        this->get_fcc_telemetry_logger(), "The current landed state is: %s",
        this->mavsdk_landed_state_to_str(this->last_fcc_landed_state.value()));

    // Get armed state
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting armed state from FCC");
    this->last_fcc_armed_state = this->mavsdk_telemtry->armed();

    RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                "The current armed state is: %s",
                this->last_fcc_armed_state.value() ? "true" : "false");
}

void FCCBridgeNode::get_battery_state() {
    // Clear cached values
    this->last_fcc_battery_state.reset();

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get battery state
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting Battery state from FCC");
    this->last_fcc_battery_state = this->mavsdk_telemtry->battery();

    RCLCPP_INFO(
        this->get_fcc_telemetry_logger(),
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
    this->last_fcc_rc_state.reset();

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get RC state
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting RC state from FCC");
    this->last_fcc_rc_state = this->mavsdk_telemtry->rc_status();

    RCLCPP_INFO(
        this->get_fcc_telemetry_logger(),
        "The current FCC RC state: RC was available once: %s\tRC is available: "
        "%s\t Signal strength: %f%%",
        this->last_fcc_rc_state->was_available_once ? "true" : "false",
        this->last_fcc_rc_state->is_available ? "true" : "false",
        static_cast<double>(this->last_fcc_rc_state->signal_strength_percent));
}

void FCCBridgeNode::get_euler_angle() {
    // Clear cached values
    this->last_fcc_euler_angle.reset();

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Get euler angle
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting euler angle from FCC");
    this->last_fcc_euler_angle = this->mavsdk_telemtry->attitude_euler();

    RCLCPP_INFO(this->get_fcc_telemetry_logger(),
                "The current FCC euler angle: Roll %f° (Positive means banking "
                "to the right)\tPitch: %f° (Positive means Nose up)\tYaw: "
                "%f°(Clockwise from above)",
                static_cast<double>(this->last_fcc_euler_angle->roll_deg),
                static_cast<double>(this->last_fcc_euler_angle->pitch_deg),
                static_cast<double>(this->last_fcc_euler_angle->yaw_deg));
}

void FCCBridgeNode::get_mission_progress() {
    // Clear cached values
    this->last_mission_progress.reset();

    // Verify the MAVSDK connection
    this->verify_mavsdk_connection();

    // Get mission progress
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Getting mission progress");
    this->last_mission_progress = this->mavsdk_mission->is_mission_finished();

    RCLCPP_INFO(this->get_fcc_telemetry_logger(),
                "The current mission is finished: %s\t Result code: %s",
                this->last_mission_progress->second ? "true" : "false",
                FCCBridgeNode::mavsdk_mission_result_to_str(
                    this->last_mission_progress->first));
}

void FCCBridgeNode::get_uav_health() {
    // Clear cached values
    this->last_fcc_health.reset();

    // Verify the MAVSDK connection
    this->verify_mavsdk_connection();

    // Get the UAV health
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(), "Getting UAV health");
    this->last_fcc_health = this->mavsdk_telemtry->health();

    static auto bool_to_str = [](const bool b) -> const char * {
        return b ? "true" : "false";
    };

    RCLCPP_INFO(
        this->get_fcc_telemetry_logger(),
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

bool FCCBridgeNode::execute_mission_plan(
    const mavsdk::Mission::MissionPlan &plan) {
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(),
                 "Got new mission plan to execute");

    RCLCPP_DEBUG_STREAM(this->get_mavsdk_interface_logger(),
                        "Executing mission: " << plan);

    // Verify MAVSDK connection
    this->verify_mavsdk_connection();

    // Ensure there is no RTH after the mission ends
    const mavsdk::Mission::Result no_rth_after_mission_result =
        this->mavsdk_mission->set_return_to_launch_after_mission(false);

    if (no_rth_after_mission_result != mavsdk::Mission::Result::Success) {
        RCLCPP_ERROR(
            this->get_mavsdk_interface_logger(),
            "Failed to disable automatic RTH after mission end with result: %s",
            FCCBridgeNode::mavsdk_mission_result_to_str(
                no_rth_after_mission_result));
        return false;
    }

    // Upload the mission plan to the FCC
    const mavsdk::Mission::Result upload_result =
        this->mavsdk_mission->upload_mission(plan);

    if (upload_result != mavsdk::Mission::Result::Success) {
        RCLCPP_ERROR(
            this->get_mavsdk_interface_logger(),
            "Failed to upload mission plan to FCC with result: %s",
            FCCBridgeNode::mavsdk_mission_result_to_str(upload_result));
        return false;
    }

    u8 mission_start_retry_count = 1;

    do {
        const mavsdk::Mission::Result mission_start_result =
            this->mavsdk_mission->start_mission();

        if (mission_start_result == mavsdk::Mission::Result::Success) {
            RCLCPP_INFO(this->get_mavsdk_interface_logger(),
                        "Successfully started mission plan");

            return true;
        } else if (mission_start_result == mavsdk::Mission::Result::Denied) {
            RCLCPP_WARN(
                this->get_mavsdk_interface_logger(),
                "Mission start was denied by FCC try %" PRIu8 " out of %" PRIu8,
                mission_start_retry_count, MAX_MISSION_START_RETRY_COUNT);
        } else {
            RCLCPP_ERROR(this->get_mavsdk_interface_logger(),
                         "Failed to start mission with result: %s",
                         FCCBridgeNode::mavsdk_mission_result_to_str(
                             mission_start_result));
            return false;
        }
        mission_start_retry_count++;
    } while (mission_start_retry_count <= MAX_MISSION_START_RETRY_COUNT);

    RCLCPP_ERROR(this->get_mavsdk_interface_logger(),
                 "Starting mission exceeded maximum amount of retries");
    return false;
}

void FCCBridgeNode::trigger_rth() {
    // In case a mission is already running this store the result of canceling
    // the mission. Unused otherwise
    mavsdk::Mission::Result mission_clear_result;
    switch (this->get_internal_state()) {
        case INTERNAL_STATE::ERROR:
            // This should never happen as the process should have exited before
            throw invalid_state_error(
                "Received command to take of while in ERROR state");
        case INTERNAL_STATE::STARTING_UP:
        case INTERNAL_STATE::ROS_SET_UP:
        case INTERNAL_STATE::MAVSDK_SET_UP:
        case INTERNAL_STATE::WAITING_FOR_ARM:
        case INTERNAL_STATE::ARMED:
        case INTERNAL_STATE::LANDED:
            // Ignore that the UAV is on ground and try an RTH anyway. This is
            // so that if the UAV is unexpectedly airborne the RTH still works
            break;
        case INTERNAL_STATE::RETURN_TO_HOME:
            RCLCPP_WARN(this->get_internal_state_logger(),
                        "Trying to trigger RTH while already returning home");
            return;
        case INTERNAL_STATE::TAKING_OFF:
        case INTERNAL_STATE::FLYING_MISSION:
        case INTERNAL_STATE::LANDING:
            // This means a mission is currently on going. Trying to clear the
            // mission.
            mission_clear_result = this->mavsdk_mission->clear_mission();
            if (mission_clear_result != mavsdk::Mission::Result::Success) {
                RCLCPP_FATAL(this->get_mavsdk_interface_logger(),
                             "Could not clear exiting mission with result: %s! "
                             "Exiting...",
                             FCCBridgeNode::mavsdk_mission_result_to_str(
                                 mission_clear_result));
                this->set_internal_state(INTERNAL_STATE::ERROR);
                this->exit_process_on_error();
            }
            break;
        case INTERNAL_STATE::WAITING_FOR_COMMAND:
            break;
        default:
            throw unknown_enum_value_error(
                std::string("Got invalid value for internal_state: ") +
                std::to_string(static_cast<int>(this->get_internal_state())));
    }

    // Deactivate all subscriber apart from the heartbeat subscriber
    this->control_subscriber.reset();
    this->safety_limits_subscriber.reset();
    this->mission_finished_subscriber.reset();
    this->uav_waypoint_command_subscriber.reset();
    this->uav_command_subscriber.reset();

    RCLCPP_WARN(this->get_mavsdk_interface_logger(), "Triggering RTH");

    // Verify MAVSDK
    this->verify_mavsdk_connection();

    // Deactivate the node to signal mission control that something went wrong
    this->deactivate();

    // Trigger RTH
    if (this->mavsdk_action->return_to_launch() !=
        mavsdk::Action::Result::Success) {
        // In this case something went wrong. Nothing left but to exit.
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Return to launch failed! Exiting...");
        this->set_internal_state(INTERNAL_STATE::ERROR);
        this->exit_process_on_error();
    }

    this->set_internal_state(INTERNAL_STATE::RETURN_TO_HOME);

    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(), "Triggered RTH");
}

void FCCBridgeNode::disarm() {
    RCLCPP_DEBUG(this->get_mavsdk_interface_logger(), "Attempting to disarm");

    if (this->get_internal_state() != INTERNAL_STATE::LANDED) {
        if (this->is_airborne()) {
            // In this case the UAV is airborne
            RCLCPP_ERROR(this->get_internal_state_logger(),
                         "Attempted disarm while in air! Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            // In this case the UAV is on the ground
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "Attempted disarm while on ground! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    const mavsdk::Action::Result res = this->mavsdk_action->disarm();
    if (res != mavsdk::Action::Result::Success) {
        RCLCPP_FATAL(this->get_safety_logger(),
                     "Failed to disarm UAV with result: %s! Exiting...",
                     FCCBridgeNode::mavsdk_action_result_to_str(res));
        this->exit_process_on_error();
    }

    RCLCPP_INFO(this->get_logger(), "Successfully disarmed UAV");
}

void FCCBridgeNode::shutdown_node() {
    // Disable timer
    if (this->shutdown_timer) {
        RCLCPP_INFO(this->get_ros_interface_logger(),
                    "Disabling and deleting shutdown timer");
        this->shutdown_timer->cancel();
        this->shutdown_timer->reset();
    }

    if (this->get_internal_state() != INTERNAL_STATE::LANDED) {
        if (this->is_airborne()) {
            // In this case the UAV is airborne
            RCLCPP_ERROR(this->get_internal_state_logger(),
                         "Attempted shutdown while in air! Triggering RTH...");
            this->trigger_rth();
            return;
        } else {
            // In this case the UAV is on the ground
            RCLCPP_FATAL(this->get_internal_state_logger(),
                         "Attempted shutdown while on ground! Exiting...");
            this->set_internal_state(INTERNAL_STATE::ERROR);
            this->exit_process_on_error();
        }
    }

    RCLCPP_INFO(this->get_ros_interface_logger(), "Shutting down node");
    this->get_node_base_interface()->get_context()->shutdown(
        "Shutdown on FCC land");
}

void FCCBridgeNode::exit_process_on_error() const {
    RCLCPP_WARN(this->get_safety_logger(), "Exiting process with EXIT_FAILURE");

    if (this->get_internal_state() != INTERNAL_STATE::ERROR) {
        RCLCPP_ERROR(this->get_safety_logger(),
                     "Exit was called while internal state was not ERROR but: "
                     "%s! Exiting anyway...",
                     this->internal_state_to_str());
    }

    std::exit(EXIT_FAILURE);
}
}  // namespace fcc_bridge
