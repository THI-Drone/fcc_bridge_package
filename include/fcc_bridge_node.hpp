//
// Created by Johan <job8197@thi.de> on 30.04.2024.
//

#ifndef THI_DRONE_WS_FCC_BRIDGE_NODE_HPP
#define THI_DRONE_WS_FCC_BRIDGE_NODE_HPP

// Libc headers
#include <cstdint>
#include <memory>
#include <optional>

// Mavsdk headers
#include "mavsdk/log_callback.h"
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/mission/mission.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavsdk/system.h"

// rclcpp headers
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

// interfaces headers
#include "interfaces/msg/battery_state.hpp"
#include "interfaces/msg/flight_state.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/mission_finished.hpp"
#include "interfaces/msg/mission_progress.hpp"
#include "interfaces/msg/mission_start.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/rc_state.hpp"
#include "interfaces/msg/safety_limits.hpp"
#include "interfaces/msg/uav_command.hpp"
#include "interfaces/msg/uav_health.hpp"
#include "interfaces/msg/uav_waypoint_command.hpp"

// CommonLib headers
#include "common_package/common_node.hpp"

// Geofence header
#include "geofence.hpp"

/**
 * @brief Holds the fcc_bridge symbols
 */
namespace fcc_bridge {

/***************************************************************************/
/*                       Fixed width integer defines                       */
/***************************************************************************/

// Using directives to allow the reuse of standard types.
using u8 = uint8_t;   /**< Unsigned 8 bit integer */
using u16 = uint16_t; /**< Unsigned 16 bit integer */
using u32 = uint32_t; /**< Unsigned 32 bit integer */
using u64 = uint64_t; /**< Unsigned 64 bit integer */

using s8 = int8_t;   /**< Signed 8 bit integer */
using s16 = int16_t; /**< Signed 16 bit integer */
using s32 = int32_t; /**< Signed 32 bit integer */
using s64 = int64_t; /**< Signed 64 bit integer */

// Explicit Geofence template instantiation
using GeofenceInstace =
    Geofence<double>; /**< Explicit template instantiation with a double */

/**
 * @brief Class that provides the bridge between MAVLink and ROS
 *
 * Contains safety checks and contingencies
 */
class FCCBridgeNode : public common_lib::CommonNode {
    /************************************************************************/
    /*                   Child logger names and accessors                   */
    /************************************************************************/

   private:
    // Logger name
    static constexpr char const *const MAVSDK_INTERNAL_LOGGER_NAME =
        "mavsdk_internal"; /**< Messages from this child logger are internal
                              MAVSDK log messages*/
    static constexpr char const *const MAVSDK_INTERFACE_LOGGER_NAME =
        "mavsdk_interface"; /**< Messages from this child logger are connected
                               to MAVSDK events */
    static constexpr char const *const ROS_INTERFACE_LOGGER_NAME =
        "ros_interface"; /**< Messages from this child logger are connected to
                            ROS events */
    static constexpr char const *const FCC_TELEMETRY_LOGGER_NAME =
        "fcc_telemetry"; /**< Messages from this child logger contain telemetry
                            from the fcc */
    static constexpr char const *const INTERNAL_STATE_LOGGER_NAME =
        "internal_state"; /**< Messages from this child logger are connected to
                             the internal state */
    static constexpr char const *const SAFETY_LOGGER_NAME =
        "safety"; /**< Message from this child logger are connected to safety
                     events */
    static constexpr char const *const COMMAND_HANDLER_LOGGER_NAME =
        "command_handler"; /**< Message from this child logger are connected to
                              command handling events */

    // Accessors
    /**
     * @brief Gets the logger for internal MAVSDK log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_mavsdk_internal_logger() const {
        return this->get_logger().get_child(MAVSDK_INTERNAL_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for MAVSDK related log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_mavsdk_interface_logger() const {
        return this->get_logger().get_child(MAVSDK_INTERFACE_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for ROS related log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_ros_interface_logger() const {
        return this->get_logger().get_child(ROS_INTERFACE_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for FCC telemetry messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_fcc_telemetry_logger() const {
        return this->get_logger().get_child(FCC_TELEMETRY_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for internal state related log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_internal_state_logger() const {
        return this->get_logger().get_child(INTERNAL_STATE_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for safety related log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_safety_logger() const {
        return this->get_logger().get_child(SAFETY_LOGGER_NAME);
    }

    /**
     * @brief Gets the logger for command related log messages
     *
     * @return The rclcpp::Logger
     */
    inline const rclcpp::Logger get_command_handler_logger() const {
        return this->get_logger().get_child(COMMAND_HANDLER_LOGGER_NAME);
    }

    /************************************************************************/
    /*                        Internal state members                        */
    /************************************************************************/

   public:
    // Internal state enum and member to track the current state of the FCC
    // Bridge
    enum INTERNAL_STATE : u8 {
        STARTING_UP = 0, /**< Used initially when the object was just created.
                            Next is the set up of the ROS components. */
        ROS_SET_UP =
            1, /**< Used when the ROS components where successfully set up */
        MAVSDK_SET_UP =
            2, /**< Used when the MAVSDK components where successfully set up.
                  Waiting for safety limits in this state. */
        WAITING_FOR_ARM =
            3,     /**< In this state the node waits for the FCC to be armed */
        ARMED = 4, /**< Waiting for TakeOff from mission control */
        TAKING_OFF = 5, /**< Take off was triggered. Used to have a state where
                           the UAV is still allowerd to be on ground while not
                           accepting other commands */
        WAITING_FOR_COMMAND = 6, /**< Waiting for a command from the waypoint or
                                    mission control node. */
        FLYING_MISSION =
            7, /**< Currently a mission is running, waiting for it to finish */
        LANDING = 8,        /**< Currently a trying to land */
        RETURN_TO_HOME = 9, /**< Returning home. In this state no more commands
                               are accepted. */
        LANDED = 10, /**< The drone has landed and the node will shutdown */

        ERROR =
            0xFF, /**< Error state to signal that an unrecoverable error has
                     occurred and even an RTH is not possible any more. Shorty
                     after this state if taken the process will exit. */
    };            /**< Possible internal states */
    INTERNAL_STATE
    internal_state; /**< Current internal state of the fcc_node */

   protected:
    /**
     * @brief Set the internal state of this node
     *
     * @param new_state The new state to take on.
     *
     * @throws std::runtime_error if new_state if not part of
     * fcc_bridge_FCCBridgeNode::INTERNAL_STATE
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void set_internal_state(const INTERNAL_STATE new_state);
    /**
     * @brief Gets the current internal state of this node
     *
     * @return The current internal state
     *
     * @warning Do not assume this function will stay constexpr!
     *
     * @note Will not trigger an exit if the FCCBridgeNode::internal_state
     * indicates an error
     */
    constexpr INTERNAL_STATE get_internal_state() const {
        return this->internal_state;
    }

   private:
    /*************************************************************************/
    /*                        MAVSDK specific members                        */
    /*************************************************************************/

    std::optional<mavsdk::Mavsdk> mavsdk; /**< The base MAVSDK instance */
    std::shared_ptr<mavsdk::System>
        mavsdk_system; /**< The MAVSDK system representing the connection to the
                          FCC */
    std::optional<mavsdk::Telemetry>
        mavsdk_telemtry; /**< The MAVSDK telemetry accessor */
    std::optional<mavsdk::Action>
        mavsdk_action; /**< The MAVSDK action to trigger simple actions on the
                          FCC */
    std::optional<mavsdk::Mission>
        mavsdk_mission; /**< The MAVSDK mission to do archive more complex tasks
                         */

    /************************************************************************/
    /*                         ROS specific members                         */
    /************************************************************************/

   private:
    // ROS publisher
    rclcpp::Publisher<interfaces::msg::GPSPosition>::SharedPtr
        gps_position_publisher; /**< Publisher to send out GPSPosition updates
                                 */
    rclcpp::Publisher<interfaces::msg::FlightState>::SharedPtr
        flight_state_publisher; /**< Publisher to send out FlightState updates
                                 */
    rclcpp::Publisher<interfaces::msg::BatteryState>::SharedPtr
        battery_state_publisher; /**<  Publisher to send out Battery state
                                    updates */
    rclcpp::Publisher<interfaces::msg::RCState>::SharedPtr
        rc_state_publisher; /**< Publisher to send out RC state updates */
    rclcpp::Publisher<interfaces::msg::Pose>::SharedPtr
        euler_angle_publisher; /**< Publisher to send out euler angle updates*/
    rclcpp::Publisher<interfaces::msg::MissionProgress>::SharedPtr
        mission_progress_publisher; /**< Publisher to send out mission progress
                                       updates */
    rclcpp::Publisher<interfaces::msg::UAVHealth>::SharedPtr
        uav_health_publisher; /**< Publisher to send out uav health updates */
    rclcpp::Publisher<interfaces::msg::MissionStart>::SharedPtr
        mission_start_publisher; /**< Publisher to send out the mission start
                                    message */

    // ROS subscriptions
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr
        mission_control_heartbeat_subscriber; /**< Subscriber for the mission
                                                 control heartbeats */
    rclcpp::Subscription<interfaces::msg::UAVCommand>::SharedPtr
        uav_command_subscriber; /**< Subscriber for uav command messages */
    rclcpp::Subscription<interfaces::msg::UAVWaypointCommand>::SharedPtr
        uav_waypoint_command_subscriber; /**< Subscriber for uav waypoint
                                            command messages */
    rclcpp::Subscription<interfaces::msg::MissionFinished>::SharedPtr
        mission_finished_subscriber; /**< Subscriber for mission finished
                                        messages */
    rclcpp::Subscription<interfaces::msg::SafetyLimits>::SharedPtr
        safety_limits_subscriber; /**< Subscriber for safety limit messages */

    // ROS timer
    rclcpp::TimerBase::SharedPtr
        fcc_telemetry_timer_5hz; /**< Timer for telemetry that is send out at
                                    5Hz */
    rclcpp::TimerBase::SharedPtr
        fcc_telemetry_timer_10hz; /**< Timer for telemetry that is send out at
                                     10Hz */
    rclcpp::TimerBase::SharedPtr
        shutdown_timer; /**< Timer to trigger a callback after the UAV has
                           landed to shutdown the node */

    /**************************************************************************/
    /*                          Cached FCC telemetry                          */
    /**************************************************************************/

   private:
    std::optional<mavsdk::Telemetry::GpsInfo>
        last_fcc_gps_info; /**< The last GPSInfo received from the FCC */
    std::optional<mavsdk::Telemetry::Position>
        last_fcc_position; /**< The last Position received from the FCC */
    std::optional<mavsdk::Telemetry::FlightMode>
        last_fcc_flight_mode; /**< The last FlightMode received from the FCC
                               */
    std::optional<mavsdk::Telemetry::LandedState>
        last_fcc_landed_state; /**< The last LandedState received from the FCC
                                */
    std::optional<bool>
        last_fcc_armed_state; /**< The last armed state received from the FCC */
    std::optional<mavsdk::Telemetry::Battery>
        last_fcc_battery_state; /**< The last received battery state from the
                                   FCC */
    std::optional<mavsdk::Telemetry::RcStatus>
        last_fcc_rc_state; /**< The last received rc state from the FCC */
    std::optional<mavsdk::Telemetry::EulerAngle>
        last_fcc_euler_angle; /**< The last received euler angle from the FCC */
    std::optional<std::pair<mavsdk::Mission::Result, bool>>
        last_mission_progress; /**< The last received mission progress from the
                                  FCC*/
    std::optional<mavsdk::Telemetry::Health>
        last_fcc_health; /**< The last received UAV health from the FCC*/

    /*************************************************************************/
    /*                          Cached ROS messages                          */
    /*************************************************************************/

    std::optional<interfaces::msg::Heartbeat>
        last_mission_control_heartbeat; /**< The last received heartbeat from
                                          mission control */

    /**************************************************************************/
    /*                             Safety members                             */
    /**************************************************************************/

   private:
    // Safety limits
    struct SafetyLimits {
        // Speed limits
        constexpr static float MIN_SPEED_LIMIT_MPS =
            0.f; /**< The minimum speed allowed when flying to a waypoint.
                  Exclusive */
        constexpr static float HARD_MAX_SPEED_LIMIT_MPS =
            5.f; /**< The hard speed limit which will cap the soft speed limit!
                  Inclusive */
        float max_speed_mps;

        // State of charge limits
        constexpr static float HARD_MIN_SOC =
            30.f; /**< The hard minimum state of charge limit in percent. Will
                     cap the soft minimum SoC limit! */

        float min_soc; /**< Minimum state of charge in percent allowed before an
                          RTH is triggered */

        // Height limits
        constexpr static float HARD_MAX_HEIGHT_M =
            50; /**< The hard height limit of the UAV. Will cap the soft height
                   limit! */

        float max_height_m; /**< Max height above the launch position the UAV is
                               allowed to climb to. Inclusive */

        // Geofence limit
        GeofenceInstace geofence; /**< Geofence to be enforced */

        /**
         * @brief Creates a SafetyLimits instance
         *
         * @param max_speed_mps_p The preferred max speed
         * @param min_soc_p The preferred minimum state of charge
         * @param max_height_m_p The preferred max height
         * @param geofence_polygon_p The target geofence polygon
         */
        constexpr SafetyLimits(
            const float max_speed_mps_p, const float min_soc_p,
            const float max_height_m_p,
            const GeofenceInstace::PolygonType &geofence_polygon_p)
            : max_speed_mps(max_speed_mps_p),
              min_soc(min_soc_p),
              max_height_m(max_height_m_p),
              geofence(geofence_polygon_p) {}
    }; /**< struct to hold all currently active safety limits */

    std::optional<struct SafetyLimits>
        safety_limits; /**< Safety limits to be enforced such as geofence and
                          max speed */

    /**************************************************************************/
    /*                            Safety functions                            */
    /**************************************************************************/

   protected:
    /**
     * @brief Checks whether setting the telemetry rate for telemetry_type was
     * successful
     *
     * @param result The result code of the operation
     * @param telemetry_type The type of telemetry whose rate was set
     *
     * @warning Exits the process if result does not indicate a success
     *
     * Implemented in src/fcc_bridge_node_safety.coo
     */
    void check_telemetry_result(const mavsdk::Telemetry::Result &result,
                                char const *const telemetry_type);
    /**
     * @brief Validates that FCCBridgeNode::safety_limits contains safe values.
     *
     * Enforces hard limits
     *
     * If there is something that cannot be corrected set
     * FCCBridgeNode::internal_state to INTERNAL_STATE::ERROR
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void validate_safety_limits();
    /**
     * @brief Checks if the current GPS Fix type is adequate for the current
     * internal state
     *
     * Verifies the GPS fix if the drone is armed and has not yet landed. If no
     * 2DFix is available abort and exit.
     * TODO: Is a 2D fix necessary or can we always assume a 3D fix
     *
     * Exits if no GPS is installed.
     *
     * Checks whether the current UAV position is inside the geofence. Triggers
     * an RTH if airborne and otherwise exits.
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::STARTING_UP,
     * FCCBridgeNode::INTERNAL_STATE::ROS_SET_UP, or
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_gps_state();
    /**
     * @brief Checks if the current flight state is adequate for the current
     * internal state
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_flight_state();
    /**
     * @brief Checks if the cached LandedState is valid in the current
     * internal_state
     *
     * @warning Triggers an exit if the state is invalid and the UAV is on
     * ground
     *
     * @throws std::runtime_error If an unknown enum value is detected
     *
     * @returns true if the state is valid and false if an RTH was triggered
     */
    bool check_landed_state();
    /**
     * @brief Checks if the cached FlightMode is valid in the current
     * internal_state
     *
     * @warning Triggers an exit if the mode is invalid and the UAV is on ground
     *
     * @throws std::runtime_error If an unknown enum value is detected
     *
     * @returns true if the mode is valid and false if an RTH was triggered
     */
    bool check_flight_mode();
    /**
     * @brief Checks if the current battery state is adequate for the current
     * internal state
     *
     * Ensures that the remaining battery percent is not less then the
     * configured minimum state of charge
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_battery_state();
    /**
     * @brief Checks if the current RC state is adequate for the current
     * internal state
     *
     * Ensures that an RC is connected
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_rc_state();
    /**
     * @brief Checks if the current UAV health is adequate for the current
     * internal state
     *
     * @note Will set FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ERROR if
     * 1. is_gyrometer_calibration_ok && is_accelerometer_calibration_ok &&
     * is_magnetometer_calibration_ok is false or...
     * 2. If the UAV is airborne && is_local_position_ok &&
     * is_global_position_ok && is_home_position_ok is false
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::STARTING_UP,
     * FCCBridgeNode::INTERNAL_STATE::ROS_SET_UP, or
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_uav_health();
    /**
     * @brief Checks whether a given point is inside the geofence stored inside
     * this node
     *
     * @param latitude_deg The latitude of the point in degrees
     * @param longitude_deg The longitude of the point in degrees
     * @param relative_altitude_m The relative altitude above the take off point
     * of the point in meters
     *
     * @returns geofence was configured && point is inside geofence
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * @note Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ERROR if no geofence was configured
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    bool check_point_in_geofence(const double latitude_deg,
                                 const double longitude_deg,
                                 const float relative_altitude_m);
    /**
     * @brief Checks whether a given point is inside the geofence stored inside
     * this node
     *
     * @param waypoint The waypoint to check
     *
     * @returns geofence was configured && point is inside geofence
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * @note Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ERROR if no geofence was configured
     */
    inline bool check_point_in_geofence(
        const interfaces::msg::Waypoint &waypoint) {
        return this->check_point_in_geofence(waypoint.latitude_deg,
                                             waypoint.longitude_deg,
                                             waypoint.relative_altitude_m);
    }
    /**
     * @brief Checks whether a given point is inside the geofence stored inside
     * this node
     *
     * @param position The position to check
     *
     * @returns geofence was configured && point is inside geofence
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * @note Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ERROR if no geofence was configured
     */
    inline bool check_point_in_geofence(
        const mavsdk::Telemetry::Position &position) {
        return this->check_point_in_geofence(position.latitude_deg,
                                             position.longitude_deg,
                                             position.relative_altitude_m);
    }
    /**
     * @brief Checks if the waypoints altitude is valid
     *
     * @param relative_altitude The relative altitude of the waypoint
     *
     * @return True if the waypoint is invalid
     */
    constexpr static bool check_waypoint_invalid(
        const float relative_altitude) {
        return 0.f >= std::abs(relative_altitude -
                               interfaces::msg::Waypoint::INVALID_ALTITUDE);
    }
    /**
     * @brief Checks if the waypoints is valid
     *
     * @param waypoint The waypoint to check
     *
     * @return True if the waypoint is invalid
     */
    constexpr static bool check_waypoint_invalid(
        const interfaces::msg::Waypoint &waypoint) {
        return FCCBridgeNode::check_waypoint_invalid(
            waypoint.relative_altitude_m);
    }
    /**
     * @brief Checks whether the given speed is inside the valid range
     * (0;MAX_SPEED]
     *
     * @param speed_mps The speed to check
     *
     * @returns MAX_SPEED was configured && point is inside geofence
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * @note Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ERROR if no MAX_SPEED was configured
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    bool check_speed(const float speed_mps);
    /**
     * @brief Validated if the last received heartbeat is not too old.
     *
     * If it is too old trigger an RTH
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void check_last_mission_control_heartbeat();

    /**************************************************************************/
    /*                         ROS specific functions                         */
    /**************************************************************************/

   protected:
    /**
     * @brief Set up ROS specific functionality
     *
     * Sets internal_state to INTERNAL_STATE::ERROR if an error is encountered
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void setup_ros();
    /**
     * @brief Callback function to be triggered when a new heartbeat is received
     *
     * @param msg The received message
     *
     * @note Should only receive heartbeats from mission control using a filter
     * expression when creating the subscription.
     *
     * Triggers an RTH if the heartbeat is invalid or goes into error state if
     * MAVSDK has run into an issue.
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void mission_control_heartbeat_subscriber_cb(
        const interfaces::msg::Heartbeat &msg);
    /**
     * @brief Callback function to be triggered when a new UAVCommand message is
     * received
     *
     * @param msg The received message
     *
     * Will trigger all the necessary safety checks to ensure only safe commands
     * are actually executed
     *
     * Triggers an RTH if the command is deemed invalid such as when
     * 1. The time stamp in the message is older than one second.
     * 2. sender_id is not the active node
     * TODO: Work out
     *
     * @throws std::runtime_error If FCCBridgeNode::internal_state is
     * FCCBridgeNode::INTERNAL_STATE::ERROR
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void uav_command_subscriber_cb(const interfaces::msg::UAVCommand &msg);
    /**
     * @brief Callback function to be triggered when a new UAVWaypointCommand
     * message is received
     *
     * @param msg The received message
     *
     * Repackages the message as an interfaces::msg::UAVCommand and calls
     * FCCBridge::uav_command_subscriber_cb
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void uav_waypoint_command_subscriber_cb(
        const interfaces::msg::UAVWaypointCommand &msg);
    /**
     * @brief Callback function to be triggered when a new MissionFinished
     * message is received
     *
     * @param msg The received message
     *
     * - If the sender is not mission_control this will trigger an RTH
     * - If mission control is not the active node this will trigger an RTH
     * - If the UAV has not yet taken off this will trigger an exit.
     * - If the UAV is airborne this message will trigger an RTH.
     * - If the UAV is on ground and error_code is != 0 this message will
     * trigger an exit.
     * - Otherwise the Node will be stopped by a timer after 5 seconds
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void mission_finished_cb(const interfaces::msg::MissionFinished &msg);
    /**
     * @brief Callback function to be triggered when a new SafetyLimits message
     * is received
     *
     * @param msg The received message
     *
     * Stores the received safety limits in FCCBridgeNode::safety_limits and
     * enforces the internal hard limits
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void safety_limits_cb(const interfaces::msg::SafetyLimits &msg);
    /**
     * @brief Callback function for the 5Hz telemetry timer
     *
     * Validates heartbeat.
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void fcc_telemetry_timer_5hz_cb();
    /**
     * @brief Callback function for the 10Hz telemetry timer
     *
     * Validates heartbeat
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    void fcc_telemetry_timer_10hz_cb();

    /*************************************************************************/
    /*                          Telemetry functions                          */
    /*************************************************************************/

   protected:
    /**
     * @brief Gets the GPS telemetry from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Verifies the GPS state using FCCBridgeNode::check_gps_state
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_gps_telemetry();
    /**
     * @brief Gets the flight state from the FCC and publishes it on the ROS
     * network
     *
     * Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::ARMED if the fcc_bridge is waiting for the
     * UAV to be armed and the flight state indicates that the UAV is armed.
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Verifies the flight state using FCCBridgeNode::check_flight_state
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_flight_state();
    /**
     * @brief Gets the battery state from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Verifies the battery state using FCCBridgeNode::check:_battery_state
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_battery_state();
    /**
     * @brief Gets the RC state from the FCC and publishes it on the ROS network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Verifies the RC state using FCCBridgeNode::check_rc_state
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_rc_state();
    /**
     * @brief Gets the euler angle from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_euler_angle();
    /**
     * @brief Gets the mission progress from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * @warning Will trigger an RTH if there is any issue with the mission
     * including no currently running mission
     *
     * Will set FCCBridgeNode::internal_state to WAITING_FOR_COMMAND or LANDED
     * depending on the current state and whether the mission has finished.
     *
     * If the drone has landed trigger a timer that will stop the node after 1
     * minute.
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_mission_progress();
    /**
     * @brief Gets the UAV health form the FCC and publishes on the ROS network
     *
     * @warning Does not check he validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * Verifies the UAV health using FCCBridgeNode::check_uav_health
     *
     * Implemented in src/fcc_bridge_node_telemetry.cpp
     */
    void send_uav_health();

    /*************************************************************************/
    /*                           Command functions                           */
    /*************************************************************************/

   protected:
    /**
     * @brief This function starts a mission that will perform a take off and
     * fly to the specified waypoint
     *
     * @param waypoint The waypoint to fly to
     *
     * @param speed_mps The speed to use while flying
     *
     * @note Verifies the that waypoint is inside the geofence
     *
     * Implemented in src/fcc_bridge_node_command.cpp
     */
    void initiate_takeoff(const interfaces::msg::Waypoint &waypoint,
                          const float speed_mps);
    /**
     * @brief This function starts a mission that will fly to the passed
     * waypoint
     *
     * @param waypoint The waypoint to fly to
     *
     * @param speed_mps The speed to use while flying
     *
     * @note Verifies the that waypoint is inside the geofence
     *
     * Implemented in src/fcc_bridge_node_command.cpp
     */
    void start_flying_to_waypoint(const interfaces::msg::Waypoint &waypoint,
                                  const float speed_mps);
    /**
     * @brief This function start a mission that will fly to the passed waypoint
     * and then land
     *
     * @param waypoint The waypoint to fly to
     *
     * @param speed_mps The speed to use while flying
     *
     * @note Verifies the that waypoint is inside the geofence
     *
     * Implemented in src/fcc_bridge_node_command.cpp
     */
    void initiate_land(const interfaces::msg::Waypoint &waypoint,
                       const float speed_mps);
    /**
     * @brief This function initiates a return to home
     *
     * @note This is a function that could allow for a more graceful RTH in the
     * future. Right now it will just call FCCBridge::trigger_rth
     *
     * Implemented in src/fcc_bridge_node_command.cpp
     */
    void initiate_rth();

    /*************************************************************************/
    /*                       MAVSDK specific functions                       */
    /*************************************************************************/

   private:
    /**
     * @brief Callback function to be triggered when MAVSDK wants to log
     * something
     *
     * @param level The target log level
     * @param message The message to log
     * @param file The source file of the log
     * @param line The line in the source file where the log originated from
     *
     * @returns If the log message should still be printed to stdout. Always
     * true
     *
     * Converts a MAVSDK log message to a ROS log message
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    bool mavsdk_log_callback(const mavsdk::log::Level level,
                             const std::string &message,
                             const std::string &file, const int line);

   protected:
    /**
     * @brief Sets up the MAVSDK components.
     *
     * Uses the ros parameter UAV_ID to identify the target UAV.
     * Set internal_state to INTERNAL_STATE::ERROR if an issue is encountered.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void setup_mavsdk();
    /**
     * @brief Verifies the MAVSDK connection.
     *
     * If there is an issue this function will exit the process.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void verify_mavsdk_connection();
    /**
     * @brief Gets the current GPSInfo and Position from the FCC
     *
     * Stores the result in internal member variables
     * FCCBridgeNode::last_fcc_gps_info and @FCCBridgeNode::last_fcc_position
     *
     * Verifies the MAVSDK connection.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_gps_telemetry();
    /**
     * @brief Gets the current FlightState from the FCC
     *
     * Stores the result in the internal member variables
     * FCCBridgeNode::last_fcc_flight_mode and
     * FCCBridgeNode::last_fcc_landed_state
     * FCCBridgeNode::last_fcc_armed_state
     *
     * Verifies the MAVSDK connection.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_flight_state();
    /**
     * @brief Gets the current Battery state from the FCC
     *
     * Stores the result in the internal member variable
     * FCCBridgeNode::last_fcc_battery_state
     *
     * Verifies the MAVSDK connection.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_battery_state();
    /**
     * @brief Gets the current remote control from the FCC
     *
     * Stores the result in the internal member variable
     * FCCBridgeNode::last_fcc_rc_state
     *
     * Verifies the MAVSDK connection.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_rc_state();
    /**
     * @brief Gets the current euler angle from the FCC
     *
     * Stores the result in the internal member variable
     * FCCBridgeNode::last_fcc_euler_angle
     *
     * Verifies the MAVSDK connection
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_euler_angle();
    /**
     * @brief Gets the current mission progress from the FCC
     *
     * Stores the result in the internal member variable
     * FCCBridgeNode::last_fcc_euler_angle
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     *
     * Verifies the MAVSDK connection
     */
    void get_mission_progress();
    /**
     * @brief Gets the current UAV health from the FCC
     *
     * Stores the result in the internal member variable
     * FCCBridgeNode::last_fcc_health
     *
     * Verifies the MAVSDK connection
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void get_uav_health();
    /**
     * @brief Executes the passed mission plan asynchronously
     *
     * @param plan The plan to execute
     *
     * @return WHether executing the mission was successful
     *
     * Verifies the MAVSDK connection
     *
     * @warning Does not perform any safety checks on the plan!
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    bool execute_mission_plan(const mavsdk::Mission::MissionPlan &plan);
    /**
     * @brief Initiates an RTH
     *
     * Deactivates the node
     *
     * @note Guarantees that FCCBridgeNode::internal_state is set to
     * FCCBridgeNode::INTERNAL_STATE::RETURN_TO_HOME if this function returns.
     *
     * @note This function is meant for safety use. To trigger a regular return
     * to home use FCCBridgeNode::initiate_rth
     *
     * @warning This function returns. It is the callers responsibility to
     * cancel his own operation.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void trigger_rth();
    /**
     * @brief Attempts to disarm the UAV
     *
     * Rejects the disarm if the UAV is airborne and will trigger an RTH
     *
     * @warning If the UAV has not yet taken off will exit the process
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void disarm();
    /**
     * @brief Forcefully shuts down the node after the UAV has landed
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void force_shutdown_node();
    /**
     * @brief Normally shuts down the node after the UAV has landed
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    void normal_shutdown_node();
    /**
     * @brief Exit the current process.
     *
     * Does not return.
     * Wrapper for test cases to check for error conditions without death tests.
     *
     * Implemented in src/fcc_bridge_node_mavsdk.cpp
     */
    [[noreturn]] void exit_process_on_error() const;

    /**************************************************************************/
    /*               MAVSDK <=> ROS "enum" conversion functions               */
    /**************************************************************************/

   public:
    /**
     * @brief Conversion function to turn a MAVSDK Gps FixType into a ROS GPS
     * FixType
     *
     * @param fix_type The MAVSDK FixType to convert
     * @return The ROS FixType
     *
     * @throws std::runtime_error If the MAVSDK FixType is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static interfaces::msg::GPSPosition::_fix_type_type fix_type_mavsdk_to_ros(
        const mavsdk::Telemetry::FixType &fix_type);
    /**
     * @brief Conversion function to turn a MAVSDK FlightMode into a ROS
     * FlightMode
     *
     * @param flight_mode The MAVSDK FlightMode to convert
     * @return The ROS FlightMode
     *
     * @throws std::runtime_error If the MAVSDK FlightMode is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static interfaces::msg::FlightState::_mode_type::_mode_type
    flight_mode_mavsdk_to_ros(const mavsdk::Telemetry::FlightMode &flight_mode);
    /**
     * @brief Conversion function to turn a MAVSDK LandedState into a ROS
     * LandedState
     *
     * @param flight_mode The MAVSDK LandedState to convert
     * @return The ROS LandedState
     *
     * @throws std::runtime_error If the MAVSDK LandedState is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static interfaces::msg::FlightState::_state_type::_state_type
    landed_state_mavsdk_to_ros(
        const mavsdk::Telemetry::LandedState &landed_state);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::ConnectionResult
     *
     * @param result The result code to convert
     *
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_connection_result_to_str(
        const mavsdk::ConnectionResult &result);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Telemetry::FlightMode
     *
     * @param result The flight mode to convert
     *
     * @return The string representation of the flight mode
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_flight_mode_to_str(
        const mavsdk::Telemetry::FlightMode &flight_mode);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Telemetry::LandedState
     *
     * @param landed_state The landed state to convert
     *
     * @return The string representation of the flight mode
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_landed_state_to_str(
        const mavsdk::Telemetry::LandedState &landed_state);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Mission::Result
     *
     * @param result The result code to convert
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_mission_result_to_str(
        const mavsdk::Mission::Result &result);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Action::Result
     *
     * @param result The result code to convert
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_action_result_to_str(
        const mavsdk::Action::Result &result);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Telemetry::FixType
     *
     * @param result The FixType to convert
     * @return The string representation of the FixType
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_fix_type_to_str(
        const mavsdk::Telemetry::FixType &fix_type);
    /**
     * @brief Conversion function to get the string representation of a
     * mavsdk::Telemetry::Result
     *
     * @param result The Result to convert
     * @return The string representation of the Result
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    static char const *mavsdk_telemetry_result_to_str(
        const mavsdk::Telemetry::Result &result);
    /**
     * @brief Conversion function to get the string representation of the
     * current value of FCCBridgeNode::internal_state
     *
     * @return The string representation of the internal state
     *
     * @throws std::runtime_error If the value is unknown
     *
     * Implemented in src/fcc_bridge_node_conversion.cpp
     */
    char const *internal_state_to_str() const;

    /**************************************************************************/
    /*                              Constructors                              */
    /**************************************************************************/

   public:
    /**
     * @brief Constructor for an FCCBridgeNode
     *
     * @param name The name of the node
     * @param node_options The node options, default to rclcpp::NodeOptions
     *
     * Sets up the ROS components, then the MAVSDK components and then activates
     * the node to signal it is ready for the SafetyLimits.
     * Exits the process on error.
     *
     * Implemented in src/fcc_bridge_node_ros.cpp
     */
    FCCBridgeNode(
        const std::string &name,
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
};

}  // namespace fcc_bridge

#endif  // THI_DRONE_WS_FCC_BRIDGE_NODE_HPP
