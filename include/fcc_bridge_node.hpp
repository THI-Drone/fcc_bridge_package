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
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/mission/mission.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavsdk/system.h"

// rclcpp headers
#include "rclcpp/node_options.hpp"
#include "rclcpp/timer.hpp"

// interfaces headers
#include "interfaces/msg/battery_state.hpp"
#include "interfaces/msg/flight_state.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/mission_progress.hpp"
#include "interfaces/msg/mission_start.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/rc_state.hpp"
#include "interfaces/msg/uav_command.hpp"
#include "interfaces/msg/uav_health.hpp"

// CommonLib headers
#include "common_package/common_node.hpp"

/**
 * @brief Holds the fcc_bridge symbols
 */
namespace fcc_bridge {

/**************************************************************************/
/*                       Fixed with integer defines                       */
/**************************************************************************/

// Using directives to allow the reuse of standard types.
using u8 = uint8_t;   /**< Unsigned 8 bit integer */
using u16 = uint16_t; /**< Unsigned 16 bit integer */
using u32 = uint32_t; /**< Unsigned 32 bit integer */
using u64 = uint64_t; /**< Unsigned 64 bit integer */

using s8 = int8_t;   /**< Signed 8 bit integer */
using s16 = int16_t; /**< Signed 16 bit integer */
using s32 = int32_t; /**< Signed 32 bit integer */
using s64 = int64_t; /**< Signed 64 bit integer */

/**
 * @brief Class that provides the bridge between MAVLink and ROS
 *
 * Contains safety checks and contingencies
 */
class FCCBridgeNode : public common_lib::CommonNode {
    /************************************************************************/
    /*                        Internal state members                        */
    /************************************************************************/

   private:
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
        WAITING_FOR_COMMAND = 5, /**< Waiting for a command from the waypoint or
                                    mission control node. */
        FLYING_ACTION =
            6, /**< Currently an action is running, waiting for it to finish */
        FLYING_MISSION =
            7, /**< Currently a mission is running, waiting for it to finish */
        RETURN_TO_HOME = 8, /**< Returning home. In this state no more commands
                               are accepted. */
        LANDED = 9, /**< The drone has landed and the node will shutdown */

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
     * @throws std::runtime_error if new_state if not part of @ref
     * fcc_bridge_FCCBridgeNode::INTERNAL_STATE
     */
    void set_internal_state(const INTERNAL_STATE new_state);
    /**
     * @brief Gets the current internal state of this node
     *
     * @return The current internal state
     *
     * @warning Do not assume this function will stay constexpr!
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

    // ROS timer
    rclcpp::TimerBase::SharedPtr
        fcc_telemetry_timer_5hz; /**< Timer for telemetry that is send out at
                                    5Hz */
    rclcpp::TimerBase::SharedPtr
        fcc_telemetry_timer_10hz; /**< Timer for telemetry that is send out at
                                     10Hz */

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

    interfaces::msg::Heartbeat
        last_mission_control_heartbeat; /**< The last received heartbeat from
                                          mission control */

    /**************************************************************************/
    /*                             Safety members                             */
    /**************************************************************************/

   private:
    // Safety limits
    struct safety_limits {
        // TODO: Implement
        constexpr static float MIN_SPEED_LIMIT_MPS =
            0; /**< The minimum speed allow when flying to a waypoint. Exclusive
                */
        constexpr static float HARD_MAX_SPEED_LIMIT_MPS =
            5; /**< The hard speed limit which will cap the soft speed limit.
                  Inclusive */
        float max_speed_mps;
    }; /**< struct to hold all currently active safety limits */

    std::optional<struct safety_limits>
        safety_limits; /**< Safety limits to be enforced such as geofence and
                          max speed */

    /**************************************************************************/
    /*                            Safety functions                            */
    /**************************************************************************/

   private:
    /**
     * @brief Callback function to be triggered when an asynchronous RTH mavsdk
     * action finished
     *
     * @param result The result of the RTH
     *
     * @note Will exit the process if the result code does not indicate success
     *
     * Sets FCCBridgeNode::internal_state to
     * FCCBridgeNode::INTERNAL_STATE::LANDED if result indicated success
     *
     * Implemented in src/fcc_bridge_node_safety.cpp
     */
    void mavsdk_rth_cb(const mavsdk::Action::Result &result);

   protected:
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
     * TODO: Geofence check
     *
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is STARTING_UP, ROS_SET_UP, or
     * ERROR
     */
    void check_gps_state();
    /**
     * @brief Checks if the current flight state is adequate for the current
     * internal state
     *
     * TODO: Currently just an empty function
     */
    void check_flight_state();
    /**
     * @brief Checks if the current battery state is adequate for the current
     * internal state
     *
     * TODO: Currently just an empty function
     */
    void check_battery_state();
    /**
     * @brief Checks if the current RC state is adequate for the current
     * internal state
     *
     * TODO: Currently just an empty function
     */
    void check_rc_state();
    /**
     * @brief Checks if the current UAV health is adequate for the current
     * internal state
     *
     * @note Will set @ref fcc_bridge::FCCBridgeNode::internal_state to @ref
     * fcc_bridge::FCCBridgeNode::INTERNAL_STATE::ERROR if
     * 1. is_gyrometer_calibration_ok && is_accelerometer_calibration_ok &&
     * is_magnetometer_calibration_ok is false or...
     * 2. If the UAV is airborne && is_local_position_ok &&
     * is_global_position_ok && is_home_position_ok is false
     *
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is STARTING_UP, ROS_SET_UP, or
     * ERROR
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
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is ERROR
     *
     * @note Sets @ref fcc_bridge::FCCBridgeNode::internal_state to ERROR if no
     * geofence was configured
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
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is ERROR
     *
     * @note Sets @ref fcc_bridge::FCCBridgeNode::internal_state to ERROR if no
     * geofence was configured
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
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is ERROR
     *
     * @note Sets @ref fcc_bridge::FCCBridgeNode::internal_state to ERROR if no
     * geofence was configured
     */
    inline bool check_point_in_geofence(
        const mavsdk::Telemetry::Position &position) {
        return this->check_point_in_geofence(position.latitude_deg,
                                             position.longitude_deg,
                                             position.relative_altitude_m);
    }
    /**
     * @brief Checks whether the given speed is inside the valid range
     * (0;MAX_SPEED]
     *
     * @param speed_mps The speed to check
     *
     * @returns MAX_SPEED was configured && point is inside geofence
     *
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is ERROR
     *
     * @note Sets @ref fcc_bridge::FCCBridgeNode::internal_state to ERROR if no
     * MAX_SPEED was configured
     */
    bool check_speed(const float speed_mps);

    /**************************************************************************/
    /*                         ROS specific functions                         */
    /**************************************************************************/

   protected:
    /**
     * @brief Set up ROS specific functionality
     *
     * Sets internal_state to INTERNAL_STATE::ERROR if an error is encountered
     */
    void setup_ros();
    /**
     * @brief Callback function to be triggered when a new heartbeat is received
     * @param msg The received message
     *
     * @note Should only receive heartbeats from mission control using a filter
     * expression when creating the subscription.
     *
     * Triggers an RTH if the heartbeat is invalid or goes into error state if
     * MAVSDK has run into an issue.
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
     * 2.
     *
     * @throws std::runtime_error If @ref
     * fcc_bridge::FCCBridgeNode::internal_state is @ref
     * fcc_bridge::FCCBridgeNode::INTERNAL_STATE::ERROR
     */
    void uav_command_subscriber_cb(const interfaces::msg::UAVCommand &msg);
    /**
     * @brief This function starts a mission that will perform a take off and
     * fly to the specified waypoint
     *
     * @note Verifies the that waypoint is inside the geofence
     * TODO: Waypoint check
     * TODO: how to signal back if waypoint is ok???
     */
    void initiate_takeoff(const interfaces::msg::Waypoint &waypoint,
                          const float speed_mps);
    /**
     * @brief This function starts a mission that will fly to the passed
     * waypoint
     *
     * @note Verifies the that waypoint is inside the geofence
     * TODO: Waypoint check
     */
    void start_flying_to_waypoint(const interfaces::msg::Waypoint &waypoint,
                                  const float speed_mps);
    /**
     * @brief Callback function for the 5Hz telemetry timer
     *
     * Validates heartbeat.
     */
    void fcc_telemetry_timer_5hz_cb();
    /**
     * @brief Callback function for the 5Hz telemetry timer
     *
     * Validates heartbeat
     */
    void fcc_telemetry_timer_10hz_cb();
    /**
     * @brief Validated if the last received heartbeat is not too old.
     *
     * If it is too old trigger an RTH
     */
    void check_last_mission_control_heartbeat();
    /**
     * @brief Gets the GPS telemetry from the FCC and publishes it on the ROS
     * network
     *
     * Verifies the GPS state using @ref
     * fcc_bridge::FCCBridgeNode::check_gps_state
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_gps_telemetry();
    /**
     * @brief Gets the flight state from the FCC and publishes it on the ROS
     * network
     *
     * Sets @ref fcc_bridge::FCCBridgeNode::internal_state to @ref
     * fcc_bridge::FCCBridgeNode::INTERNAL_STATE::ARMED if the fcc_bridge is
     * waiting for the UAV to be armed and the flight state indicates that the
     * UAV is armed.
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_flight_state();
    /**
     * @brief Gets the battery state from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_battery_state();
    /**
     * @brief Gets the RC state from the FCC and publishes it on the ROS network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_rc_state();
    /**
     * @brief Gets the euler angle from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check the validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_euler_angle();
    /**
     * @brief Gets the mission progress from the FCC and publishes it on the ROS
     * network
     *
     * @warning Does not check he validity of the last heartbeat. That is the
     * calling functions responsibility
     *
     * @warning Will trigger an RTH if there is any issue with the mission
     * including no currently running mission
     */
    void send_mission_progress();
    /**
     * @brief Gets the UAV health form the FCC and publishes on the ROS network
     *
     * Verifies the GPS state using @ref
     * fcc_bridge::FCCBridgeNode::check_uav_health
     *
     * @warning Does not check he validity of the last heartbeat. That is the
     * calling functions responsibility
     */
    void send_uav_health();

    /*************************************************************************/
    /*                       MAVSDK specific functions                       */
    /*************************************************************************/

   protected:
    /**
     * @brief Sets up the MAVSDK components.
     *
     * Uses the ros parameter UAV_ID to identify the target UAV.
     * Set internal_state to INTERNAL_STATE::ERROR if an issue is encountered.
     */
    void setup_mavsdk();
    /**
     * @brief Verifies the MAVSDK connection.
     *
     * If there is an issue this function will exit the process.
     */
    void verify_mavsdk_connection();
    /**
     * @brief Gets the current GPSInfo and Position from the FCC
     *
     * Stores the result in internal member variables @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_gps_info and @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_position
     *
     * Verifies the MAVSDK connection.
     */
    void get_gps_telemetry();
    /**
     * @brief Gets the current FlightState from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_flight_state
     *
     * Verifies the MAVSDK connection.
     * @note MAVSDK calls this information FlightMode
     */
    void get_flight_state();
    /**
     * @brief Gets the current Battery state from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_battery_state
     *
     * Verifies the MAVSDK connection.
     */
    void get_battery_state();
    /**
     * @brief Gets the current remote control from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_rc_state
     *
     * Verifies the MAVSDK connection.
     */
    void get_rc_state();
    /**
     * @brief Gets the current euler angle from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_euler_angle
     *
     * Verifies the MAVSDK connection
     */
    void get_euler_angle();
    /**
     * @brief Gets the current mission progress from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_euler_angle
     *
     * Verifies the MAVSDK connection
     */
    void get_mission_progress();
    /**
     * @brief Gets the current UAV health from the FCC
     *
     * Stores the result in the internal member variable @ref
     * fcc_bridge::FCCBridgeNode::last_fcc_health
     *
     * Verifies the MAVSDK connection
     */
    void get_uav_health();
    /**
     * @brief Executes the passed mission plan asynchronously
     *
     * @param plan The plan to execute
     *
     * Verifies the MAVSDK connection
     *
     * @warning Does not perform any safety checks on the plan!
     */
    bool execute_mission_plan(const mavsdk::Mission::MissionPlan &plan);
    /**
     * @brief Initiates an RTH
     *
     * Deactivates the node
     *
     * @note Guarantees that @ref fcc_bridge::FCCBridgeNode::internal_state is
     * set to @ref fcc_bridge::FCCBridgeNode::INTERNAL_STATE::RETURN_TO_HOME if
     * this function returns.
     *
     * @warning This function returns. It is the callers responsibility to
     * cancel his own operation.
     */
    void trigger_rth();
    /**
     * @brief Exit the current process.
     *
     * Does not return.
     * Wrapper for test cases to check for error conditions without death tests.
     */
    [[noreturn]] void exit_process_on_error();

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
     */
    static interfaces::msg::FlightState::_state_type::_state_type
    landed_state_mavsdk_to_ros(
        const mavsdk::Telemetry::LandedState &landed_state);
    /**
     * @brief Conversion function to get the string representation of a @ref
     * mavsdk::ConnectionResult
     *
     * @param result The result code to convert
     *
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     */
    static char const *mavsdk_connection_result_to_str(
        const mavsdk::ConnectionResult &result);
    /**
     * @brief Conversion function to get the string representation of a @ref
     * mavsdk::Telemetry::FlightMode
     *
     * @param result The flight mode to convert
     *
     * @return The string representation of the flight mode
     *
     * @throws std::runtime_error If the value is unknown
     */
    static char const *mavsdk_flight_mode_to_str(
        const mavsdk::Telemetry::FlightMode &flight_mode);
    /**
     * @brief Conversion function to get the string representation of a @ref
     * mavsdk::Telemetry::LandedState
     *
     * @param landed_state The landed state to convert
     *
     * @return The string representation of the flight mode
     *
     * @throws std::runtime_error If the value is unknown
     */
    static char const *mavsdk_landed_state_to_str(
        const mavsdk::Telemetry::LandedState &landed_state);
    /**
     * @brief Conversion function to get the string representation of a @ref
     * mavsdk::Mission::Result
     *
     * @param result The result code to convert
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     */
    static char const *mavsdk_mission_result_to_str(
        const mavsdk::Mission::Result &result);
    /**
     * @brief Conversion function to get the string representation of a @ref
     * mavsdk::Action::Result
     *
     * @param result The result code to convert
     * @return The string representation of the result
     *
     * @throws std::runtime_error If the value is unknown
     */
    static char const *mavsdk_action_result_to_str(
        const mavsdk::Action::Result &result);
    /**
     * @brief Conversion function to get the string representation of the
     * current value of @ref fcc_bridge::FCCBridgeNode::internal_state
     *
     * @return The string representation of the internal state
     *
     * @throws std::runtime_error If the value is unknown
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
     */
    FCCBridgeNode(
        const std::string &name/*,
        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()*/);
};

}  // namespace fcc_bridge

#endif  // THI_DRONE_WS_FCC_BRIDGE_NODE_HPP
