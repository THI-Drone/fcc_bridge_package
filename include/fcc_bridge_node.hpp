//
// Created by Johan on 30.04.2024.
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
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/heartbeat.hpp"

// CommonLib headers
#include "common_package/common_node.hpp"

namespace fcc_bridge {

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

class FCCBridgeNode : public common_lib::CommonNode {
   private:
    // Possible internal states
    enum INTERNAL_STATE : u8 {
        STARTING_UP = 0,
        ROS_SET_UP = 1,
        MAVSDK_SET_UP = 2,
        WAITING_FOR_ARM = 3,
        ARMED = 4,
        WAITING_FOR_COMMAND = 5,
        FLYING_ACTION = 6,
        FLYING_MISSION = 7,
        RETURN_TO_HOME = 8,
        LANDED = 9,

        ERROR = 0xFF,
    };
    INTERNAL_STATE internal_state;  ///> Current internal state of the fcc_node

    // MAVSDK objects
    std::optional<mavsdk::Mavsdk> mavsdk;
    std::shared_ptr<mavsdk::System> mavsdk_system;
    std::optional<mavsdk::Telemetry> mavsdk_telemtry;
    std::optional<mavsdk::Action> mavsdk_action;
    std::optional<mavsdk::Mission> mavsdk_mission;

    // ROS publisher
    rclcpp::Publisher<interfaces::msg::GPSPosition>::SharedPtr
        gps_position_publisher;

    // ROS subscriptions
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr
        mission_control_heartbeat_subscriber;

    // ROS timer
    rclcpp::TimerBase::SharedPtr fcc_telemetry_timer_5hz;
    rclcpp::TimerBase::SharedPtr fcc_telemetry_timer_10hz;

    // Cached FCC Telemetry
    std::optional<mavsdk::Telemetry::GpsInfo> last_fcc_gps_info;
    std::optional<mavsdk::Telemetry::Position> last_fcc_position;

    // Last heartbeat from mission control
    interfaces::msg::Heartbeat last_mission_control_heatbeat;

   protected:
    // Safety functions
    // bool check_point_in_geofence();

    // ROS functions
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
     * Should only receive heartbeats from mission control using a filter
     * expression when creating the subscription.
     * Triggers an RTH if the heartbeat is invalid or goes into error state if
     * MAVSDK has run into an issue.
     */
    void mission_control_heartbeat_subscriber_cb(
        const interfaces::msg::Heartbeat &msg);
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
    void check_last_mission_control_heatbeat();

    // MAVSDK functions
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
    void verify_connection();
    /**
     * @brief Gets the current GPSInfo and Position from the FCC
     *
     * Stores the result in internal member variables.
     * Verifies the MAVSDK connection.
     */
    void get_gps_telemetry();
    /**
     * @brief Initiates an RTH
     *
     * Deactivates the node
     */
    void trigger_rth();

    // Enum conversion functions
    /**
     * @brief Conversion function to turn a MAVSDK Gps FixType into a ROS GPS
     * FixType
     *
     * @param fix_type The MAVSDK FixType to convert
     * @return The ROS FixType
     *
     * @throws RuntimeError If the MAVSDK FixType is unknown
     */
    static interfaces::msg::GPSPosition::_fix_type_type fix_type_mavsdk_to_ros(
        const mavsdk::Telemetry::FixType &fix_type);

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
