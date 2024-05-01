//
// Created by Johan on 30.04.2024.
//

#ifndef THI_DRONE_WS_FCC_BRIDGE_NODE_HPP
#define THI_DRONE_WS_FCC_BRIDGE_NODE_HPP

// Libc headers
#include <cstdint>
#include <optional>

// Mavsdk headers
#include "mavsdk/mavsdk.h"
#include "mavsdk/plugins/action/action.h"
#include "mavsdk/plugins/mission/mission.h"
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavsdk/system.h"

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
    INTERNAL_STATE internal_state;
    mavsdk::Mavsdk mavsdk;
    mavsdk::System mavsdk_system;
    mavsdk::Telemetry telemtry;
    std::optional<mavsdk::Action> mavsdk_action;
    std::optional<mavsdk::Mission> mavsdk_mission;
   protected:
    void setup_ros();
    void setup_mavsdk();
   public:
    FCCBridgeNode();
};

}  // namespace fcc_bridge

#endif  // THI_DRONE_WS_FCC_BRIDGE_NODE_HPP
