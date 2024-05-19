//
// Created by Johan <job8197@thi.de> on 19.05.2024.
//

#include "test_fixtures.hpp"

namespace fcc_bridge::test {

#define ENUM_TO_STR(parent_namespace, member) \
    case parent_namespace::member:            \
        return #parent_namespace "_" #member

std::string internal_state_suffix_gen(
    const testing::TestParamInfo<INTERNAL_STATE> &info) {
    switch (info.param) {
        ENUM_TO_STR(INTERNAL_STATE, ERROR);
        ENUM_TO_STR(INTERNAL_STATE, STARTING_UP);
        ENUM_TO_STR(INTERNAL_STATE, ROS_SET_UP);
        ENUM_TO_STR(INTERNAL_STATE, MAVSDK_SET_UP);
        ENUM_TO_STR(INTERNAL_STATE, WAITING_FOR_ARM);
        ENUM_TO_STR(INTERNAL_STATE, ARMED);
        ENUM_TO_STR(INTERNAL_STATE, TAKING_OFF);
        ENUM_TO_STR(INTERNAL_STATE, WAITING_FOR_COMMAND);
        ENUM_TO_STR(INTERNAL_STATE, FLYING_MISSION);
        ENUM_TO_STR(INTERNAL_STATE, LANDING);
        ENUM_TO_STR(INTERNAL_STATE, RETURN_TO_HOME);
        ENUM_TO_STR(INTERNAL_STATE, LANDED);
        default:
            throw unknown_enum_value_error(
                std::string("Got unknown INTERNAL_STATE value: ") +
                std::to_string(static_cast<int>(info.param)));
    }
}

}  // namespace fcc_bridge::test
