//
// Created by Johan <job8197@thi.de> on 15.05.2024.
//

#include "fcc_bridge_node_mavsdk_mock.hpp"
#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"

namespace fcc_bridge::test::safety {

namespace {

using LandedState = mavsdk::Telemetry::LandedState;

const auto ALL_LANDED_STATES = testing::Values(
    LandedState::Unknown, LandedState::OnGround, LandedState::TakingOff,
    LandedState::InAir, LandedState::Landing);

#define ENUM_TO_STR(parent_namespace, member) \
    case parent_namespace::member:            \
        return #parent_namespace "_" #member

const char *landed_state_to_suffix(const LandedState &landed_state) {
    switch (landed_state) {
        ENUM_TO_STR(LandedState, Unknown);
        ENUM_TO_STR(LandedState, OnGround);
        ENUM_TO_STR(LandedState, TakingOff);
        ENUM_TO_STR(LandedState, InAir);
        ENUM_TO_STR(LandedState, Landing);
        default:
            throw unknown_enum_value_error(
                std::string(
                    "Got invalid mavsdk::Telemetry::LandedState value: ") +
                std::to_string(static_cast<int>(landed_state)));
    }
}

std::string landed_state_suffix_gen(
    const testing::TestParamInfo<LandedState> &param_info) {
    return landed_state_to_suffix(param_info.param);
}

std::string combined_suffix_gen(
    const testing::TestParamInfo<std::tuple<INTERNAL_STATE, LandedState>>
        &param_info) {
    const testing::TestParamInfo<INTERNAL_STATE> state_info(
        std::get<INTERNAL_STATE>(param_info.param), param_info.index);
    return internal_state_suffix_gen(state_info) + "__" +
           landed_state_to_suffix(std::get<LandedState>(param_info.param));
}

}  // namespace

template <typename T>
class LandedStateFixture : public ValuedTestFixture<T> {
   public:
    LandedStateFixture() = default;

    ~LandedStateFixture() { fake_landed_state.reset(); }
};

using ErrorInternalState = ValuedTestFixture<LandedState>;

TEST_P(ErrorInternalState, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(INTERNAL_STATE::ERROR);
    fake_landed_state = ErrorInternalState::GetParam();
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_landed_state(),
                 invalid_state_error);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, ErrorInternalState, ALL_LANDED_STATES,
                         landed_state_suffix_gen);

using InvalidInternalState =
    ValuedTestFixture<std::tuple<INTERNAL_STATE, LandedState>>;

TEST_P(InvalidInternalState, Test) {
    fake_landed_state = std::get<LandedState>(InvalidInternalState::GetParam());
    this->fcc_bridge_node_wrapper->set_internal_state(
        std::get<INTERNAL_STATE>(InvalidInternalState::GetParam()));
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_landed_state(),
                 normal_fcc_exit);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(
    , InvalidInternalState,
    testing::Combine(testing::Values(INTERNAL_STATE::STARTING_UP,
                                     INTERNAL_STATE::ROS_SET_UP),
                     ALL_LANDED_STATES),
    combined_suffix_gen);

}  // namespace fcc_bridge::test::safety