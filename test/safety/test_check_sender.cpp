//
// Created by Johan <job8197@thi.de> on 19.05.2024.
//

// Interfaces header
#include <interfaces/msg/control.hpp>

// Common lib header
#include <common_package/node_names.hpp>

#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"

namespace fcc_bridge::test::safety{

    /*
     * Checks the correct behaviour for an invalid sender if the UAV is airborne
     */

    using FailureInAirborneStates = ValuedTestFixture<INTERNAL_STATE>;

    TEST_P(FailureInAirborneStates, NoActiveNode) {
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInAirborneStates::GetParam());
        EXPECT_FALSE(this->fcc_bridge_node_wrapper->check_sender("", ""));
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(), INTERNAL_STATE::RETURN_TO_HOME);
    }

    TEST_P(FailureInAirborneStates, ExpectedNodeNotActive) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInAirborneStates::GetParam());
        EXPECT_FALSE(this->fcc_bridge_node_wrapper->check_sender("", ""));
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(), INTERNAL_STATE::RETURN_TO_HOME);
    }

    TEST_P(FailureInAirborneStates, ActualSenderNotExpected) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInAirborneStates::GetParam());
        EXPECT_FALSE(this->fcc_bridge_node_wrapper->check_sender("", active_node.c_str()));
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(), INTERNAL_STATE::RETURN_TO_HOME);
    }

    INSTANTIATE_TEST_SUITE_P(, FailureInAirborneStates, AIRBORNE_STATES);

    /*
     * Checks the correct behaviour for an invalid sender if the UAV is not airborne
     */

    using FailureInOnGroundStates = ValuedTestFixture<INTERNAL_STATE>;

    TEST_P(FailureInOnGroundStates, NoActiveNode) {
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInOnGroundStates::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", ""), normal_fcc_exit);
    }

    TEST_P(FailureInOnGroundStates, ExpectedNodeNotActive) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInOnGroundStates::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", ""), normal_fcc_exit);
    }

    TEST_P(FailureInOnGroundStates, ActualSenderNotExpected) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInOnGroundStates::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", active_node.c_str()), normal_fcc_exit);
    }

    INSTANTIATE_TEST_SUITE_P(, FailureInOnGroundStates, ON_GROUND_STATES);

    /*
     * Checks the correct behaviour for an invalid sender if the UAV is in the Error state
     */

    using FailureInErrorState = ValuedTestFixture<INTERNAL_STATE>;

    TEST_P(FailureInErrorState, NoActiveNode) {
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInErrorState::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", ""), invalid_state_error);
    }

    TEST_P(FailureInErrorState, ExpectedNodeNotActive) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInErrorState::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", ""), invalid_state_error);
    }

    TEST_P(FailureInErrorState, ActualSenderNotExpected) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            FailureInErrorState::GetParam());
        EXPECT_THROW(this->fcc_bridge_node_wrapper->check_sender("", active_node.c_str()), invalid_state_error);
    }

    INSTANTIATE_TEST_SUITE_P(, FailureInErrorState, ERROR_STATE);

    /*
     * Checks the correct behaviour for a valid sender if the UAV is in any state
     */

    using SuccessInAllStates = ValuedTestFixture<INTERNAL_STATE>;

    TEST_P(SuccessInAllStates, ValidSender) {
        const static std::string active_node = *(::common_lib::node_names::VALID_CONTROL_NODE_NAMES.begin());
        ::interfaces::msg::Control msg;
        msg.target_id = active_node;
        msg.active = true;
        this->fcc_bridge_node_wrapper->control_cb(msg);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SuccessInAllStates::GetParam());
        EXPECT_TRUE(this->fcc_bridge_node_wrapper->check_sender(active_node, active_node.c_str()));
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(), SuccessInAllStates::GetParam());
    }

    INSTANTIATE_TEST_SUITE_P(, SuccessInAllStates, ALL_STATES);

}