//
// Created by Johan <job8197@thi.de> on 10.05.2024.
//

#include "test_header.hpp"

namespace fcc_bridge_test {

using RES_TYPE = mavsdk::Action::Result;

using TestMAVSDKRTHCBFAILURE = ValuedTestFixture<RES_TYPE>;

/**
 * @brief Test that the RTH result callback function correctly detects an error
 * return code and tries to exit the process
 */
TEST_P(TestMAVSDKRTHCBFAILURE, RTHFailure) {
    RCLCPP_DEBUG(
        TEST_LOGGER, "Testing: %s",
        ::fcc_bridge::FCCBridgeNode::mavsdk_action_result_to_str(GetParam()));
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::MAVSDK_SET_UP)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    EXPECT_THROW(this->fcc_bridge_node_wrapper->mavsdk_rth_cb(GetParam()),
                 normal_fcc_exit);
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(
    , TestMAVSDKRTHCBFAILURE,
    testing::Values(RES_TYPE::Unknown, RES_TYPE::NoSystem,
                    RES_TYPE::ConnectionError, RES_TYPE::Busy,
                    RES_TYPE::CommandDenied,
                    RES_TYPE::CommandDeniedLandedStateUnknown,
                    RES_TYPE::CommandDeniedNotLanded, RES_TYPE::Timeout,
                    RES_TYPE::VtolTransitionSupportUnknown,
                    RES_TYPE::NoVtolTransitionSupport, RES_TYPE::ParameterError,
                    RES_TYPE::Unsupported, RES_TYPE::Failed));

/**
 * @brief Test that a successful landing results in the node being in LANDED
 * state
 */
TEST_F(BaseTestFixture, MAVSDKRTHCBSUCESS) {
    RCLCPP_DEBUG(TEST_LOGGER, "Testing RTH callback with success result");
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::MAVSDK_SET_UP)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    EXPECT_NO_THROW(
        this->fcc_bridge_node_wrapper->mavsdk_rth_cb(RES_TYPE::Success));
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::LANDED)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

// TODO: Test disarming once implemented
}  // namespace fcc_bridge_test
