//
// Created by Johan <job8197@thi.de> on 11.05.2024.
//

#include "test_header.hpp"

namespace fcc_bridge_test {

using RES_TYPE = mavsdk::Telemetry::Result;

using TestMAVSDKRTelemetryRateFailure = ValuedTestFixture<RES_TYPE>;

/**
 * @brief Test that the telemetry result checker correctly detects an error
 * code and tries to exit the process
 */
TEST_P(TestMAVSDKRTelemetryRateFailure, TelemetryRateSetFailure) {
    RCLCPP_DEBUG(TEST_LOGGER, "Testing: %s",
                 ::fcc_bridge::FCCBridgeNode::mavsdk_telemetry_result_to_str(
                     GetParam()));
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::MAVSDK_SET_UP)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_telemetry_result(
                     GetParam(), testing::UnitTest::GetInstance()
                                     ->current_test_info()
                                     ->test_case_name()),
                 normal_fcc_exit);
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, TestMAVSDKRTelemetryRateFailure,
                         testing::Values(RES_TYPE::Unknown, RES_TYPE::NoSystem,
                                         RES_TYPE::ConnectionError,
                                         RES_TYPE::Busy,
                                         RES_TYPE::CommandDenied,
                                         RES_TYPE::Timeout,
                                         RES_TYPE::Unsupported));

/**
 * @brief Test that a successful telemetry rate setting does not create an exit
 * */
TEST_F(BaseTestFixture, TelemetryRateSetSucess) {
    RCLCPP_DEBUG(TEST_LOGGER,
                 "Testing telemetry rate checker with success result");
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::MAVSDK_SET_UP)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    EXPECT_NO_THROW(this->fcc_bridge_node_wrapper->check_telemetry_result(
        RES_TYPE::Success, this->test_info_->test_case_name()));
    ASSERT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              FCCBridgeNodeWrapper::INTERNAL_STATE::MAVSDK_SET_UP)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

}  // namespace fcc_bridge_test
