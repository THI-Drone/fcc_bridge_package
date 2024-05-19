//
// Created by Johan <job8197@thi.de> on 11.05.2024.
//

#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"
#include "test_logger.hpp"

namespace fcc_bridge::test::safety {

using RES_TYPE = mavsdk::Telemetry::Result;

namespace {

#define ENUM_TO_STR(parent_namespace, member) \
    case parent_namespace::member:            \
        return #parent_namespace "_" #member

std::string res_type_to_suffix(const testing::TestParamInfo<RES_TYPE> &info) {
    switch (info.param) {
        ENUM_TO_STR(RES_TYPE, Unknown);
        ENUM_TO_STR(RES_TYPE, Success);
        ENUM_TO_STR(RES_TYPE, NoSystem);
        ENUM_TO_STR(RES_TYPE, ConnectionError);
        ENUM_TO_STR(RES_TYPE, Busy);
        ENUM_TO_STR(RES_TYPE, CommandDenied);
        ENUM_TO_STR(RES_TYPE, Timeout);
        ENUM_TO_STR(RES_TYPE, Unsupported);
        default:
            throw unknown_enum_value_error(
                std::string(
                    "Got an unknown mavsdk::Telemetry::Result value: ") +
                std::to_string(static_cast<int>(info.param)));
    }
}

}  // namespace

using TestMAVSDKRTelemetryRateFailure = ValuedTestFixture<RES_TYPE>;

/**
 * @brief Test that the telemetry result checker correctly detects an error
 * code and tries to exit the process
 */
TEST_P(TestMAVSDKRTelemetryRateFailure, TelemetryRateSetFailure) {
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
                                         RES_TYPE::Unsupported),
                         res_type_to_suffix);

/**
 * @brief Test that a successful telemetry rate setting does not create an exit
 * */
TEST_F(BaseTestFixture, TelemetryRateSetSucess) {
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

}  // namespace fcc_bridge::test::safety
