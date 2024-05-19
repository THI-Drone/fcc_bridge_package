//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_SAFETY_FIXTURES_HPP
#define THI_DRONE_WS_SAFETY_FIXTURES_HPP

#include "fcc_bridge_node_wrapper.hpp"
#include "test_fixtures.hpp"

namespace fcc_bridge::test::safety {

class FCCBridgeNodeWrapper : public ::fcc_bridge::test::FCCBridgeNodeWrapper {
    // check_telemetry_result test cases implemented in
    // test/safety/test_check_telemetry_result.cpp
    FRIEND_TEST(TestMAVSDKRTelemetryRateFailure, TelemetryRateSetFailure);
    FRIEND_TEST(BaseTestFixture, TelemetryRateSetSucess);

    // check_sender test cases implemented in
    // test/safety/test_check_sender.cpp
    FRIEND_TEST(FailureInAirborneStates, NoActiveNode);
    FRIEND_TEST(FailureInAirborneStates, ExpectedNodeNotActive);
    FRIEND_TEST(FailureInAirborneStates, ActualSenderNotExpected);
    FRIEND_TEST(FailureInOnGroundStates, NoActiveNode);
    FRIEND_TEST(FailureInOnGroundStates, ExpectedNodeNotActive);
    FRIEND_TEST(FailureInOnGroundStates, ActualSenderNotExpected);
    FRIEND_TEST(FailureInErrorState, NoActiveNode);
    FRIEND_TEST(FailureInErrorState, ExpectedNodeNotActive);
    FRIEND_TEST(FailureInErrorState, ActualSenderNotExpected);
    FRIEND_TEST(SuccessInAllStates, ValidSender);

    // mavsdk_rth_cb test cases implemented in
    // test/safety/test_mavsdk_rth_cb.cpp
    FRIEND_TEST(TestMAVSDKRTHCBFAILURE, RTHFailure);
    FRIEND_TEST(BaseTestFixture, MAVSDKRTHCBSUCESS);
};

using BaseTestFixture =
    ::fcc_bridge::test::BaseTestFixture<FCCBridgeNodeWrapper>;

template <typename T>
using ValuedTestFixture =
    ::fcc_bridge::test::ValuedTestFixture<T, FCCBridgeNodeWrapper>;

}  // namespace fcc_bridge::test::safety

#endif  // THI_DRONE_WS_SAFETY_FIXTURES_HPP
