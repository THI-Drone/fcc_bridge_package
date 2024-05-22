//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_SAFETY_FIXTURES_HPP
#define THI_DRONE_WS_SAFETY_FIXTURES_HPP

#include "fcc_bridge_node_wrapper.hpp"
#include "test_fixtures.hpp"

namespace fcc_bridge::test::safety {

class FCCBridgeNodeWrapper : public ::fcc_bridge::test::FCCBridgeNodeWrapper {
    // check_gps_state test cases implemented in
    // test/safety/test_check_gps_state.cpp
    template <typename>
    friend class GPSTestFixture;
    FRIEND_TEST(GPSErrorTestFixture, ErrorState);
    FRIEND_TEST(GPSInvalidStateTestFixture, InvalidState);
    FRIEND_TEST(GPSValidFixTypeTestFixture, Test);
    FRIEND_TEST(GPSInValidFixTypeTestFixture, Test);
    FRIEND_TEST(PosInGeofence, Test);
    FRIEND_TEST(GeofenceViolationAirborne, Test);
    FRIEND_TEST(GeofenceViolationOnGround, Test);

    // check_telemetry_result test cases implemented in
    // test/safety/test_check_telemetry_result.cpp
    FRIEND_TEST(TelemetryRateSetFailure, TelemetryRateSetFailure);
    FRIEND_TEST(TelemetryRateSetSucess, TelemetryRateSetSucess);

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

    // validate_safety_limits test cases implemented in
    // test/safety/test_validate_safety_limits.cpp
    FRIEND_TEST(SafetyLimit, NoSafetyLimit);
    FRIEND_TEST(SafetyLimit, ValidSpeedLimit);
    FRIEND_TEST(SafetyLimit, MinimumSpeedLimit);
    FRIEND_TEST(SafetyLimit, NegativeSpeedLimit);
    FRIEND_TEST(SafetyLimit, TooLargeSpeedLimit);
    FRIEND_TEST(SafetyLimit, PositiveInfinitySpeedLimit);
    FRIEND_TEST(SafetyLimit, NegativeInfinitySpeedLimit);
    FRIEND_TEST(SafetyLimit, QuietNaNSpeedLimit);
    FRIEND_TEST(SafetyLimit, SignalingNaNSpeedLimit);
    FRIEND_TEST(SafetyLimit, ValidStateOfCharge);
    FRIEND_TEST(SafetyLimit, MinimumStateOfCharge);
    FRIEND_TEST(SafetyLimit, NegativeStateOfCharge);
    FRIEND_TEST(SafetyLimit, PositiveInfinityStateOfCharge);
    FRIEND_TEST(SafetyLimit, NegativeInfinityStateOfCharge);
    FRIEND_TEST(SafetyLimit, QuietNaNStateOfCharge);
    FRIEND_TEST(SafetyLimit, SignalingNaNStateOfCharge);
    FRIEND_TEST(SafetyLimit, ValidMaxHeight);
    FRIEND_TEST(SafetyLimit, TooLargeMaxHeight);
    FRIEND_TEST(SafetyLimit, PositiveInfinityMaxHeight);
    FRIEND_TEST(SafetyLimit, NegativeInfinityMaxHeight);
    FRIEND_TEST(SafetyLimit, QuietNaNMaxHeight);
    FRIEND_TEST(SafetyLimit, SignalingNaNMaxHeight);
    FRIEND_TEST(SafetyLimit, ValidGeofence);
    FRIEND_TEST(SafetyLimit, InvalidGeofence);
    FRIEND_TEST(SafetyLimit, AllInvalid);
};

using BaseTestFixture =
    ::fcc_bridge::test::BaseTestFixture<FCCBridgeNodeWrapper>;

template <typename T>
using ValuedTestFixture =
    ::fcc_bridge::test::ValuedTestFixture<T, FCCBridgeNodeWrapper>;

}  // namespace fcc_bridge::test::safety

#endif  // THI_DRONE_WS_SAFETY_FIXTURES_HPP
