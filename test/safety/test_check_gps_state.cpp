//
// Created by Johan <job8197@thi.de> on 20.05.2024.
//

// Interfaces header
#include <interfaces/msg/safety_limits.hpp>

// Commonlib header
#include <common_package/node_names.hpp>

// Test header
#include "fcc_bridge_node_mavsdk_mock.hpp"
#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"

namespace fcc_bridge::test::safety {

namespace {

using FixType = mavsdk::Telemetry::FixType;

const auto HARD_CHECK_STATES = testing::Values(
    INTERNAL_STATE::ARMED, INTERNAL_STATE::TAKING_OFF,
    INTERNAL_STATE::WAITING_FOR_COMMAND, INTERNAL_STATE::FLYING_MISSION,
    INTERNAL_STATE::LANDING, INTERNAL_STATE::RETURN_TO_HOME);

const auto VALID_HARD_CHECK_FIX_TYPES =
    testing::Values(FixType::Fix2D, FixType::Fix3D, FixType::FixDgps,
                    FixType::RtkFloat, FixType::RtkFixed);

const auto INVALID_HARD_CHECK_FIX_TYPES =
    testing::Values(FixType::NoGps, FixType::NoFix);

const auto SOFT_CHECK_STATES =
    testing::Values(INTERNAL_STATE::WAITING_FOR_ARM,
                    INTERNAL_STATE::MAVSDK_SET_UP, INTERNAL_STATE::LANDED);

const auto VALID_SOFT_CHECK_FIX_TYPES =
    testing::Values(FixType::NoFix, FixType::Fix2D, FixType::Fix3D,
                    FixType::FixDgps, FixType::RtkFloat, FixType::RtkFixed);

const auto INVALID_SOFT_CHECK_FIX_TYPES = testing::Values(FixType::NoGps);

#define ENUM_TO_STR(parent_namespace, member) \
    case parent_namespace::member:            \
        return #parent_namespace "_" #member

const char *fix_type_to_suffix(const FixType &fix_type) {
    switch (fix_type) {
        ENUM_TO_STR(FixType, NoGps);
        ENUM_TO_STR(FixType, NoFix);
        ENUM_TO_STR(FixType, Fix2D);
        ENUM_TO_STR(FixType, Fix3D);
        ENUM_TO_STR(FixType, FixDgps);
        ENUM_TO_STR(FixType, RtkFloat);
        ENUM_TO_STR(FixType, RtkFixed);
        default:
            throw unknown_enum_value_error(
                std::string("Got invalid mavsdk::Telemetry::FixType value: ") +
                std::to_string(static_cast<int>(fix_type)));
    }
}

std::string suffix_gen(
    const testing::TestParamInfo<std::tuple<INTERNAL_STATE, FixType>>
        &param_info) {
    const testing::TestParamInfo<INTERNAL_STATE> state_info(
        std::get<INTERNAL_STATE>(param_info.param), param_info.index);
    return internal_state_suffix_gen(state_info) + "__" +
           fix_type_to_suffix(std::get<FixType>(param_info.param));
}

}  // namespace

using GPSErrorTestFixture = ValuedTestFixture<INTERNAL_STATE>;

TEST_P(GPSErrorTestFixture, ErrorState) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        GPSErrorTestFixture::GetParam());
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_gps_state(),
                 invalid_state_error);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, GPSErrorTestFixture, ERROR_STATE,
                         internal_state_suffix_gen);

using GPSInvalidStateTestFixture = ValuedTestFixture<INTERNAL_STATE>;

TEST_P(GPSInvalidStateTestFixture, InvalidState) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        GPSErrorTestFixture::GetParam());
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_gps_state(),
                 normal_fcc_exit);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, GPSInvalidStateTestFixture,
                         testing::Values(INTERNAL_STATE::STARTING_UP,
                                         INTERNAL_STATE::ROS_SET_UP),
                         internal_state_suffix_gen);

template <typename T>
class GPSTestFixture : public ValuedTestFixture<T> {
   protected:
    constexpr static float MAX_HEIGHT_M = 20;

   public:
    GPSTestFixture() {
        mavsdk::Telemetry::GpsInfo gps_info;
        gps_info.fix_type = mavsdk::Telemetry::FixType::Fix3D;
        gps_info.num_satellites = 5;
        fake_gps_info = gps_info;

        mavsdk::Telemetry::Position gps_position;
        gps_position.longitude_deg = 1;
        gps_position.latitude_deg = 1;
        gps_position.relative_altitude_m = 5;
        gps_position.absolute_altitude_m = 10;
        fake_gps_position = gps_position;

        interfaces::msg::SafetyLimits safety_limits;
        safety_limits.sender_id = ::common_lib::node_names::MISSION_CONTROL;
        safety_limits.max_height_m = MAX_HEIGHT_M;
        safety_limits.max_speed_m_s = 2;
        safety_limits.min_soc = 50;
        interfaces::msg::Waypoint waypoint;
        waypoint.longitude_deg = 0;
        waypoint.latitude_deg = 0;
        safety_limits.geofence_points.push_back(waypoint);
        waypoint.longitude_deg = 2;
        safety_limits.geofence_points.push_back(waypoint);
        waypoint.longitude_deg = 1;
        waypoint.latitude_deg = 2;
        safety_limits.geofence_points.push_back(waypoint);
        this->fcc_bridge_node_wrapper->safety_limits_cb(safety_limits);
    }
    ~GPSTestFixture() {
        fake_gps_info.reset();
        fake_gps_position.reset();
    }
};

using GPSValidFixTypeTestFixture =
    GPSTestFixture<std::tuple<INTERNAL_STATE, FixType>>;

TEST_P(GPSValidFixTypeTestFixture, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        std::get<INTERNAL_STATE>(GPSValidFixTypeTestFixture::GetParam()));
    fake_gps_info->fix_type =
        std::get<FixType>(GPSValidFixTypeTestFixture::GetParam());
    EXPECT_NO_THROW(this->fcc_bridge_node_wrapper->check_gps_state());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              std::get<INTERNAL_STATE>(GPSValidFixTypeTestFixture::GetParam()))
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(HardCheck, GPSValidFixTypeTestFixture,
                         testing::Combine(HARD_CHECK_STATES,
                                          VALID_HARD_CHECK_FIX_TYPES),
                         suffix_gen);

INSTANTIATE_TEST_SUITE_P(SoftCheck, GPSValidFixTypeTestFixture,
                         testing::Combine(SOFT_CHECK_STATES,
                                          VALID_SOFT_CHECK_FIX_TYPES),
                         suffix_gen);

using GPSInValidFixTypeTestFixture =
    GPSTestFixture<std::tuple<INTERNAL_STATE, FixType>>;

TEST_P(GPSInValidFixTypeTestFixture, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        std::get<INTERNAL_STATE>(GPSInValidFixTypeTestFixture::GetParam()));
    fake_gps_info->fix_type =
        std::get<FixType>(GPSInValidFixTypeTestFixture::GetParam());
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_gps_state(),
                 normal_fcc_exit);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(HardCheck, GPSInValidFixTypeTestFixture,
                         testing::Combine(HARD_CHECK_STATES,
                                          INVALID_HARD_CHECK_FIX_TYPES),
                         suffix_gen);

INSTANTIATE_TEST_SUITE_P(SoftCheck, GPSInValidFixTypeTestFixture,
                         testing::Combine(SOFT_CHECK_STATES,
                                          INVALID_SOFT_CHECK_FIX_TYPES),
                         suffix_gen);

using PosInGeofence = GPSTestFixture<INTERNAL_STATE>;

TEST_P(PosInGeofence, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        PosInGeofence::GetParam());
    EXPECT_NO_THROW(this->fcc_bridge_node_wrapper->check_gps_state());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              PosInGeofence::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

using GeofenceViolationAirborne = GPSTestFixture<INTERNAL_STATE>;

TEST_P(GeofenceViolationAirborne, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        GeofenceViolationAirborne::GetParam());
    fake_gps_position->longitude_deg = -1;
    EXPECT_NO_THROW(this->fcc_bridge_node_wrapper->check_gps_state());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::RETURN_TO_HOME)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, GeofenceViolationAirborne, AIRBORNE_STATES,
                         internal_state_suffix_gen);

using GeofenceViolationOnGround = GPSTestFixture<INTERNAL_STATE>;

TEST_P(GeofenceViolationOnGround, Test) {
    this->fcc_bridge_node_wrapper->set_internal_state(
        GeofenceViolationOnGround::GetParam());
    fake_gps_position->longitude_deg = -1;
    EXPECT_THROW(this->fcc_bridge_node_wrapper->check_gps_state(),
                 normal_fcc_exit);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

}  // namespace fcc_bridge::test::safety
