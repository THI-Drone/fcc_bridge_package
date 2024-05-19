//
// Created by Johan <job8197@thi.de> on 19.05.2024.
//

// Libc header
#include <limits>

// Interfaces header
#include <interfaces/msg/safety_limits.hpp>

// CommonLib header
#include <common_package/node_names.hpp>

// Test header
#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"

namespace fcc_bridge::test::safety {

class SafetyLimit : public ValuedTestFixture<INTERNAL_STATE> {
   protected:
    using StructLimits = FCCBridgeNodeWrapper::SafetyLimits;
    using SafetyLimits = ::interfaces::msg::SafetyLimits;
    using Waypoint = ::interfaces::msg::Waypoint;
    SafetyLimits dummy_safety_limits;

   public:
    SafetyLimit() {
        this->dummy_safety_limits.sender_id =
            ::common_lib::node_names::MISSION_CONTROL;
        this->dummy_safety_limits.max_height_m = 5;
        this->dummy_safety_limits.min_soc = 80;
        this->dummy_safety_limits.max_speed_m_s = 10;
        Waypoint waypoint;
        waypoint.relative_altitude_m = Waypoint::INVALID_ALTITUDE;
        waypoint.longitude_deg = 0;
        waypoint.latitude_deg = 0;
        this->dummy_safety_limits.geofence_points.push_back(waypoint);
        waypoint.longitude_deg = 2;
        this->dummy_safety_limits.geofence_points.push_back(waypoint);
        waypoint.longitude_deg = 1;
        waypoint.latitude_deg = 1;
        this->dummy_safety_limits.geofence_points.push_back(waypoint);
    }
};

TEST_P(SafetyLimit, NoSafetyLimit) {
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

TEST_P(SafetyLimit, SpeedLimit) {
    // Test allowed value
    this->dummy_safety_limits.max_speed_m_s = 2;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
              2)
        << "Actual max_speed value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test to minimum value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.max_speed_m_s =
        FCCBridgeNodeWrapper::SafetyLimits::MIN_SPEED_LIMIT_MPS;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
              StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
        << "Actual max_speed value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test negative value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.max_speed_m_s = -1;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
              StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
        << "Actual max_speed value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test too large value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.max_speed_m_s =
        StructLimits::HARD_MAX_SPEED_LIMIT_MPS + 1;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
              StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
        << "Actual max_speed value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    if constexpr (std::numeric_limits<float>::has_infinity) {
        // Test positive infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_speed_m_s =
            std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
            StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
            << "Actual max_speed value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()
                   ->max_speed_mps;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
        // Test negative infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_speed_m_s =
            -std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
            StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
            << "Actual max_speed value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()
                   ->max_speed_mps;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_quiet_NaN) {
        // Test quiet NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_speed_m_s =
            std::numeric_limits<float>::quiet_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
            StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
            << "Actual max_speed value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()
                   ->max_speed_mps;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_signaling_NaN) {
        // Test signaling NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_speed_m_s =
            std::numeric_limits<float>::signaling_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_speed_mps,
            StructLimits::HARD_MAX_SPEED_LIMIT_MPS)
            << "Actual max_speed value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()
                   ->max_speed_mps;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
}

TEST_P(SafetyLimit, StateOfCharge) {
    // Test allowed value
    this->dummy_safety_limits.min_soc = 80;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc, 80)
        << "Actual min_soc value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test to minimum value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.min_soc =
        FCCBridgeNodeWrapper::SafetyLimits::HARD_MIN_SOC;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
              StructLimits::HARD_MIN_SOC)
        << "Actual min_soc value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test negative value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.min_soc = -1;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
              StructLimits::HARD_MIN_SOC)
        << "Actual min_soc value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    if constexpr (std::numeric_limits<float>::has_infinity) {
        // Test positive infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.min_soc =
            std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
                  StructLimits::HARD_MIN_SOC)
            << "Actual min_soc value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
        // Test negative infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.min_soc =
            -std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
                  StructLimits::HARD_MIN_SOC)
            << "Actual min_soc value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_quiet_NaN) {
        // Test quiet NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.min_soc =
            std::numeric_limits<float>::quiet_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
                  StructLimits::HARD_MIN_SOC)
            << "Actual min_soc value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_signaling_NaN) {
        // Test signaling NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.min_soc =
            std::numeric_limits<float>::signaling_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc,
                  StructLimits::HARD_MIN_SOC)
            << "Actual min_soc value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->min_soc;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
}

TEST_P(SafetyLimit, MaxHeight) {
    // Test allowed value
    this->dummy_safety_limits.max_height_m = 25;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
              25)
        << "Actual max_height_m value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Test too large value
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.max_height_m =
        FCCBridgeNodeWrapper::SafetyLimits::HARD_MAX_HEIGHT_M + 1;
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
              StructLimits::HARD_MAX_HEIGHT_M)
        << "Actual max_height_m value: "
        << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    if constexpr (std::numeric_limits<float>::has_infinity) {
        // Test positive infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_height_m =
            std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
            StructLimits::HARD_MAX_HEIGHT_M)
            << "Actual max_height_m value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
        // Test negative infinity
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_height_m =
            -std::numeric_limits<float>::infinity();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
            StructLimits::HARD_MAX_HEIGHT_M)
            << "Actual max_height_m value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_quiet_NaN) {
        // Test quiet NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_height_m =
            std::numeric_limits<float>::quiet_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
            StructLimits::HARD_MAX_HEIGHT_M)
            << "Actual max_height_m value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
    if constexpr (std::numeric_limits<float>::has_signaling_NaN) {
        // Test signaling NaN
        this->fcc_bridge_node_wrapper->set_internal_state(
            INTERNAL_STATE::MAVSDK_SET_UP);
        this->dummy_safety_limits.max_height_m =
            std::numeric_limits<float>::signaling_NaN();
        this->fcc_bridge_node_wrapper->safety_limits_cb(
            this->dummy_safety_limits);
        this->fcc_bridge_node_wrapper->set_internal_state(
            SafetyLimit::GetParam());
        this->fcc_bridge_node_wrapper->validate_safety_limits();
        ASSERT_TRUE(
            this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
        EXPECT_EQ(
            this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m,
            StructLimits::HARD_MAX_HEIGHT_M)
            << "Actual max_height_m value: "
            << this->fcc_bridge_node_wrapper->get_safety_limits()->max_height_m;
        EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
                  SafetyLimit::GetParam())
            << "Value of internal_state: "
            << this->fcc_bridge_node_wrapper->internal_state_to_str();
    }
}

TEST_P(SafetyLimit, Geofence) {
    // Pass valid geofence
    this->fcc_bridge_node_wrapper->safety_limits_cb(this->dummy_safety_limits);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()
                  ->geofence.get_polygon_point_count(),
              3);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              SafetyLimit::GetParam())
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
    // Invalid geofence
    this->fcc_bridge_node_wrapper->set_internal_state(
        INTERNAL_STATE::MAVSDK_SET_UP);
    this->dummy_safety_limits.geofence_points[1].longitude_deg = 0;
    EXPECT_THROW(this->fcc_bridge_node_wrapper->safety_limits_cb(
                     this->dummy_safety_limits),
                 normal_fcc_exit);
    this->fcc_bridge_node_wrapper->set_internal_state(SafetyLimit::GetParam());
    this->fcc_bridge_node_wrapper->validate_safety_limits();
    ASSERT_TRUE(this->fcc_bridge_node_wrapper->get_safety_limits().has_value());
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_safety_limits()
                  ->geofence.get_polygon_point_count(),
              2);
    EXPECT_EQ(this->fcc_bridge_node_wrapper->get_internal_state(),
              INTERNAL_STATE::ERROR)
        << "Value of internal_state: "
        << this->fcc_bridge_node_wrapper->internal_state_to_str();
}

INSTANTIATE_TEST_SUITE_P(, SafetyLimit, ALL_STATES, internal_state_suffix_gen);

}  // namespace fcc_bridge::test::safety
