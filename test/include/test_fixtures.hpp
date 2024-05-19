//
// Created by Johan <job8197@thi.de> on 10.05.2024.
//

#ifndef THI_DRONE_WS_TEST_FIXTURES_HPP
#define THI_DRONE_WS_TEST_FIXTURES_HPP

#include <gtest/gtest.h>

#include <type_traits>

#include "fcc_bridge_node_wrapper.hpp"

/**
 * @brief Namespace for testcases and related members for the fcc_bridge
 */
namespace fcc_bridge::test {

/**
 * @brief Base test fixture class initializing the default ros context to domain
 * id 3 and creating a FCCBridgeNodeWrapper object
 *
 * @tparam T A derived class from FCCBridgeNodeWrapper
 */
template <class T>
class BaseTestFixture : public testing::Test {
    static_assert(std::is_base_of_v<FCCBridgeNodeWrapper, T>,
                  "BaseFixture template parameter must be derived from "
                  "FCCBridgeNodeWrapper");

   protected:
    std::shared_ptr<T> fcc_bridge_node_wrapper; /**< Object to hold an instance
                                                   of FCCBridgeNodeWrapper */

    /**
     * @brief Constructor to initialize the default context and create the
     * wrapper object
     */
    BaseTestFixture() {
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(3);
        rclcpp::init(0, nullptr, init_options);
        this->fcc_bridge_node_wrapper = std::make_shared<T>();
    }

    /**
     * @brief Destructor to shutdown the ros context
     */
    ~BaseTestFixture() override { rclcpp::shutdown(); }
};

/**
 * @brief Template fixture to allow for parameterised tests that use the the
 * FCCBridgeNode
 *
 * @tparam T Type of the parameter to use
 * @tparam NodeWrapper A derived class from FCCBridgeNodeWrapper
 */
template <typename T, class NodeWrapper>
class ValuedTestFixture : public testing::WithParamInterface<T>,
                          public BaseTestFixture<NodeWrapper> {};

using INTERNAL_STATE = FCCBridgeNodeWrapper::INTERNAL_STATE;

const inline auto ALL_STATES = testing::Values<INTERNAL_STATE>(
    INTERNAL_STATE::ERROR, INTERNAL_STATE::STARTING_UP,
    INTERNAL_STATE::ROS_SET_UP, INTERNAL_STATE::MAVSDK_SET_UP,
    INTERNAL_STATE::WAITING_FOR_ARM, INTERNAL_STATE::ARMED,
    INTERNAL_STATE::TAKING_OFF, INTERNAL_STATE::WAITING_FOR_COMMAND,
    INTERNAL_STATE::FLYING_MISSION, INTERNAL_STATE::LANDING,
    INTERNAL_STATE::RETURN_TO_HOME, INTERNAL_STATE::LANDED);

const inline auto AIRBORNE_STATES = testing::Values<INTERNAL_STATE>(
    INTERNAL_STATE::TAKING_OFF, INTERNAL_STATE::WAITING_FOR_COMMAND,
    INTERNAL_STATE::FLYING_MISSION, INTERNAL_STATE::LANDING,
    INTERNAL_STATE::RETURN_TO_HOME);

const inline auto ON_GROUND_STATES = testing::Values<INTERNAL_STATE>(
    INTERNAL_STATE::STARTING_UP, INTERNAL_STATE::ROS_SET_UP,
    INTERNAL_STATE::MAVSDK_SET_UP, INTERNAL_STATE::WAITING_FOR_ARM,
    INTERNAL_STATE::ARMED, INTERNAL_STATE::LANDED);

const inline auto ERROR_STATE =
    testing::Values<INTERNAL_STATE>(INTERNAL_STATE::ERROR);

std::string internal_state_suffix_gen(
    const testing::TestParamInfo<INTERNAL_STATE>& info);

}  // namespace fcc_bridge::test

#endif  // THI_DRONE_WS_TEST_FIXTURES_HPP
