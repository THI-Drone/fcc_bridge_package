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

}  // namespace fcc_bridge::test

#endif  // THI_DRONE_WS_TEST_FIXTURES_HPP
