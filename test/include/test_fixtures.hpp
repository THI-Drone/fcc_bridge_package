//
// Created by Johan <job8197@thi.de> on 10.05.2024.
//

#ifndef THI_DRONE_WS_TEST_HEADER_HPP
#define THI_DRONE_WS_TEST_HEADER_HPP

#include <gtest/gtest.h>

#include "fcc_bridge_node.hpp"

/**
 * @brief Namespace for testcases and related members for the fcc_bridge
 */
namespace fcc_bridge::test {

/**
 * @brief Error to be thrown instead of exiting the process by
 * exit_process_on_error if FCCBridgeNode::internal_state is set to ERROR
 */
class normal_fcc_exit : std::runtime_error {
   public:
    /**
     * @brief Constructs an exception to signal exit_process_on_error was called
     * with FCCBridgeNode::internal_state set to ERROR
     *
     * @param func_name The name of the function were fcc_exit was thrown
     */
    explicit normal_fcc_exit(const std::string &func_name) noexcept
        : std::runtime_error(func_name + " was called") {}
};

/**
 * @brief Error to be thrown instead of exiting the process by
 * exit_process_on_error if FCCBridgeNode::internal_state is not set to ERROR
 */
class abnormal_fcc_exit : std::runtime_error {
   public:
    /**
     * @brief Constructs an exception to signal exit_process_on_error was called
     * without FCCBridgeNode::internal_state set to ERROR
     *
     * @param func_name The name of the function were fcc_exit was thrown
     */
    explicit abnormal_fcc_exit(const std::string &func_name) noexcept
        : std::runtime_error(func_name + " was called") {}
};

/**
 * @brief Wrapper around fcc_bridge::FCCBridgeNode to make protected members
 * available
 */
class FCCBridgeNodeWrapper : public fcc_bridge::FCCBridgeNode {
   public:
    /**
     * @brief Constructor creates an instance of fcc_bridge::FCCBridgeNode with
     * name test_mavsdk_rth_fcc_bridge
     */
    FCCBridgeNodeWrapper()
        : fcc_bridge::FCCBridgeNode("test_mavsdk_rth_fcc_bridge") {}

    /************************************************************************/
    /*           Friend test cases to allow access to private and           */
    /*                 protected members of this wrapper                    */
    /************************************************************************/

    /**
     * @brief Safety related friends found in test/safety
     */
    // mavsdk_rth_cb test cases implemented in
    // test/safety/test_mavsdk_rth_cb.cpp
    FRIEND_TEST(TestMAVSDKRTHCBFAILURE, RTHFailure);
    FRIEND_TEST(BaseTestFixture, MAVSDKRTHCBSUCESS);

    // check_telemetry_result test cases implemented in
    // test/safety/test_check_telemetry_result.cpp
    FRIEND_TEST(TestMAVSDKRTelemetryRateFailure, TelemetryRateSetFailure);
    FRIEND_TEST(BaseTestFixture, TelemetryRateSetSucess);
};

/**
 * @brief Base test fixture class initializing the default ros context to domain
 * id 3 and creating a FCCBridgeNodeWrapper object
 */
class BaseTestFixture : public testing::Test {
   protected:
    std::shared_ptr<FCCBridgeNodeWrapper>
        fcc_bridge_node_wrapper; /**< Object to hold an instance of
                                    FCCBridgeNodeWrapper */

    /**
     * @brief Constructor to initialize the default context and create the
     * wrapper object
     */
    BaseTestFixture() {
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(3);
        rclcpp::init(0, nullptr, init_options);
        this->fcc_bridge_node_wrapper =
            std::make_shared<FCCBridgeNodeWrapper>();
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
 */
template <typename T>
class ValuedTestFixture : public testing::WithParamInterface<T>,
                          public BaseTestFixture {};

const rclcpp::Logger TEST_LOGGER = rclcpp::get_logger(
    "fcc_bridge_test_logger"); /**< Logger to be used in test cases */
}  // namespace fcc_bridge_test

#endif  // THI_DRONE_WS_TEST_HEADER_HPP
