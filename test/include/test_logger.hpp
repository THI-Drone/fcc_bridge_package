//
// Created by Johan <job8917@thi.> on 13.05.2024.
//

#ifndef THI_DRONE_WS_TEST_LOGGER_HPP
#define THI_DRONE_WS_TEST_LOGGER_HPP

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fcc_bridge::test {

const rclcpp::Logger TEST_LOGGER = rclcpp::get_logger(
    "fcc_bridge_test_logger"); /**< Logger to be used in test cases */

}

#endif  // THI_DRONE_WS_TEST_LOGGER_HPP
