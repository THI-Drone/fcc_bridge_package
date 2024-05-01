//
// Created by Johan on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {
void FCCBridgeNode::setup_mavsdk() {
    RCLCPP_DEBUG(this->get_logger(), "Setting up MAVSDK");
    this->mavsdk = mavsdk::Mavsdk(mavsdk::Mavsdk::Configuration(0, MAV_COMP_ID_ONBOARD_COMPUTER, false));
}
}  // namespace fcc_bridge