//
// Created by Johan on 30.04.2024.
//

#include "fcc_bridge_node.hpp"

namespace fcc_bridge {
    FCCBridgeNode::FCCBridgeNode()  : CommonNode("fcc_bridge"), internal_state(INTERNAL_STATE::STARTING_UP) {
        RCLCPP_DEBUG(this->get_logger(), "Created %s instance", this->get_name());
    }

    void FCCBridgeNode::setup_ros() {
        RCLCPP_DEBUG(this->get_logger(), "Setting up ROS");
        if(this->internal_state != INTERNAL_STATE::STARTING_UP) {
            RCLCPP_ERROR(this->get_logger(), "Repeated try to setup ros components");
            return;
        }
    }
}
