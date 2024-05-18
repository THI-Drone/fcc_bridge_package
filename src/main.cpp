#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

// CommonLib header
#include "common_package/node_names.hpp"
#include "fcc_bridge_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fcc_bridge::FCCBridgeNode>(
        common_lib::node_names::MISSION_CONTROL));
    rclcpp::shutdown();
    return 0;
}
