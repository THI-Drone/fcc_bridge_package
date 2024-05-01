#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "fcc_bridge_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fcc_bridge::FCCBridgeNode>("fcc_bridge"));
    rclcpp::shutdown();
    return 0;
}
