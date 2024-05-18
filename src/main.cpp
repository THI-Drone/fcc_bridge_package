// rclcpp header
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

// CommonLib header
#include "common_package/node_names.hpp"

// FCC Bridge header
#include "fcc_bridge_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fcc_bridge::FCCBridgeNode>(
            common_lib::node_names::MISSION_CONTROL));
    } catch (const fcc_bridge::invalid_state_error &e) {
        RCLCPP_FATAL(rclcpp::get_logger("EMERGENCY"),
                     "A invalid state error was thrown by the fcc bridge: %s",
                     e.what());
        rclcpp::shutdown();
        throw;
    } catch (const fcc_bridge::unknown_enum_value_error &e) {
        RCLCPP_FATAL(rclcpp::get_logger("EMERGENCY"),
                     "A unknown enum value was thrown by the fcc bridge: %s",
                     e.what());
        rclcpp::shutdown();
        throw;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
