//
// Created by Johan <job8197@thi.de> on 10.05.2024.
//

/**
 * @brief Mock class to emulate all calls into MAVSDK that would require a
 * MAVLink network and an autopilot
 */

#include "test_header.hpp"

namespace fcc_bridge {

void FCCBridgeNode::setup_mavsdk() {}

void FCCBridgeNode::verify_mavsdk_connection() {}

void FCCBridgeNode::get_gps_telemetry() {}

void FCCBridgeNode::get_flight_state() {}

void FCCBridgeNode::get_battery_state() {}

void FCCBridgeNode::get_rc_state() {}

void FCCBridgeNode::get_euler_angle() {}

void FCCBridgeNode::get_mission_progress() {}

void FCCBridgeNode::get_uav_health() {}

bool FCCBridgeNode::execute_mission_plan(
    const mavsdk::Mission::MissionPlan &plan) {
    (void)plan;
    return true;
}

void FCCBridgeNode::trigger_rth() {}

void FCCBridgeNode::exit_process_on_error() {
    if (this->internal_state != INTERNAL_STATE::ERROR) {
        throw fcc_bridge_test::abnormal_fcc_exit(
            std::string(__func__) +
            " was called without internal state set to error");
    }
    throw fcc_bridge_test::normal_fcc_exit(
        std::string(__func__) + " was called with internal state set to error");
}
}  // namespace fcc_bridge