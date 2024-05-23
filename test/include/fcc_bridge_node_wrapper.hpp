//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_FCC_BRIDGE_NODE_WRAPPER_HPP
#define THI_DRONE_WS_FCC_BRIDGE_NODE_WRAPPER_HPP

#include <gtest/gtest.h>

#include "fcc_bridge/fcc_bridge_node.hpp"

namespace fcc_bridge::test {

/**
 * @brief Wrapper around fcc_bridge::FCCBridgeNode to make protected members
 * available
 */
class FCCBridgeNodeWrapper : public FCCBridgeNode {
   public:
    /**
     * @brief Constructor creates an instance of fcc_bridge::FCCBridgeNode with
     * name test_mavsdk_rth_fcc_bridge
     */
    FCCBridgeNodeWrapper() : FCCBridgeNode("test_fcc_bridge") {}
};

}  // namespace fcc_bridge::test

#endif  // THI_DRONE_WS_FCC_BRIDGE_NODE_WRAPPER_HPP
