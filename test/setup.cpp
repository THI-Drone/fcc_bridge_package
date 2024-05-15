//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#include "setup.hpp"

#include "geofence/setup.hpp"
#include "safety/setup.hpp"

namespace fcc_bridge::test::setup {

void setup() {
    geofence::setup::setup();
    safety::setup::setup();
}

}  // namespace fcc_bridge::test::setup
