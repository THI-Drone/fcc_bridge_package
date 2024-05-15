//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#include "geofence/setup.hpp"

#include "geofence/test_isEqual.hpp"

namespace fcc_bridge::test::geofence::setup {

void setup() {
    TestAlmostEqualFinite::setup();
    TestEqualNonFinite::setup();
}

}  // namespace fcc_bridge::test::geofence::setup
