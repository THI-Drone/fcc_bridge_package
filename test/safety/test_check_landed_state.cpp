//
// Created by Johan <job8197@thi.de> on 15.05.2024.
//

#include "fcc_exit_exceptions.hpp"
#include "safety/safety_fixtures.hpp"
#include "test_logger.hpp"

namespace fcc_bridge::test::safety {

using LandedState = mavsdk::Telemetry::LandedState;

using ERRORStateFixture = ValuedTestFixture<LandedState>;

}  // namespace fcc_bridge::test::safety