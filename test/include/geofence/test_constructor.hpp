//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_TEST_CONSTRUCTOR_HPP
#define THI_DRONE_WS_TEST_CONSTRUCTOR_HPP

#include <gtest/gtest.h>

#include "geofence/test_geofence.hpp"

namespace fcc_bridge::test::geofence {

template <typename T>
class GeofenceConstructor : public testing::Test {};

}  // namespace fcc_bridge::test::geofence

#endif  // THI_DRONE_WS_TEST_CONSTRUCTOR_HPP
