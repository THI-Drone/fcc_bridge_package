//
// Created by Johan <job8917@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_TEST_GEOFENCE_HPP
#define THI_DRONE_WS_TEST_GEOFENCE_HPP

#include <gtest/gtest.h>

#include "fcc_bridge/geofence.hpp"

namespace fcc_bridge::test::geofence {

template <typename T>
class TestGeofence : public Geofence<T> {
   public:
    using PointType = typename Geofence<T>::PointType;
    using PolygonType = typename Geofence<T>::PolygonType;

    constexpr explicit TestGeofence(
        const std::initializer_list<PointType> &init)
        : Geofence<T>(PolygonType(init)) {}

    constexpr explicit TestGeofence(const PolygonType &polygon)
        : Geofence<T>(polygon) {}

    constexpr explicit TestGeofence() = default;

    FRIEND_TEST(Geofence, DifferentTypes);
    FRIEND_TEST(GeofenceConstructor, Default);

    FRIEND_TEST(GeofenceIsEqual, DifferentTypes);
    FRIEND_TEST(TestAlmostEqualFinite, EqualNonFinite);
    FRIEND_TEST(TestEqualNonFinite, EqualNonFinite);
};

}  // namespace fcc_bridge::test::geofence

#endif  // THI_DRONE_WS_TEST_GEOFENCE_HPP
