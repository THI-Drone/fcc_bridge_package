//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

/*
 * Tests based on https://github.com/chrberger/geofence/tree/master/test
 */

/*
 * MIT License
 *
 * Copyright (c) 2018  Christian Berger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "geofence/test_constructor.hpp"

namespace fcc_bridge::test::geofence {

using GeofenceConstructorTypes =
    testing::Types<int8_t, int16_t, int32_t, int64_t, float, double,
                   long double>;

TYPED_TEST_SUITE_P(GeofenceConstructor);

// Check the default constructor
TYPED_TEST_P(GeofenceConstructor, Default) {
    TestGeofence<TypeParam> local_geofence;
    EXPECT_THROW(local_geofence.isIn({0, 0}), invalid_polygon_error);
    EXPECT_THROW(local_geofence.isIn({2, 123}), invalid_polygon_error);
}

// Check the constructor with only one point
TYPED_TEST_P(GeofenceConstructor, OnePoint) {
    EXPECT_THROW((TestGeofence<TypeParam>{{0, 0}}), invalid_polygon_error);
    if constexpr (std::is_floating_point_v<TypeParam>) {
        if constexpr (std::numeric_limits<TypeParam>::has_infinity) {
            constexpr TypeParam INF =
                std::numeric_limits<TypeParam>::infinity();
            EXPECT_THROW((TestGeofence<TypeParam>{{INF, 0}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{-INF, 1}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{.3f, INF}}),
                         invalid_polygon_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_quiet_NaN) {
            constexpr TypeParam QNaN =
                std::numeric_limits<TypeParam>::quiet_NaN();
            EXPECT_THROW((TestGeofence<TypeParam>{{QNaN, 0}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{.3f, QNaN}}),
                         invalid_polygon_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_signaling_NaN) {
            constexpr TypeParam SNaN =
                std::numeric_limits<TypeParam>::signaling_NaN();
            EXPECT_THROW((TestGeofence<TypeParam>{{SNaN, 0}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{.3f, SNaN}}),
                         invalid_polygon_error);
        }
    }
}

// Check the constructor with two points
TYPED_TEST_P(GeofenceConstructor, TwoPoints) {
    EXPECT_THROW((TestGeofence<TypeParam>{{0, 0}, {0, 0}}),
                 invalid_polygon_error);
    if constexpr (std::is_floating_point_v<TypeParam>) {
        if constexpr (std::numeric_limits<TypeParam>::has_infinity) {
            constexpr TypeParam INF =
                std::numeric_limits<TypeParam>::infinity();
            EXPECT_THROW((TestGeofence<TypeParam>{{{INF, 0}, {0, INF}}}),
                         invalid_polygon_error);
            EXPECT_THROW(
                (TestGeofence<TypeParam>{{132.323f, 8723.3238f}, {-INF, 1}}),
                invalid_polygon_error);
            EXPECT_THROW(
                (TestGeofence<TypeParam>{{.3f, INF}, {-INF, .000023f}}),
                invalid_polygon_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_quiet_NaN) {
            constexpr TypeParam QNaN =
                std::numeric_limits<TypeParam>::quiet_NaN();
            EXPECT_THROW((TestGeofence<TypeParam>{{.223f, 834.23f}, {QNaN, 0}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{QNaN, 33}, {.3f, QNaN}}),
                         invalid_polygon_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_signaling_NaN) {
            constexpr TypeParam SNaN =
                std::numeric_limits<TypeParam>::signaling_NaN();
            EXPECT_THROW((TestGeofence<TypeParam>{{65323.34f, 533.844f},
                                                  {SNaN, 0.003434f}}),
                         invalid_polygon_error);
            EXPECT_THROW((TestGeofence<TypeParam>{{.3f, SNaN}, {.343f, SNaN}}),
                         invalid_polygon_error);
        }
    }
}

// Check the constructor with three points
TYPED_TEST_P(GeofenceConstructor, ThreePoints) {
    EXPECT_NO_THROW((TestGeofence<TypeParam>{{0, 0}, {0, 1}, {1, 0}}));
    EXPECT_NO_THROW((TestGeofence<TypeParam>{{0, 0}, {0, 0}, {1, 1}}));
    EXPECT_NO_THROW((TestGeofence<TypeParam>{{0, 0}, {1, 1}, {0, 0}}));
    EXPECT_NO_THROW((TestGeofence<TypeParam>{{1, 1}, {0, 0}, {0, 0}}));
    EXPECT_NO_THROW((TestGeofence<TypeParam>{{0, 0}, {0, 0}, {0, 0}}));
    TestGeofence<TypeParam> geofence{{0, 0}, {2, 0}, {1, 2}};
    EXPECT_TRUE(geofence.isIn({1, 1}));
    EXPECT_FALSE(geofence.isIn({2, 2}));
    if constexpr (std::is_floating_point_v<TypeParam>) {
        if constexpr (std::numeric_limits<TypeParam>::has_infinity) {
            constexpr TypeParam INF =
                std::numeric_limits<TypeParam>::infinity();
            EXPECT_THROW((TestGeofence<TypeParam>{{{INF, 0}, {0, 0}, {0, 0}}}),
                         invalid_point_error);
            EXPECT_THROW((TestGeofence<TypeParam>{
                             {132.323f, 8723.3238f}, {-INF, 1}, {INF, .323f}}),
                         invalid_point_error);
            EXPECT_THROW(
                (TestGeofence<TypeParam>{
                    {.3f, 232.233f}, {-232.2f, -323.455f}, {-INF, .000023f}}),
                invalid_point_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_quiet_NaN) {
            constexpr TypeParam QNaN =
                std::numeric_limits<TypeParam>::quiet_NaN();
            EXPECT_THROW((TestGeofence<TypeParam>{
                             {QNaN, 834.23f}, {QNaN, 0}, {323, 123}}),
                         invalid_point_error);
            EXPECT_THROW(
                (TestGeofence<TypeParam>{{23, 33}, {.3f, QNaN}, {3, 2}}),
                invalid_point_error);
            EXPECT_THROW(
                (TestGeofence<TypeParam>{{23, 33}, {.3f, .323f}, {QNaN, 2}}),
                invalid_point_error);
        }
        if constexpr (std::numeric_limits<TypeParam>::has_signaling_NaN) {
            constexpr TypeParam SNaN =
                std::numeric_limits<TypeParam>::signaling_NaN();
            EXPECT_THROW(
                (TestGeofence<TypeParam>{
                    {4556, SNaN}, {65323.34f, 533.844f}, {SNaN, 0.003434f}}),
                invalid_point_error);
            EXPECT_THROW((TestGeofence<TypeParam>{
                             {232, 54}, {.3f, SNaN}, {.343f, SNaN}}),
                         invalid_point_error);
            EXPECT_THROW((TestGeofence<TypeParam>{
                             {232, 54}, {.3f, 3343}, {SNaN, .343f}}),
                         invalid_point_error);
        }
    }
}

REGISTER_TYPED_TEST_SUITE_P(GeofenceConstructor, Default, OnePoint, TwoPoints,
                            ThreePoints);

INSTANTIATE_TYPED_TEST_SUITE_P(, GeofenceConstructor, GeofenceConstructorTypes);

}  // namespace fcc_bridge::test::geofence
