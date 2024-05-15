//
// Created by Johan <job8197@thi.de> on 12.05.2024.
//

/*
 * Test based on https://github.com/chrberger/geofence/tree/master/test
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

#include <limits>

#include "test_header.hpp"

template <typename T>
class test_geofence : protected fcc_bridge::Geofence<T> {
    FRIEND_TEST(Geofence, Equality);
};

using FloatType = float;

const FloatType LOW = std::numeric_limits<FloatType>::lowest();

const FloatType MIN = std::numeric_limits<FloatType>::min();

const FloatType MAX = std::numeric_limits<FloatType>::max();

const FloatType EPS = std::numeric_limits<FloatType>::epsilon();

const FloatType INF = std::numeric_limits<FloatType>::infinity();

const FloatType QNaN = std::numeric_limits<FloatType>::quiet_NaN();

const FloatType SNaN = std::numeric_limits<FloatType>::signaling_NaN();

const FloatType DNM = std::numeric_limits<FloatType>::denorm_min();

using FloatGeofence = fcc_bridge::Geofence<FloatType>;

using AlmostEqualParamType = std::pair<FloatType, FloatType>;

class TestAlmostEqualFinite
    : public testing::TestWithParam<AlmostEqualParamType>,
      FloatGeofence {
   public:
    static std::vector<AlmostEqualParamType> values;
    static void SetUpTestSuite() {
        throw std::runtime_error("Shit!");
        TestAlmostEqualFinite::values.insert(
            TestAlmostEqualFinite::values.end(),
            {AlmostEqualParamType{LOW, LOW},
             AlmostEqualParamType{LOW, LOW + EPS},
             AlmostEqualParamType{MIN, MIN},
             AlmostEqualParamType{MIN, MIN + EPS},
             AlmostEqualParamType{MAX, MAX},
             AlmostEqualParamType{MAX, MAX - EPS}});
        if (std::numeric_limits<FloatType>::has_denorm) {
            TestAlmostEqualFinite::values.insert(
                TestAlmostEqualFinite::values.end(),
                {AlmostEqualParamType{DNM, DNM},
                 AlmostEqualParamType{DNM, DNM + EPS}});
        } else {
            RCLCPP_WARN(::fcc_bridge_test::TEST_LOGGER,
                        "Platform has no denormal floats!");
        }
    }
};

std::vector<AlmostEqualParamType> TestAlmostEqualFinite::values;

using EqualNonFiniteParamType = std::pair<FloatType, FloatType>;

class TestEqualNonFinite
    : public testing::TestWithParam<EqualNonFiniteParamType>,
      FloatGeofence {
   public:
    static std::vector<EqualNonFiniteParamType> values;
    static void SetUpTestSuite() {
        if (std::numeric_limits<FloatType>::has_infinity) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                {EqualNonFiniteParamType{INF, 0},
                 EqualNonFiniteParamType{-INF, 0},
                 EqualNonFiniteParamType{INF, 1},
                 EqualNonFiniteParamType{-INF, 1},
                 EqualNonFiniteParamType{INF, -1},
                 EqualNonFiniteParamType{-INF, -1},
                 EqualNonFiniteParamType{INF, -INF}});
        } else {
            RCLCPP_WARN(::fcc_bridge_test::TEST_LOGGER,
                        "Platform has no float infinitives!");
        }
        if (std::numeric_limits<FloatType>::has_quiet_NaN) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                {EqualNonFiniteParamType{QNaN, 0},
                 EqualNonFiniteParamType{QNaN, 1},
                 EqualNonFiniteParamType{QNaN, -1}});
        } else {
            RCLCPP_WARN(::fcc_bridge_test::TEST_LOGGER,
                        "Platform has no quiet float NaNs!");
        }
        if (std::numeric_limits<FloatType>::has_signaling_NaN) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                {EqualNonFiniteParamType{SNaN, 0},
                 EqualNonFiniteParamType{SNaN, 1},
                 EqualNonFiniteParamType{SNaN, -1}});
        } else {
            RCLCPP_WARN(::fcc_bridge_test::TEST_LOGGER,
                        "Platform has no signaling float NaNs!");
        }
        if (std::numeric_limits<FloatType>::has_infinity &&
            std::numeric_limits<FloatType>::has_quiet_NaN) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                {EqualNonFiniteParamType{QNaN, INF},
                 EqualNonFiniteParamType{QNaN, -INF}});
        }
        if (std::numeric_limits<FloatType>::has_infinity &&
            std::numeric_limits<FloatType>::has_signaling_NaN) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                {EqualNonFiniteParamType{SNaN, INF},
                 EqualNonFiniteParamType{SNaN, -INF}});
        }
        if (std::numeric_limits<FloatType>::has_quiet_NaN &&
            std::numeric_limits<FloatType>::has_signaling_NaN) {
            TestEqualNonFinite::values.insert(
                TestEqualNonFinite::values.end(),
                EqualNonFiniteParamType{QNaN, SNaN});
        }
    }
};

std::vector<EqualNonFiniteParamType> TestEqualNonFinite::values;

TEST(Geofence, Equality) {
    EXPECT_TRUE(test_geofence<uint16_t>::isEqual(15, 15));
    EXPECT_FALSE(test_geofence<int16_t>::isEqual(15, -15));
    EXPECT_FALSE(test_geofence<float>::isEqual(0, 0.001f));
    EXPECT_TRUE(test_geofence<float>::isEqual(0.0009f, 0.0009f));
    EXPECT_TRUE(test_geofence<double>::isEqual(static_cast<double>(0.0000009f),
                                               static_cast<double>(0.0000009)));
}

TEST_P(TestAlmostEqualFinite, EqualNonFinite) {
    const AlmostEqualParamType &param = TestAlmostEqualFinite::GetParam();
    EXPECT_TRUE(FloatGeofence::isEqual(param.first, param.second));
    EXPECT_TRUE(FloatGeofence::isEqual(param.second, param.first));
}

INSTANTIATE_TEST_SUITE_P(, TestAlmostEqualFinite,
                         testing::ValuesIn(TestAlmostEqualFinite::values));

TEST_P(TestEqualNonFinite, EqualNonFinite) {
    const EqualNonFiniteParamType &param = TestEqualNonFinite::GetParam();
    FloatGeofence::isEqual(param.first, param.second);
    FloatGeofence::isEqual(param.second, param.first);
}

INSTANTIATE_TEST_SUITE_P(, TestEqualNonFinite,
                         testing::ValuesIn(TestEqualNonFinite::values));

/*
TEST_CASE("non-poplygon returns false") {
    std::vector<std::array<int, 2>> polygon;
    std::array<int, 2> point{1, 2};
    CHECK(!geofence::isIn<int>(polygon, point));
}
*/