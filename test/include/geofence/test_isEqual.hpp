//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_TEST_ISEQUAL_HPP
#define THI_DRONE_WS_TEST_ISEQUAL_HPP

#include <gtest/gtest.h>

#include <limits>

namespace fcc_bridge::test::geofence {

using FloatType = double;

const FloatType LOW = std::numeric_limits<FloatType>::lowest();

const FloatType MIN = std::numeric_limits<FloatType>::min();

const FloatType MAX = std::numeric_limits<FloatType>::max();

const FloatType EPS = std::numeric_limits<FloatType>::epsilon();

const FloatType INF = std::numeric_limits<FloatType>::infinity();

const FloatType QNaN = std::numeric_limits<FloatType>::quiet_NaN();

const FloatType SNaN = std::numeric_limits<FloatType>::signaling_NaN();

const FloatType DNM = std::numeric_limits<FloatType>::denorm_min();

using AlmostEqualParamType = std::pair<FloatType, FloatType>;

class TestAlmostEqualFinite
    : public testing::TestWithParam<AlmostEqualParamType> {
   public:
    static std::vector<AlmostEqualParamType> values;
    static void setup();
};

using EqualNonFiniteParamType = std::pair<FloatType, FloatType>;

class TestEqualNonFinite
    : public testing::TestWithParam<EqualNonFiniteParamType> {
   public:
    static std::vector<EqualNonFiniteParamType> values;
    static void setup();
};

}  // namespace fcc_bridge::test::geofence

#endif  // THI_DRONE_WS_TEST_ISEQUAL_HPP
