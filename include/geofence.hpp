/*
 * See here: https://github.com/chrberger/geofence/blob/master/geofence.hpp
 *
 * Modifications by Johan <job8197@thi.de>
 */

/*
 * MIT License
 *
 * Copyright (c) 2020  Christian Berger
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

/*
 * Copyright (c) 1970-2003, Wm. Randolph Franklin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimers. Redistributions in binary
 * form must reproduce the above copyright notice in the documentation and/or
 * other materials provided with the distribution. The name of W. Randolph
 * Franklin may not be used to endorse or promote products derived from this
 * Software without specific prior written permission. THE SOFTWARE IS PROVIDED
 * "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef GEOFENCE_HPP
#define GEOFENCE_HPP

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#if (math_errhandling & MATH_ERRNO) != MATH_ERRNO
#error The geofence libary requires floating point math errors to set ERRNO
#endif

namespace fcc_bridge {

/**
 * @brief Class to signal that the polygon of a geofence is invalid
 */
class invalid_polygon_error : public std::runtime_error {
   public:
    /**
     * @brief Constructs a invalid_polygon_error
     *
     * @param msg The error message to convey
     */
    explicit invalid_polygon_error(const std::string &msg)
        : std::runtime_error(msg) {

          };
};

/**
 * @brief Class to signal that a point is invalid
 */
class invalid_point_error : public std::runtime_error {
   public:
    /**
     * @brief Constructs a invalid_point_error
     *
     * @param msg The error message to convey
     */
    explicit invalid_point_error(const std::string &msg)
        : std::runtime_error(msg) {

          };
};

/**
 * @brief This class defines a geofence and allows checking if a given point is
 * inside it.
 *
 * @tparam T The type of a coordinate axis value. Must be arithmetic.
 */
template <typename T>
class Geofence {
    static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

    static_assert(!(std::is_integral_v<T> && std::is_unsigned_v<T>),
                  "For integral types only signed types are allowed");

   public:
    using PointType = std::array<T, 2>; /**< The type of a single point
                                           {Latitude_deg, Longitude_deg} */

    using PolygonType =
        std::vector<PointType>; /**< The type of the polygon used */

   private:
    const PolygonType convex_polygon; /**< The convex polygon that hold the
                                         point of the geofence*/

    constexpr static uint8_t X{0};
    constexpr static uint8_t Y{1};

   public:
    /**
     * @brief Constructs a geofence object
     *
     * @param polygon The polygon to use for the geofence. Will be turned into a
     * convex polygon
     *
     * @throws invalid_point_error If the polygon contains any point that is not
     * a finite float. Only possible if std::is_floating_point_v<T> is true
     */
    constexpr explicit Geofence(const PolygonType &polygon)
        : convex_polygon(Geofence::getConvexHull(polygon)) {}

    /**
     * @brief Construct an empty geofence object
     */
    constexpr Geofence() : convex_polygon{} {}

    /**
     * @brief Gets the current count of points of the polygon
     *
     * @return The amount of points in the polygon
     */
    inline typename PolygonType::size_type get_polygon_point_count() const {
        return this->convex_polygon.size();
    }

   protected:
    /**
     * @brief Test if to values a and b are close enough to each other to be
     * considered equal
     *
     * @param a The first value
     * @param b The second value
     *
     * @return true if a and b are (almost - in case of floating points) equal
     *
     * @throws invalid_point_error If either a or b are not finite. Only
     * possible if std::is_floating_point_v<T> is true
     */
    constexpr static bool isEqual(const T a, const T b) {
        // Inspired by:
        // https://www.embeddeduse.com/2019/08/26/qt-compare-two-floats/

#pragma GCC diagnostic push
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wabsolute-value"
#endif

        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(a) || !std::isfinite(b)) {
                throw invalid_point_error(
                    "Got a non finite float to check for equality");
            }
            constexpr T EPSILON = 1.0e-09f;
            return (std::abs(a - b) <= EPSILON)
                       ? true
                       : std::abs(a - b) <=
                             EPSILON * std::max(std::abs(a), std::abs(b));
        } else {
            return a == b;
        }

#pragma GCC diagnostic pop
    }

    /**
     * @brief Compute convex hull using Andrew's monotone chain algorithm.
     *
     * @param polygon The polygon to make convex
     *
     * @return The convex polygon
     *
     * @throws invalid_polygon_error If the polygon contains less than 3 points
     * or a computation on the points results in an overflow or otherwise
     * invalid behaviour
     *
     * @throws invalid_point_error If the polygon contains any point that is not
     * a finite float. Only possible if std::is_floating_point_v<T> is true
     */
    constexpr static PolygonType getConvexHull(const PolygonType &polygon) {
        // Inspired by:
        // https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C++

        std::function<bool(const PointType &, const PointType &)> isLeft;

        std::function<T(const PointType &, const PointType &,
                        const PointType &)>
            ccw;

        if (polygon.size() < 3) {
            throw invalid_polygon_error(
                "The non convex polygon needs at least 3 points");
        }
        if constexpr (std::is_floating_point_v<T>) {
            for (const PointType &point : polygon) {
                if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
                    throw invalid_point_error(
                        "Got an non finite float in a point");
                }
            }

            isLeft = [](const PointType &a, const PointType &b) {
                return (a[X] < b[X] || (!(b[X] < a[X]) && a[Y] < b[Y]));
            };

            ccw = [](const PointType &a, const PointType &b,
                     const PointType &c) -> T {
                errno = 0;
                const T res = (b[X] - a[X]) * (c[Y] - a[Y]) -
                              (b[Y] - a[Y]) * (c[X] - a[X]);
                if ((errno & (EDOM | ERANGE)) != 0) {
                    throw invalid_polygon_error(
                        "Computing the cross product resulted in invalid float "
                        "operations");
                }
                return res;
            };

        } else {
            isLeft = [](const PointType &a, const PointType &b) -> bool {
                return (a[X] < b[X] || ((a[X] == b[X]) && a[Y] < b[Y]));
            };

            ccw = [](const PointType &a, const PointType &b,
                     const PointType &c) -> T {
                /*
                 * return (b[X] - a[X]) * (c[Y] - a[Y]) - (b[Y] - a[Y]) * (c[X]
                 * - a[X]);
                 */
                T bax;    // b[X] - a[x]
                T cay;    // c[Y] - a[Y]
                T prod1;  // abx * cay
                T bay;    // b[Y] - a[Y]
                T cax;    // c[X] - a[X]
                T prod2;  // bay * cax
                T delta;  // prod1 - prod2
                if (__builtin_sub_overflow(b[X], a[X], &bax) ||
                    __builtin_sub_overflow(c[Y], a[Y], &cay) ||
                    __builtin_mul_overflow(bax, cay, &prod1) ||
                    __builtin_sub_overflow(b[Y], a[Y], &bay) ||
                    __builtin_sub_overflow(c[X], a[X], &cax) ||
                    __builtin_mul_overflow(bay, cax, &prod2) ||
                    __builtin_sub_overflow(prod1, prod2, &delta)) {
                    throw invalid_polygon_error(
                        "Computing the cross product resulted in an integer "
                        "overflow");
                }
                return delta;
            };
        }

        PolygonType sortedPolygon{polygon};
        std::sort(sortedPolygon.begin(), sortedPolygon.end(), isLeft);

        const typename PolygonType::size_type polygon_size{
            sortedPolygon.size()};
        typename PolygonType::size_type k{0};

        PolygonType convex_hull{polygon_size * 2};

        // Lower hull
        for (typename PolygonType::size_type i = 0; i < polygon_size; ++i) {
            while (k >= 2 && ccw(convex_hull[k - 2], convex_hull[k - 1],
                                 polygon[i]) <= 0) {
                k--;
            }
            convex_hull[k++] = polygon[i];
        }

        // Upper hull
        for (typename PolygonType::size_type i = polygon_size - 1, t = k + 1;
             i > 0; --i) {
            while (k >= t && ccw(convex_hull[k - 2], convex_hull[k - 1],
                                 polygon[i - 1]) <= 0) {
                k--;
            }
            convex_hull[k++] = polygon[i - 1];
        }

        convex_hull.resize(k - 1);
        return convex_hull;
    }

   public:
    /**
     * @brief Checks if the given point p is inside the geofence polygon
     *
     * @param point point to test whether inside or not
     *
     * @return true if p is inside the polygon OR when p is any vertex OR on an
     * edge of the convex hull
     *
     * @throws invalid_polygon_error if the polygon has less then 3 points
     * @throws invalid_point_error If the polygon contains any point that is not
     * a finite float. Only possible if std::is_floating_point_v<T> is true
     *
     */
    constexpr bool isIn(const PointType &point) const {
        // The algorithms is based on W. Randolph Franklin's implementation
        // that can be found here:
        // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html

        if (this->convex_polygon.size() < 3) {
            throw invalid_polygon_error(
                "Polygon has only " +
                std::to_string(this->convex_polygon.size()) + " points");
        }

        bool inside{false};
        const std::size_t POINTS{this->convex_polygon.size()};

        if constexpr (std::is_floating_point_v<T>) {
            if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
                throw invalid_point_error("Got an non finite float in a point");
            }
            errno = 0;
            for (std::size_t i{0}, j{POINTS - 1}; i < POINTS; j = i++) {
                const PointType &cur_point_i = this->convex_polygon[i];
                const PointType &cur_point_j = this->convex_polygon[j];
                if (((cur_point_i[Y] > point[Y]) !=
                     (cur_point_j[Y] > point[Y])) &&
                    (point[X] < (cur_point_j[X] - cur_point_i[X]) *
                                        (point[Y] - cur_point_i[Y]) /
                                        (cur_point_j[Y] - cur_point_i[Y]) +
                                    cur_point_i[X])) {
                    inside = !inside;
                }
            }
            if ((errno & (EDOM | ERANGE)) != 0) {
                throw invalid_polygon_error(
                    "There was an arithmetic error checking if the point is "
                    "inside the polygon");
            }
        } else {
            for (std::size_t i{0}, j{POINTS - 1}; i < POINTS; j = i++) {
                const PointType &cur_point_i = this->convex_polygon[i];
                const PointType &cur_point_j = this->convex_polygon[j];
                if ((cur_point_i[Y] > point[Y]) !=
                    (cur_point_j[Y] > point[Y])) {
                    // Transformed equation to get rid of the division
                    T cur_j_sub_i_x;  // cur_point_j[X] - cur_point_i[X]
                    T p_sub_cur_i_y;  // point[Y] - cur_point_i[Y]
                    T prod1;          // (cur_point_j[X] - cur_point_i[X]) *
                                      // (point[Y] - cur_point_i[Y])
                    T cur_j_sub_i_y;  // cur_point_j[Y] - cur_point_i[Y]
                    T p_sub_cur_i_x;  // point[X] - cur_point_i[X]
                    T prod2;  // (point[X] - cur_point_i[X]) * (cur_point_j[Y] -
                              // cur_point_i[Y])

                    if (__builtin_sub_overflow(cur_point_j[X], cur_point_i[X],
                                               &cur_j_sub_i_x) ||
                        __builtin_sub_overflow(point[Y], cur_point_i[Y],
                                               &p_sub_cur_i_y) ||
                        __builtin_mul_overflow(cur_j_sub_i_x, p_sub_cur_i_y,
                                               &prod1) ||
                        __builtin_sub_overflow(cur_point_j[Y], cur_point_i[Y],
                                               &cur_j_sub_i_y) ||
                        __builtin_sub_overflow(point[X], cur_point_i[X],
                                               &p_sub_cur_i_x) ||
                        __builtin_mul_overflow(p_sub_cur_i_x, cur_j_sub_i_y,
                                               &prod2)) {
                        throw invalid_polygon_error(
                            "There was an arithmetic error checking if the "
                            "point is inside the polygon");
                    }
                    if (0 < cur_j_sub_i_y) {
                        if (prod2 < prod1) {
                            inside = !inside;
                        }
                    } else if (cur_j_sub_i_y < 0) {
                        // Switch inequality if original divisor is negative
                        if (prod2 > prod1) {
                            inside = !inside;
                        }
                    } else {
                        throw invalid_polygon_error(
                            "There was an arithmetic error checking if the "
                            "point is inside the polygon");
                    }
                }
            }
        }

        return inside;
    }
};

}  // namespace fcc_bridge

#endif
