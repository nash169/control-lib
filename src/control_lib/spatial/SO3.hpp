/*
    This file is part of control-lib.

    Copyright (c) 2020, 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef CONTROLLIB_SPATIAL_SO3_HPP
#define CONTROLLIB_SPATIAL_SO3_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        struct SO3 {
            /* Init via translation and orientation */
            SO3(const Eigen::Matrix3d& rot) : _rot(rot) {}

            /* Init via vector representation */
            SO3(const Eigen::Vector3d& omega) : _rot(Eigen::AngleAxisd(omega.tail(3).norm(), omega.tail(3).normalized())) {}

            /* Default constructor */
            SO3() = default;

            /* Space elements difference */
            Eigen::Vector3d operator-(SO3 const& obj) const { return obj.actionInverse(*this).log(); }

            /* Space dimension */
            constexpr static size_t dimension() { return 3; }

            /* rotation */
            Eigen::Matrix3d _rot;

            /* Tangent and contagent plane elements (optionals) */
            Eigen::Vector3d _vel, _acc, _eff;

        protected:
            SO3 action(const SO3& rot) const { return SO3(Eigen::Matrix3d(_rot * rot._rot)); }

            SO3 actionInverse(const SO3& rot) const { return SO3(Eigen::Matrix3d(_rot.transpose() * rot._rot)); }

            Eigen::Vector3d log(const SO3& rot) const
            {
                Eigen::AngleAxisd aa(rot._rot);

                return aa.angle() * aa.axis();
            }

            Eigen::Vector3d log() { return log(*this); }
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SO3_HPP