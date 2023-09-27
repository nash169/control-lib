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
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,SO
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef CONTROLLIB_SPATIAL_SO_HPP
#define CONTROLLIB_SPATIAL_SO_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <int N = Eigen::Dynamic, bool Left = false>
        struct SO {
            /* Init via translation and orientation */
            SO(const Eigen::Matrix<double, N, N>& rot) : _rot(rot) {}

            /* Default constructor */
            SO() = default;

            /* Space dimension */
            constexpr static size_t dimension() { return N * (N - 1) / 2; }

            /* Space elements difference */
            Eigen::Vector3d operator-(SO const& obj) const
            {
                if constexpr (Left)
                    // Left operation yields a tangent vector in the lie algebra
                    return obj.actInvLeft(*this).logarithm();
                else
                    // Right operation yields a tangent vector in the tangent space of the point itself
                    return obj.actInvRight(*this).logarithm();
            }

            /* rotation */
            Eigen::Matrix<double, N, N> _rot;

            /* Tangent (_t = velocity, _tt = acceleration) and cotangent space (_ct = effort) elements (optionals) */
            Eigen::Matrix<double, N, 1> _v, _a, _f;

            SO act(const SO& rot) const { return SO(Eigen::Matrix3d(_rot * rot._rot)); }

            SO actInvRight(const SO& rot) const { return SO(_rot.transpose() * rot._rot); }
            SO actInvLeft(const SO& rot) const { return SO(rot._rot * _rot.transpose()); }

            Eigen::Matrix<double, N, 1> logarithm(const SO& rot) const
            {
                return Eigen::VectorXd::Zero(N);
            }

            Eigen::Matrix<double, N, 1> logarithm() { return logarithm(*this); }
        };

        // template <>
        // struct SO<3> {
        //     /* Init via vector representation */
        //     SO(const Eigen::Vector3d& omega) : _rot(Eigen::AngleAxisd(omega.tail(3).norm(), omega.tail(3).normalized())) {}
        // };

        template <>
        Eigen::Vector3d SO<3>::logarithm(const SO& rot) const
        {
            Eigen::AngleAxisd aa(rot._rot);

            return aa.angle() * aa.axis();
        }

        template <>
        Eigen::Vector3d SO<3, true>::logarithm(const SO& rot) const
        {
            Eigen::AngleAxisd aa(rot._rot);

            return aa.angle() * aa.axis();
        }

    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SO_HPP