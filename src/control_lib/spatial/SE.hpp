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

#ifndef CONTROLLIB_SPATIAL_SE_HPP
#define CONTROLLIB_SPATIAL_SE_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <size_t N, bool Left = false>
        struct SE {
            /* Init via translation and orientation */
            SE(const Eigen::Matrix<double, N, N>& rot, const Eigen::Matrix<double, N, 1>& trans) : _rot(rot), _trans(trans) {}

            /* Default constructor */
            SE() = default;

            /* Space elements difference (the right operator gives a vector in the tangent space attached to the current point) */
            Eigen::Matrix<double, 2 * N, 1> operator-(SE const& obj) const
            {
                if constexpr (Left == true)
                    return obj.actInvLeft(*this).logarithm();
                else
                    return obj.actInvRight(*this).logarithm();
            }

            /* Space dimension */
            constexpr static size_t dimension() { return 2 * N; }

            /* Translation & rotation */
            Eigen::Matrix<double, N, 1> _trans;
            Eigen::Matrix<double, N, N> _rot;

            /* Tangent (_t = velocity, _tt = acceleration) and cotangent space (_ct = effort) elements (optionals) */
            Eigen::Matrix<double, 2 * N, 1> _v, _a, _f;

            SE act(const SE& pose) const { return SE(_rot * pose._rot, _trans + _rot * pose._trans); } // to check

            SE actInvRight(const SE& pose) const { return SE(_rot.transpose() * pose._rot, _rot.transpose() * (pose._trans - _trans)); }
            SE actInvLeft(const SE& pose) const { return SE(pose._rot * _rot.transpose(), pose._trans - pose._rot * _rot.transpose() * _trans); } // to check

            Eigen::Matrix<double, 2 * N, 1> logarithm(const SE& pose)
            {
                return Eigen::VectorXd::Zero(2 * N);
            }

            Eigen::Matrix<double, 2 * N, 1> logarithm() { return logarithm(*this); }
        };

        // template <>
        // struct SE<3> {
        //     /* Init via vector representation */
        //     SE(const Eigen::Matrix<double, 6, 1>& x) : _trans(x.head(3)), _rot(Eigen::AngleAxisd(x.tail(3).norm(), x.tail(3).normalized())) {}
        // };

        template <>
        Eigen::Matrix<double, 6, 1> SE<3>::logarithm(const SE<3>& pose)
        {
            // cartesian space representation omega
            Eigen::AngleAxisd aa(pose._rot);
            Eigen::Vector3d omega = aa.angle() * aa.axis();

            // lie algebra representation omega
            Eigen::Matrix3d omega_skew;
            omega_skew << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0;
            double theta = omega.norm(), A = std::sin(theta) / theta, B = (1 - std::cos(theta)) / std::pow(theta, 2);

            // cartesian representation [V^-1 x t, omega]
            return (Eigen::Matrix<double, 6, 1>() << (Eigen::Matrix3d::Identity() - 0.5 * omega_skew + (1 - 0.5 * A / B) / std::pow(theta, 2) * omega_skew * omega_skew) * pose._trans, omega).finished();
        }
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SE_HPP