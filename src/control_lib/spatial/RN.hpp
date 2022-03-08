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

#ifndef CONTROLLIB_SPATIAL_RN_HPP
#define CONTROLLIB_SPATIAL_RN_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <size_t N>
        struct RN {
            /* Init via translation and orientation */
            RN(const Eigen::Matrix<double, N, 1>& pos) : _pos(pos) {}

            RN() = default;

            /* Space elements difference */
            Eigen::Matrix<double, N, 1> operator-(RN const& obj) const { return _pos - obj._pos; }

            /* Space dimension */
            constexpr static size_t dimension() { return N; }

            /* Tangent and contagent plane elements (optionals) */
            Eigen::Matrix<double, N, 1> _pos, _vel, _acc, _eff;
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_RN_HPP