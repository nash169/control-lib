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

#ifndef CONTROLLIB_TOOLS_MATH_HPP
#define CONTROLLIB_TOOLS_MATH_HPP

#include <Eigen/Core>
namespace control_lib {
    namespace tools {
        inline Eigen::Vector3d eulerError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref);

        inline Eigen::Vector3d rotationError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref);

        inline Eigen::Vector4d quaternionError(const Eigen::Vector4d& curr, const Eigen::Vector4d& ref);

        inline Eigen::MatrixXd kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

        Eigen::MatrixXd solveVectorized(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);

        Eigen::MatrixXd bartelsStewart(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);
    } // namespace tools
} // namespace control_lib

#endif // CONTROLLIB_TOOLS_MATH_HPP