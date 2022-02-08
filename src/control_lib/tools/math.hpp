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

#include <Eigen/Geometry>

namespace control_lib {
    namespace tools {
        inline Eigen::Vector3d eulerError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
            Eigen::Vector3d error = curr - ref;

            for (size_t i = 0; i < 3; i++) {
                if (error(i) > M_PI)
                    error(i) -= 2 * M_PI;
                else if (error(i) < -M_PI)
                    error(i) += 2 * M_PI;
            }

            return error;
        }

        inline Eigen::Vector3d rotationError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
            Eigen::Matrix3d R_current = Eigen::AngleAxisd(curr.norm(), curr.normalized()).toRotationMatrix(),
                            R_desired = Eigen::AngleAxisd(ref.norm(), ref.normalized()).toRotationMatrix();

            Eigen::AngleAxisd aa = Eigen::AngleAxisd(R_desired.transpose() * R_current);

            return aa.axis() * aa.angle();
        }

        inline Eigen::Vector4d quaternionError(const Eigen::Vector4d& curr, const Eigen::Vector4d& ref)
        {
            Eigen::Quaterniond q_current = Eigen::Quaterniond(curr), q_desired = Eigen::Quaterniond(ref);
            return (q_current.inverse() * q_desired).coeffs();
        }

        inline Eigen::MatrixXd kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
        {
            Eigen::MatrixXd C(A.rows() * B.rows(), A.cols() * B.cols());

            for (size_t i = 0; i < A.rows(); i++) {
                for (size_t j = 0; j < A.cols(); j++)
                    C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
            }

            return C;
        }

        Eigen::MatrixXd solveVectorized(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);

        Eigen::MatrixXd bartelsStewart(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);
    } // namespace tools
} // namespace control_lib

#endif // CONTROLLIB_TOOLS_MATH_HPP