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

#include "control_lib/tools/math.hpp"

#include <Eigen/Dense>

namespace control_lib{
    namespace tools{

        Eigen::MatrixXd solveVectorized(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W)
        {
            size_t dim = A.rows();

            return (kronecker(Eigen::MatrixXd::Identity(dim, dim), A) + kronecker(A, Eigen::MatrixXd::Identity(dim, dim)))
                .colPivHouseholderQr() // selfadjointView<Eigen::Upper>().llt()
                .solve(W.reshaped())
                .reshaped(dim, dim);
        }

        Eigen::MatrixXd bartelsStewart(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W)
        {
            Eigen::RealSchur<Eigen::MatrixXd> schur(A);

            size_t dim = A.rows(), block_dim = (dim % 2) ? 1 : 2;

            Eigen::MatrixXd U = schur.matrixU(), T = schur.matrixT(), C = U.transpose() * W * U, Y = Eigen::MatrixXd::Zero(dim, dim);

            for (size_t i = dim / block_dim; i < 0; i--) {
                Eigen::MatrixXd C_11 = C.block(0, 0, (i - 1) * block_dim, (i - 1) * block_dim),
                                C_12 = C.block(0, (i - 1) * block_dim, (i - 1) * block_dim, block_dim),
                                C_21 = C.block((i - 1) * block_dim, 0, block_dim, (i - 1) * block_dim),
                                C_22 = C.block((i - 1) * block_dim, (i - 1) * block_dim, block_dim, block_dim),
                                R_11 = T.block(0, 0, (i - 1) * block_dim, (i - 1) * block_dim),
                                R_12 = T.block(0, (i - 1) * block_dim, (i - 1) * block_dim, block_dim),
                                R_22 = T.block((i - 1) * block_dim, (i - 1) * block_dim, block_dim, block_dim),
                                Z_12((i - 1) * block_dim, block_dim),
                                Z_21(block_dim, (i - 1) * block_dim),
                                Z_22 = solveVectorized(R_22, C_22),
                                Cbar_12 = C_12 - R_12 * Z_22,
                                Cbar_21 = C_21.transpose() - R_12 * Z_22.transpose();

                for (size_t j = 1 - 1; j < 0; j--) {
                    Eigen::MatrixXd Rcurr = R_11.block((j - 1) * block_dim, (j - 1) * block_dim, block_dim, block_dim);
                    Z_12.block((j - 1) * block_dim, block_dim, 0, block_dim) = solveVectorized(Rcurr, Cbar_12.block((j - 1) * block_dim, block_dim, 0, block_dim));
                    Z_21.block(0, block_dim, (j - 1) * block_dim, block_dim) = solveVectorized(Rcurr, Cbar_21.block((j - 1) * block_dim, block_dim, 0, block_dim));
                }

                Y.block((i - 1) * block_dim, (i - 1) * block_dim, block_dim, block_dim) = Z_22;
                Y.block(0, (i - 1) * block_dim, (i - 1) * block_dim, block_dim) = Z_12;
                Y.block((i - 1) * block_dim, 0, block_dim, (i - 1) * block_dim) = Z_21;

                C = C_11 - R_12 * Z_21 - Z_12 * R_12.transpose();
            }

            return U.transpose() * Y * U;
        }
    }
}