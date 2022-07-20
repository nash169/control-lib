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

#ifndef CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP
#define CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP

#include <functional>
#include <vector>

#include <optimization_lib/QpoasesOptimizer.hpp>

#include "control_lib/controllers/AbstractController.hpp"
#include "control_lib/spatial/RN.hpp"

namespace control_lib {
    namespace defaults {
        struct quadratic_programming {
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space = spatial::RN<3>>
        class QuadraticProgramming : public AbstractController<Params, Space> {
        public:
            QuadraticProgramming() {}

            QuadraticProgramming& setCostAcceleration(const Eigen::MatrixXd& Q)
            {
                _Q = Q;
                return *this;
            }

            QuadraticProgramming& setCostEffort(const Eigen::MatrixXd& R)
            {
                _R = R;
                return *this;
            }

            void update(const Space& x) override
            {
            }

        protected:
            Eigen::MatrixXd _Q, _R;

            std::vector<std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>> _A;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP