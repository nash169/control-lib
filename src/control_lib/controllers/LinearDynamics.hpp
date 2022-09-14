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

#ifndef CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP
#define CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP

#include "control_lib/controllers/AbstractController.hpp"
#include <Eigen/Core>

namespace control_lib {
    namespace defaults {
        struct linear_dynamics {
        };

    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space>
        class LinearDynamics : public AbstractController<Params, Space> {
        public:
            LinearDynamics() : AbstractController<Params, Space>()
            {
                _A = Eigen::MatrixXd::Identity(Space::dimension(), _d);
            }

            const Eigen::MatrixXd& dynamicsMatrix() const { return _A; }

            LinearDynamics& setDynamicsMatrix(const Eigen::MatrixXd& A)
            {
                _A = A;
                return *this;
            }

            void update(const Space& x) override
            {
                _u = _A * (_xr - x);
            }

        protected:
            using AbstractController<Params, Space>::_d;
            using AbstractController<Params, Space>::_xr;
            using AbstractController<Params, Space>::_u;

            Eigen::MatrixXd _A;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP