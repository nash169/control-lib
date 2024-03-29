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

#ifndef CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP
#define CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP

#include <Eigen/Core>

#include "control_lib/tools/macros.hpp"

namespace control_lib {
    namespace defaults {
        struct controller {
            // Time step controller
            PARAM_SCALAR(double, dt, 1.0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space>
        class AbstractController {
        public:
            AbstractController() : _dt(Params::controller::dt()) {}

            // Output of dimension of the controller (the input dimension is automatically defined by the dimension of Space)
            const size_t& dimension() const { return _d; };

            // Controller time step (this is used in different ways depending on the controller type)
            const double& step() const { return _dt; };

            // Controller Target (this must be an element of Space)
            const Space& reference() const { return _xr; };

            // Set controller output dimension
            AbstractController& setDimension(const size_t& d)
            {
                _d = d;
                return *this;
            }

            // Set controller time step
            AbstractController& setStep(const double& dt)
            {
                _dt = dt;
                return *this;
            }

            // Set controller reference
            AbstractController& setReference(const Space& x)
            {
                _xr = x;
                return *this;
            }

            // Actual controller computation (must be overridden)
            virtual void update(const Space& x) = 0;

            // Update controller internal state and return reference to output (define it as overload operator())
            const Eigen::VectorXd& action(const Space& x)
            {
                update(x);
                return _u;
            }

            const Eigen::VectorXd& operator()(const Space& x)
            {
                update(x);
                return _u;
            }

            // Return reference to output
            const Eigen::VectorXd& output() const
            {
                return _u;
            }

        protected:
            // Output dimension
            size_t _d;

            // Integration time step
            double _dt;

            // Reference state
            Space _xr;

            // Control state
            Eigen::VectorXd _u;
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP