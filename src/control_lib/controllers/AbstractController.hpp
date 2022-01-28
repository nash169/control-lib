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

#ifndef CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP
#define CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP

#include "control_lib/tools/math.hpp"
#include "control_lib/utils/ControlState.hpp"

namespace control_lib {
    namespace controllers {
        class AbstractController {
        public:
            AbstractController(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01) : _output_dim(output_dim), _time_step(time_step)
            {
                // Init input and reference
                _input = utils::ControlState(input_dim, type);
                _reference = utils::ControlState(input_dim, type);

                // Init output
                _output = Eigen::VectorXd::Zero(output_dim);
            }

            AbstractController() {}

            virtual ~AbstractController() {}

            void init();

            Eigen::VectorXd getOutput()
            {
                return _output;
            }

            void setControlDimension(const size_t output_dim)
            {
                _output_dim = output_dim;
            }

            void setInput(const Eigen::VectorXd& input)
            {
                _input.setState(input);
            }

            void setReference(const Eigen::VectorXd& reference)
            {
                _reference.setState(reference);
            }

            void setTimeStep(const double time_step)
            {
                _time_step = time_step;
            }

            virtual Eigen::VectorXd update(const Eigen::VectorXd& state) = 0;

        protected:
            // Controller frequency
            double _time_step;

            // Input state and reference
            utils::ControlState _input, _reference;

            // Output dimension and state
            size_t _output_dim;
            Eigen::VectorXd _output;
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP