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

#include "control_lib/controllers/Feedback.hpp"

#include <iostream>

namespace control_lib {
    namespace controllers {
        Feedback::Feedback(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step) : AbstractController(type, input_dim, output_dim, time_step)
        {
            // This might be moved in the abstract class
            _error = utils::ControlState(input_dim, type);

            // Init integral error
            _integral_error = Eigen::VectorXd::Zero(input_dim);
        }

        Feedback& Feedback::setGains(const std::string& type, const Eigen::MatrixXd& mat)
        {
            if (!type.compare("p"))
                _gains.p_matrix = mat;
            else if (!type.compare("d"))
                _gains.d_matrix = mat;
            else
                _gains.i_matrix = mat;

            return *this;
        }

        Eigen::VectorXd Feedback::update(const Eigen::VectorXd& state)
        {
            // This might change if we integrate the delay inside the controller
            _output.setZero();

            // Memorize position error for the integral part
            auto error_prev = _error.getPos();

            // This should be speeded up using the information from the reference
            setInput(state);

            // Update error
            _error = _input - _reference;

            if (_gains.p_matrix.size())
                _output += -_gains.p_matrix * _error.getPos();

            if (_gains.d_matrix.size())
                _output += -_gains.d_matrix * _error._velocity;

            if (_gains.i_matrix.size()) {
                if (error_prev.size())
                    _integral_error += (error_prev + _error.getPos()) * _time_step / 2;

                _output += -_gains.i_matrix * _integral_error;
            }

            return _output;
        }
    } // namespace controllers
} // namespace control_lib
