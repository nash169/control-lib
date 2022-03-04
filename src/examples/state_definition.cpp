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

#include <control_lib/Control.hpp>
#include <control_lib/spatial/SE3.hpp>
#include <iostream>

using namespace control_lib;

int main(int argc, char const* argv[])
{
    /* If orientation is define the first part of the state is considered to be
    defining the location of the body in the space. Otherwise the state is
    considered to be generalized cooridnate where LINEAR geometry applies by default.
    The dimension of the state to the number of degree of freedom plus the
    pose in the space (consider one dimension more four quaternion). If the length
    of the state exceeds the dimension deifined velocity, acceleration and effort are
    filled in this order. */

    // Examples state
    // 3 pose + 4 quaternion + 5 generalized coordinates -> dimension = 12

    size_t dim = 12;
    Eigen::VectorXd q = Eigen::VectorXd::Random(dim), q_ref = Eigen::VectorXd::Random(dim);

    // Reference state creation
    q << 0.9572, 0.4854, 0.8003, 0.1419, 0.4218, 0.9157, 0.7922, 0.9595, 0.6557, 0.0357, 0.8491, 0.9340;
    utils::ControlState state(dim, ControlSpace::LINEAR | ControlSpace::QUATERNION);
    state.setState(q);

    // Reference state createio
    q_ref << 0.6948, 0.3171, 0.9502, 0.0344, 0.4387, 0.3816, 0.7655, 0.7952, 0.1869, 0.4898, 0.4456, 0.6463;
    utils::ControlState state_ref(dim, ControlSpace::LINEAR | ControlSpace::QUATERNION);
    state_ref.setState(q_ref);

    // Difference between states
    auto diff = state - state_ref;

    std::cout << "Pose" << std::endl;
    std::cout << diff._pose.transpose() << std::endl;

    std::cout << "Orientation" << std::endl;
    std::cout << diff._orientation.transpose() << std::endl;

    std::cout << "Generalized coordinate" << std::endl;
    std::cout << diff._coordinate.transpose() << std::endl;

    std::cout << "Velocity" << std::endl;
    std::cout << diff._velocity.transpose() << std::endl;

    std::cout << "Acceleration" << std::endl;
    std::cout << diff._acceleration.transpose() << std::endl;

    std::cout << "Effort" << std::endl;
    std::cout << diff._effort.transpose() << std::endl;

    return 0;
}
