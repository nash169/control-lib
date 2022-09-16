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

#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>

#include <control_lib/spatial/RN.hpp>
#include <control_lib/spatial/SE3.hpp>
#include <control_lib/spatial/SO3.hpp>

#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };
};

int main(int argc, char const* argv[])
{
    // Referece state
    Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
    Eigen::Matrix3d oDes;
    oDes << 0.591427, -0.62603, 0.508233,
        0.689044, 0.719749, 0.0847368,
        -0.418848, 0.300079, 0.857041;
    spatial::SE3 sDes(oDes, xDes);

    // Current state
    spatial::SE3 sCurr(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    controllers::LinearDynamics<Params, spatial::SE3> ctr1;

    std::cout << ctr1.dimension() << std::endl;
    // ctr1.setReference(sDes);
    // std::cout << "Linear DS action" << std::endl;
    // std::cout << ctr1.action(sCurr).transpose() << std::endl;

    // sDes._vel = ctr1.action(sCurr);
    // sCurr._vel = Eigen::VectorXd::Random(6);
    // controllers::Feedback<Params, spatial::SE3> ctr2;
    // ctr2
    //     // .setStiffness(Eigen::MatrixXd::Identity(6, 6))
    //     .setDamping(Eigen::MatrixXd::Identity(6, 6))
    //     // .setIntegral(Eigen::MatrixXd::Identity(6, 6))
    //     .setReference(sDes);
    // std::cout << "Feedback action" << std::endl;
    // std::cout << ctr2.action(sCurr).transpose() << std::endl;

    // std::cout << "test" << std::endl;
    // spatial::RN<3> a(Eigen::Vector3d::Random()), b(Eigen::Vector3d::Random());
    // std::cout << (a - b).transpose() << std::endl;
    // std::cout << (a._pos - b._pos).transpose() << std::endl;

    return 0;
}
