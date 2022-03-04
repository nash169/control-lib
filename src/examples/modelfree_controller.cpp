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
#include <control_lib/controllers/Feedback2.hpp>
#include <control_lib/spatial/SE3.hpp>
#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };
};

int main(int argc, char const* argv[])
{
    // Examples state
    // 3 pose + 4 quaternion + 5 generalized coordinates -> dimension = 12

    size_t dim = 12, ctr_dim = 4;
    Eigen::VectorXd q = Eigen::VectorXd::Random(2 * dim - 1), q_ref = Eigen::VectorXd::Random(2 * dim - 1);

    // Reference state creation
    q << 0.9572, 0.4854, 0.8003, 0.1419, 0.4218, 0.9157, 0.7922, 0.9595, 0.6557, 0.0357, 0.8491, 0.9340,
        0.7094, 0.7547, 0.2760, 0.6797, 0.6551, 0.1626, 0.1190, 0.4984, 0.9597, 0.3404, 0.5853;

    // Reference state creation
    q_ref << 0.6948, 0.3171, 0.9502, 0.0344, 0.4387, 0.3816, 0.7655, 0.7952, 0.1869, 0.4898, 0.4456, 0.6463,
        0.7513, 0.2551, 0.5060, 0.6991, 0.8909, 0.9593, 0.5472, 0.1386, 0.1493, 0.2575, 0.8407;

    // Controller creation
    double step = 0.001;
    Eigen::MatrixXd p_gains = Eigen::MatrixXd::Random(ctr_dim, dim),
                    d_gains = Eigen::MatrixXd::Random(ctr_dim, dim - 1),
                    i_gains = Eigen::MatrixXd::Random(ctr_dim, dim);

    controllers::Feedback ctr(ControlSpace::LINEAR | ControlSpace::QUATERNION, dim, ctr_dim, step);

    ctr.setReference(q_ref);
    ctr.setGains("p", p_gains).setGains("d", d_gains).setGains("i", i_gains);

    // std::cout << "Control ouput (step 1)" << std::endl;
    // std::cout << ctr.update(q).transpose() << std::endl;

    // std::cout << "Control ouput (step 2)" << std::endl;
    // std::cout << ctr.update(q).transpose() << std::endl;

    Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
    Eigen::Matrix3d oDes;
    oDes << 0.591427, -0.62603, 0.508233,
        0.689044, 0.719749, 0.0847368,
        -0.418848, 0.300079, 0.857041;
    spatial::SE3 curr(oDes, xDes);
    Eigen::VectorXd vel = Eigen::VectorXd::Random(6);

    controllers::Feedback2<Params, spatial::SE3> ctr2;
    ctr2
        .setStiffness(Eigen::MatrixXd::Identity(6, 6))
        .setDamping(Eigen::MatrixXd::Identity(6, 6))
        .setIntegral(Eigen::MatrixXd::Identity(6, 6));
    std::cout << "action" << std::endl;
    std::cout << ctr2.action(curr, vel).transpose() << std::endl;

    return 0;
}
