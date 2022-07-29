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

#include <control_lib/controllers/QuadraticProgramming.hpp>
#include <control_lib/spatial/SE3.hpp>
#include <control_lib/spatial/State.h>
#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {};
    struct quadratic_programming : public defaults::quadratic_programming {};
};

struct Model {
    Model() = default;

    Eigen::VectorXd state() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd positionLower() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd positionUpper() const { return Eigen::VectorXd::Random(4); }

    Eigen::VectorXd velocity() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd velocityLower() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd velocityUpper() const { return Eigen::VectorXd::Random(4); }

    Eigen::VectorXd acceleration() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd accelerationLower() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd accelerationUpper() const { return Eigen::VectorXd::Random(4); }

    Eigen::VectorXd effort() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd effortLower() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd effortUpper() const { return Eigen::VectorXd::Random(4); }

    Eigen::MatrixXd inertiaMatrix() const { return Eigen::MatrixXd::Random(4, 4); }
    Eigen::MatrixXd selectionMatrix() const { return Eigen::MatrixXd::Identity(4, 4); }
    Eigen::VectorXd nonLinearEffects() const { return Eigen::VectorXd::Random(4); }
    Eigen::MatrixXd jacobian() const { return Eigen::MatrixXd::Random(3, 4); }
    Eigen::MatrixXd jacobianDerivative() const { return Eigen::MatrixXd::Random(3, 4); }
};

struct Target {
    Target() = default;

    Eigen::VectorXd state() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd velocity() const { return Eigen::VectorXd::Random(4); }
    Eigen::VectorXd acceleration() const { return Eigen::VectorXd::Random(4); }
};

struct Reference {
    Reference() = default;

    Eigen::VectorXd state() const { return Eigen::VectorXd::Random(3); }
    Eigen::VectorXd velocity() const { return Eigen::VectorXd::Random(3); }
    Eigen::VectorXd acceleration() const { return Eigen::VectorXd::Random(3); }
};

int main(int argc, char const* argv[])
{
    size_t nP = 4, nS = 3;
    controllers::QuadraticProgramming<Params> ctr(nP, nS);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Random(nP, nP),
                    R = Eigen::MatrixXd::Random(nP, nP);
    spatial::SE3 x(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    Model model;
    Target target;
    Reference reference;

    // ctr.energyMinimization(Q);
    ctr.dynamicsTracking(Q, target);
    ctr.controlSaturation(R);
    ctr.slackVariable();
    ctr.modelDynamics(model);
    ctr.inverseKinematics(model, reference);
    ctr.inverseDynamics(model, reference);
    ctr.positionLimits(model);
    ctr.velocityLimits(model);
    ctr.accelerationLimits(model);
    ctr.effortLimits(model);

    ctr.init();
    ctr.update(x);

    return 0;
}
