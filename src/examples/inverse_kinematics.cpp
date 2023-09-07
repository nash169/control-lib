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

#include <control_lib/controllers/InverseKinematics.hpp>

#include <control_lib/spatial/SE.hpp>

#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
        PARAM_SCALAR(double, dt, 1.0);
    };
    struct inverse_kinematics : public defaults::inverse_kinematics {
        // Problem dimension
        PARAM_SCALAR(size_t, nP, 7);

        // Slack variable dimension
        PARAM_SCALAR(size_t, nS, 6);
    };
};

struct Model {
    Model()
    {
        _uP = Eigen::VectorXd::Random(Params::inverse_kinematics::nP());
        _lP = Eigen::VectorXd::Random(Params::inverse_kinematics::nP());
        _uV = Eigen::VectorXd::Random(Params::inverse_kinematics::nP());
        _lV = Eigen::VectorXd::Random(Params::inverse_kinematics::nP());
    }

    Eigen::VectorXd positionLower() const { return _lP; }
    Eigen::VectorXd positionUpper() const { return _uP; }
    Eigen::VectorXd velocityLower() const { return _lV; }
    Eigen::VectorXd velocityUpper() const { return _uV; }
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& pos) const { return Eigen::MatrixXd::Random(Params::inverse_kinematics::nS(), Params::inverse_kinematics::nP()); }

    Eigen::VectorXd _uP, _lP, _uV, _lV;
};

struct ConfigurationVelocityTarget : public controllers::AbstractController<Params, spatial::R<Params::inverse_kinematics::nP()>> {
    ConfigurationVelocityTarget()
    {
        _d = Params::inverse_kinematics::nP();
        _u = Eigen::VectorXd::Zero(_d);
    }

    void update(const spatial::R<Params::inverse_kinematics::nP()>& x) override
    {
        _u = Eigen::VectorXd::Random(_d);
    }
};

struct TaskVelocityTarget : public controllers::AbstractController<Params, spatial::SE<3>> {
    TaskVelocityTarget()
    {
        _d = spatial::SE<3>::dimension();
        _u = Eigen::VectorXd::Zero(_d);
    }

    void update(const spatial::SE<3>& x) override
    {
        _u = Eigen::VectorXd::Random(_d);
    }
};

int main(int argc, char const* argv[])
{
    // Config
    spatial::R<Params::inverse_kinematics::nP()> state;
    state._x = Eigen::Matrix<double, Params::inverse_kinematics::nP(), 1>::Random();
    state._v = Eigen::Matrix<double, Params::inverse_kinematics::nP(), 1>::Random();

    // Task
    spatial::SE<3> pose(Eigen::MatrixXd::Identity(3, 3), Eigen::VectorXd::Zero(3));

    // Robot model
    std::shared_ptr<Model> model(new Model());

    // Configuration target model
    ConfigurationVelocityTarget configTarget;
    configTarget.update(state);

    // Task target model
    TaskVelocityTarget taskTarget;
    taskTarget.update(pose);

    // Controller
    controllers::InverseKinematics<Params, Model> ctr;
    Eigen::MatrixXd Q = 10 * Eigen::MatrixXd::Identity(Params::inverse_kinematics::nP(), Params::inverse_kinematics::nP()),
                    W = Eigen::MatrixXd::Identity(Params::inverse_kinematics::nS(), Params::inverse_kinematics::nS());

    ctr
        .setModel(model)
        .velocityMinimization(Q)
        .velocityTracking(Q, configTarget)
        .slackVariable(W)
        .inverseKinematics(state, taskTarget)
        .positionLimits(state)
        .velocityLimits()
        .init();

    std::cout << "State" << std::endl;
    std::cout << state._x.transpose() << std::endl;
    std::cout << "Velocity" << std::endl;
    std::cout << state._v.transpose() << std::endl;
    std::cout << "Task Target" << std::endl;
    std::cout << taskTarget.output().transpose() << std::endl;
    std::cout << "Config Target" << std::endl;
    std::cout << configTarget.output().transpose() << std::endl;
    std::cout << "Hessian" << std::endl;
    std::cout << ctr._opt._H << std::endl;
    std::cout << "Gradient" << std::endl;
    std::cout << ctr._opt._g.transpose() << std::endl;
    std::cout << "Constraints" << std::endl;
    std::cout << ctr._opt._A << std::endl;
    std::cout << "Constraints lower" << std::endl;
    std::cout << ctr._opt._lbA.transpose() << std::endl;
    std::cout << "Constraints upper" << std::endl;
    std::cout << ctr._opt._ubA.transpose() << std::endl;
    std::cout << "Lower Bounds" << std::endl;
    std::cout << ctr._opt._lb.transpose() << std::endl;
    std::cout << "Upper Bounds" << std::endl;
    std::cout << ctr._opt._ub.transpose() << std::endl;

    std::cout << "====" << std::endl;

    configTarget.update(state);
    taskTarget.update(pose);
    ctr.update(state);

    std::cout << "State" << std::endl;
    std::cout << state._x.transpose() << std::endl;
    std::cout << "Velocity" << std::endl;
    std::cout << state._v.transpose() << std::endl;
    std::cout << "Task Target" << std::endl;
    std::cout << taskTarget.output().transpose() << std::endl;
    std::cout << "Config Target" << std::endl;
    std::cout << configTarget.output().transpose() << std::endl;
    std::cout << "Hessian" << std::endl;
    std::cout << ctr._opt._H << std::endl;
    std::cout << "Gradient" << std::endl;
    std::cout << ctr._opt._g.transpose() << std::endl;
    std::cout << "Constraints" << std::endl;
    std::cout << ctr._opt._A << std::endl;
    std::cout << "Constraints lower" << std::endl;
    std::cout << ctr._opt._lbA.transpose() << std::endl;
    std::cout << "Constraints upper" << std::endl;
    std::cout << ctr._opt._ubA.transpose() << std::endl;
    std::cout << "Lower Bounds" << std::endl;
    std::cout << ctr._opt._lb.transpose() << std::endl;
    std::cout << "Upper Bounds" << std::endl;
    std::cout << ctr._opt._ub.transpose() << std::endl;

    return 0;
}
