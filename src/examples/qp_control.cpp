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
#include <control_lib/spatial/R.hpp>
#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {};
    struct quadratic_programming : public defaults::quadratic_programming {
        // Problem dimension
        PARAM_SCALAR(size_t, nP, 4);

        // Control input dimension
        PARAM_SCALAR(size_t, nC, 4);

        // Slack variable dimension
        PARAM_SCALAR(size_t, nS, 3);
    };
};

struct Model {
    Model()
    {
        _state = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _velocity = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _acceleration = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _effort = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
    }

    const Eigen::VectorXd& state() const { return _state; }
    const Eigen::VectorXd& velocity() const { return _velocity; }
    const Eigen::VectorXd& acceleration() const { return _acceleration; }
    const Eigen::VectorXd& effort() const { return _effort; }

    Eigen::VectorXd positionLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd positionUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd velocityLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd velocityUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd accelerationLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd accelerationUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd effortLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }
    Eigen::VectorXd effortUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nP()); }

    Eigen::MatrixXd inertiaMatrix(const Eigen::VectorXd& pos) const { return pos * pos.transpose(); }
    Eigen::MatrixXd selectionMatrix() const { return Eigen::MatrixXd::Identity(Params::quadratic_programming::nP(), Params::quadratic_programming::nP()); }
    Eigen::VectorXd nonLinearEffects(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return 1 * vel; }
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& pos) const { return Eigen::MatrixXd::Random(3, Params::quadratic_programming::nP()); }
    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return Eigen::MatrixXd::Random(3, Params::quadratic_programming::nP()); }

    Eigen::VectorXd _state, _velocity, _acceleration, _effort;
};

struct ConfigurationTarget {
    ConfigurationTarget()
    {
        _state = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _velocity = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _acceleration = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
        _effort = Eigen::VectorXd::Random(Params::quadratic_programming::nP());
    }

    size_t dimension() const { return Params::quadratic_programming::nP(); }

    const Eigen::VectorXd& state() const { return _state; }

    const Eigen::VectorXd& velocity() const { return _velocity; }

    const Eigen::VectorXd& acceleration() const { return _acceleration; }

    const Eigen::VectorXd& effort() const { return _effort; }

    Eigen::VectorXd _state, _velocity, _acceleration, _effort;
};

struct TaskTarget {
    TaskTarget()
    {
        _state = Eigen::VectorXd::Random(3);
        _velocity = Eigen::VectorXd::Random(3);
        _acceleration = Eigen::VectorXd::Random(3);
        _effort = Eigen::VectorXd::Random(3);
    }

    size_t dimension() const { return 3; }

    const Eigen::VectorXd& state() const { return _state; }

    const Eigen::VectorXd& velocity() const { return _velocity; }

    const Eigen::VectorXd& acceleration() const { return _acceleration; }

    const Eigen::VectorXd& effort() const { return _effort; }

    Eigen::VectorXd _state, _velocity, _acceleration, _effort;
};

int main(int argc, char const* argv[])
{
    // State
    spatial::R<Params::quadratic_programming::nP()> state;
    state._x = Eigen::Matrix<double, Params::quadratic_programming::nP(), 1>::Random();
    state._v = Eigen::Matrix<double, Params::quadratic_programming::nP(), 1>::Random();

    // Robot model
    std::shared_ptr<Model> model(new Model());

    // Task target model
    TaskTarget targetTask;

    // Configuration target model
    ConfigurationTarget targetConfig;

    // Controller
    controllers::QuadraticProgramming<Params, Model> ctr;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Random(Params::quadratic_programming::nP(), Params::quadratic_programming::nP()),
                    R = Eigen::MatrixXd::Random(Params::quadratic_programming::nC(), Params::quadratic_programming::nC()),
                    W = Eigen::MatrixXd::Identity(Params::quadratic_programming::nS(), Params::quadratic_programming::nS());

    // std::cout << targetConfig._effort.transpose() << std::endl;

    ctr
        .setModel(model)
        .accelerationMinimization(Q)
        .accelerationTracking(Q, targetConfig)
        .effortMinimization(R)
        .effortTracking(R, targetConfig)
        .slackVariable(W)
        .modelDynamics(state)
        // .inverseKinematics(state, targetTask)
        .inverseDynamics(state, targetTask)
        .positionLimits(state)
        .velocityLimits(state)
        .accelerationLimits()
        .effortLimits();

    // std::cout << state._x.transpose() << std::endl;
    // std::cout << state._v.transpose() << std::endl;
    // std::cout << task._velocity.transpose() << std::endl;
    // std::cout << "-" << std::endl;

    // std::cout << "Hessian" << std::endl;
    // std::cout << ctr._opt._H << std::endl;
    // std::cout << "Gradient" << std::endl;
    // std::cout << ctr._opt._g.transpose() << std::endl;
    // std::cout << "Constraints" << std::endl;
    // std::cout << ctr._opt._A << std::endl;
    // std::cout << "Constraints lower" << std::endl;
    // std::cout << ctr._opt._lbA.transpose() << std::endl;
    // std::cout << "Constraints upper" << std::endl;
    // std::cout << ctr._opt._ubA.transpose() << std::endl;
    // std::cout << "Lower" << std::endl;
    // std::cout << ctr._opt._lb.transpose() << std::endl;
    // std::cout << "Upper" << std::endl;
    // std::cout << ctr._opt._ub.transpose() << std::endl;

    std::cout << "====" << std::endl;

    state._x = Eigen::Matrix<double, Params::quadratic_programming::nP(), 1>::Random();
    state._v = Eigen::Matrix<double, Params::quadratic_programming::nP(), 1>::Random();

    targetTask._velocity = Eigen::Matrix<double, 3, 1>::Random();
    targetTask._acceleration = Eigen::Matrix<double, 3, 1>::Random();

    targetConfig._acceleration = Eigen::Matrix<double, 4, 1>::Random();
    // targetConfig._effort = Eigen::Matrix<double, 4, 1>::Random();

    // std::cout << targetConfig._effort.transpose() << std::endl;

    ctr.update(state);

    // std::cout << "Hessian" << std::endl;
    // std::cout << ctr._opt._H << std::endl;
    // std::cout << "Gradient" << std::endl;
    // std::cout << ctr._opt._g.transpose() << std::endl;
    // std::cout << "Constraints" << std::endl;
    // std::cout << ctr._opt._A << std::endl;
    // std::cout << "Constraints lower" << std::endl;
    // std::cout << ctr._opt._lbA.transpose() << std::endl;
    // std::cout << "Constraints upper" << std::endl;
    // std::cout << ctr._opt._ubA.transpose() << std::endl;
    // std::cout << "Lower" << std::endl;
    // std::cout << ctr._opt._lb.transpose() << std::endl;
    // std::cout << "Upper" << std::endl;
    // std::cout << ctr._opt._ub.transpose() << std::endl;

    return 0;
}
