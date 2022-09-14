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
#include <control_lib/spatial/RN.hpp>
#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {};
    struct quadratic_programming : public defaults::quadratic_programming {
        // Problem dimension
        PARAM_SCALAR(size_t, nV, 4);

        // Control input dimension
        PARAM_SCALAR(size_t, nC, 4);

        // Slack variable dimension
        PARAM_SCALAR(size_t, nS, 3);
    };
};

struct Model {
    Model()
    {
        _state = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
        _velocity = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
        _acceleration = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
        _effort = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
    }

    const Eigen::VectorXd& state() const { return _state; }
    const Eigen::VectorXd& velocity() const { return _velocity; }
    const Eigen::VectorXd& acceleration() const { return _acceleration; }
    const Eigen::VectorXd& effort() const { return _effort; }

    Eigen::VectorXd positionLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd positionUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd velocityLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd velocityUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd accelerationLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd accelerationUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd effortLower() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }
    Eigen::VectorXd effortUpper() const { return Eigen::VectorXd::Random(Params::quadratic_programming::nV()); }

    Eigen::MatrixXd inertiaMatrix(const Eigen::VectorXd& pos) const { return pos * pos.transpose(); }
    Eigen::MatrixXd selectionMatrix() const { return Eigen::MatrixXd::Identity(Params::quadratic_programming::nV(), Params::quadratic_programming::nV()); }
    Eigen::VectorXd nonLinearEffects(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return 1 * vel; }
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& pos) const { return Eigen::MatrixXd::Random(3, Params::quadratic_programming::nV()); }
    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return Eigen::MatrixXd::Random(3, Params::quadratic_programming::nV()); }

    Eigen::VectorXd _state, _velocity, _acceleration, _effort;
};

struct ConfigurationTarget {
    ConfigurationTarget()
    {
        _state = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
        _velocity = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
        _acceleration = Eigen::VectorXd::Random(Params::quadratic_programming::nV());
    }

    size_t dim() const { return Params::quadratic_programming::nV(); }

    const Eigen::VectorXd& state() const { return _state; }

    const Eigen::VectorXd& velocity() const { return _velocity; }

    const Eigen::VectorXd& acceleration() const { return _acceleration; }

    Eigen::VectorXd _state, _velocity, _acceleration;
};

struct TaskTarget {
    TaskTarget()
    {
        _state = Eigen::VectorXd::Random(3);
        _velocity = Eigen::VectorXd::Random(3);
        _acceleration = Eigen::VectorXd::Random(3);
    }

    size_t dim() const { return 3; }

    const Eigen::VectorXd& state() const { return _state; }

    const Eigen::VectorXd& velocity() const { return _velocity; }

    const Eigen::VectorXd& acceleration() const { return _acceleration; }

    Eigen::VectorXd _state, _velocity, _acceleration;
};

int main(int argc, char const* argv[])
{
    // Controller
    controllers::QuadraticProgramming<Params, Model> ctr;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Random(Params::quadratic_programming::nV(), Params::quadratic_programming::nV()),
                    R = Eigen::MatrixXd::Random(Params::quadratic_programming::nV(), Params::quadratic_programming::nV());

    // Robot model
    Model model;

    // Task target model
    TaskTarget task;

    // Configuration target model
    ConfigurationTarget config;

    spatial::RN<Params::quadratic_programming::nV()> state;
    state._pos = Eigen::Matrix<double, Params::quadratic_programming::nV(), 1>::Random();
    state._vel = Eigen::Matrix<double, Params::quadratic_programming::nV(), 1>::Random();

    ctr
        .modelDynamics(state)
        .inverseKinematics(state, task);

    // std::cout << state._pos.transpose() << std::endl;
    // std::cout << state._vel.transpose() << std::endl;
    // std::cout << task._velocity.transpose() << std::endl;
    // std::cout << "-" << std::endl;
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
    std::cout << "Lower" << std::endl;
    std::cout << ctr._opt._lbA.transpose() << std::endl;
    std::cout << "Upper" << std::endl;
    std::cout << ctr._opt._ubA.transpose() << std::endl;

    std::cout << "====" << std::endl;

    state._pos = Eigen::Matrix<double, Params::quadratic_programming::nV(), 1>::Random();
    state._vel = Eigen::Matrix<double, Params::quadratic_programming::nV(), 1>::Random();
    task._velocity = Eigen::Matrix<double, 3, 1>::Random();
    task._acceleration = Eigen::Matrix<double, 3, 1>::Random();

    // ctr.tempUpdate(state);
    // std::cout << state._pos.transpose() << std::endl;
    // std::cout << state._vel.transpose() << std::endl;
    // std::cout << task._velocity.transpose() << std::endl;
    // std::cout << "-" << std::endl;
    // std::cout << ctr._opt._A << std::endl;
    // std::cout << ctr._opt._lbA.transpose() << std::endl;
    // std::cout << ctr._opt._ubA.transpose() << std::endl;

    // Eigen::MatrixXd mat(4,11)
    // Model model;
    // Target target;
    // Reference reference;

    // // ctr.energyMinimization(Q);
    // ctr.dynamicsTracking(Q, target);
    // ctr.controlSaturation(R);
    // ctr.slackVariable();
    // ctr.modelDynamics(model);
    // ctr.inverseKinematics(model, reference);
    // ctr.inverseDynamics(model, reference);
    // ctr.positionLimits(model);
    // ctr.velocityLimits(model);
    // ctr.accelerationLimits(model);
    // ctr.effortLimits(model);

    // ctr.init();
    // ctr.update(x);

    // std::shared_ptr<Eigen::MatrixXd> mat = std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Zero(5, 5));

    // // Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> submat = mat.block(0, 0, 3, 3);

    // std::shared_ptr<Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>> submat = std::make_shared<Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>>(mat->block(0, 0, 3, 3));

    // mat->conservativeResize(Eigen::NoChange, 10);

    // *submat = Eigen::MatrixXd::Random(3, 3);

    // std::cout << *mat << std::endl;

    // submat = Eigen::MatrixXd::Random(3, 3);

    // std::cout << "--" << std::endl;
    // std::cout << mat << std::endl;

    // auto* p = mat.transpose().data();

    // Eigen::VectorXd vec = Eigen::VectorXd::Zero(5);
    // Eigen::Ref<Eigen::VectorXd> subvec = vec.segment(0, 3);
    // vec.conservativeResize(10);
    // subvec = Eigen::VectorXd::Random(3);

    // std::cout << vec << std::endl;

    // subvec = Eigen::VectorXd::Random(3);

    // std::cout << "--" << std::endl;
    // std::cout << vec << std::endl;

    // std::vector<std::function<void(int)>> vec;

    // {
    //     int b = 4;
    //     vec.push_back(std::bind(fun, std::placeholders::_1, b));
    // }

    // vec[0](1);

    return 0;
}
