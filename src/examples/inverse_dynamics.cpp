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

// spatial
#include <control_lib/spatial/SE.hpp>

// controller
#include <control_lib/controllers/QuadraticControl.hpp>
#include <control_lib/controllers/QuadraticProgramming.hpp>

#include <iostream>

using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
        PARAM_SCALAR(double, dt, 1.0);
    };

    struct quadratic_control : public defaults::quadratic_control {
        // State dimension
        PARAM_SCALAR(size_t, nP, 7);

        // Control/Input dimension
        PARAM_SCALAR(size_t, nC, 7);

        // Slack variable dimension
        PARAM_SCALAR(size_t, nS, 6);

        // derivative order
        PARAM_SCALAR(size_t, oD, 2);
    };

    struct quadratic_programming : public defaults::quadratic_programming {
        // Problem dimension
        PARAM_SCALAR(size_t, nP, 7);

        // Control input dimension
        PARAM_SCALAR(size_t, nC, 7);

        // Slack variable dimension
        PARAM_SCALAR(size_t, nS, 6);
    };

    PARAM_SCALAR(size_t, nP, 7);

    PARAM_SCALAR(size_t, nC, 7);

    PARAM_SCALAR(size_t, nT, 6);
};

struct Model {
    Model()
    {
        // zero derivative constraints
        _uP = Eigen::VectorXd::Random(Params::nP());
        _lP = Eigen::VectorXd::Random(Params::nP());

        // first derivative constraints
        _uV = Eigen::VectorXd::Random(Params::nP());
        _lV = Eigen::VectorXd::Random(Params::nP());

        // second derivative constraints
        _uA = Eigen::VectorXd::Random(Params::nP());
        _lA = Eigen::VectorXd::Random(Params::nP());

        // input constraints
        _uE = Eigen::VectorXd::Random(Params::nP());
        _lE = Eigen::VectorXd::Random(Params::nP());
    }

    // Limits
    Eigen::VectorXd positionLower() const { return _lP; }
    Eigen::VectorXd positionUpper() const { return _uP; }

    Eigen::VectorXd velocityLower() const { return _lV; }
    Eigen::VectorXd velocityUpper() const { return _uV; }

    Eigen::VectorXd accelerationLower() const { return _lA; }
    Eigen::VectorXd accelerationUpper() const { return _uA; }

    Eigen::VectorXd effortLower() const { return _lE; }
    Eigen::VectorXd effortUpper() const { return _uE; }

    // For dynamics' constraint
    Eigen::MatrixXd inertiaMatrix(const Eigen::VectorXd& pos) const { return pos * pos.transpose(); }
    Eigen::MatrixXd selectionMatrix() const { return Eigen::MatrixXd::Identity(Params::nP(), Params::nP()); }
    Eigen::VectorXd nonLinearEffects(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return 1 * vel; }

    // For inverse kinematics/dynamics constraints
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& pos) const { return (pos * pos.transpose()).block(0, 0, Params::nT(), Params::nP()); }
    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& pos, const Eigen::VectorXd& vel) const { return (pos * vel.transpose()).block(0, 0, Params::nT(), Params::nP()); }

    Eigen::VectorXd _uP, _lP, _uV, _lV, _uA, _lA, _uE, _lE;
};

// depending on the qp configuration it can be pos, vel or acc
struct ConfigurationTarget : public controllers::AbstractController<Params, spatial::R<Params::nP()>> {
    ConfigurationTarget()
    {
        _d = Params::nP();
        _u = Eigen::VectorXd::Zero(_d);
    }

    void update(const spatial::R<Params::nP()>& x) override
    {
        _u = Eigen::VectorXd::Random(_d);
    }
};

// control target
struct InputTarget : public controllers::AbstractController<Params, spatial::R<Params::nC()>> {
    InputTarget()
    {
        _d = Params::nC();
        _u = Eigen::VectorXd::Zero(_d);
    }

    void update(const spatial::R<Params::nC()>& x) override
    {
        _u = Eigen::VectorXd::Random(_d);
    }
};

// depending on the constraints it can be either pos, vel or acc
struct TaskTarget : public controllers::AbstractController<Params, spatial::SE<3>> {
    TaskTarget()
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
    spatial::R<Params::nP()> state;
    state._x = Eigen::Matrix<double, Params::nP(), 1>::Random();
    state._v = Eigen::Matrix<double, Params::nP(), 1>::Random();

    // Task
    spatial::SE<3> pose(Eigen::MatrixXd::Identity(3, 3), Eigen::VectorXd::Zero(3));

    // Robot model
    std::shared_ptr<Model> model(new Model());

    // Configuration target
    ConfigurationTarget configTarget;
    configTarget.update(state);

    // Input target
    InputTarget inputTarget;
    inputTarget.update(state);

    // Task target model
    TaskTarget taskTarget;
    taskTarget.update(pose);

    // Controller
    controllers::QuadraticControl<Params, Model> ctr;
    controllers::QuadraticProgramming<Params, Model> id;
    Eigen::MatrixXd Q = 10.0 * Eigen::MatrixXd::Identity(Params::nP(), Params::nP()),
                    R = 5.0 * Eigen::MatrixXd::Identity(Params::nC(), Params::nC()),
                    S = 1.0 * Eigen::MatrixXd::Identity(Params::nT(), Params::nT());

    ctr
        .setModel(model)
        .stateCost(Q)
        .stateReference(configTarget.output())
        .inputCost(R)
        .inputReference(inputTarget.output())
        .slackCost(S)
        .modelConstraint()
        // .inverseKinematics(taskTarget.output())
        .inverseDynamics(taskTarget.output())
        .positionLimits()
        .velocityLimits()
        .accelerationLimits()
        .effortLimits()
        .init(state);

    id
        .setModel(model)
        .accelerationMinimization(Q)
        .accelerationTracking(Q, configTarget)
        .effortMinimization(R)
        .effortTracking(R, inputTarget)
        .slackVariable(S)
        .modelDynamics(state)
        // .inverseKinematics(state, targetTask)
        .inverseDynamics(state, taskTarget)
        .positionLimits(state)
        .velocityLimits(state)
        .accelerationLimits()
        .effortLimits()
        .init();

    std::cout << "State" << std::endl;
    std::cout << state._x.transpose() << std::endl;
    std::cout << "Velocity" << std::endl;
    std::cout << state._v.transpose() << std::endl;
    std::cout << "Task Target" << std::endl;
    std::cout << taskTarget.output().transpose() << std::endl;
    std::cout << "Config Target" << std::endl;
    std::cout << configTarget.output().transpose() << std::endl;
    std::cout << "Input Target" << std::endl;
    std::cout << inputTarget.output().transpose() << std::endl;

    std::cout << "Hessian" << std::endl;
    std::cout << ctr._opt._H << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._H << std::endl;

    std::cout << "Gradient" << std::endl;
    std::cout << ctr._opt._g.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._g.transpose() << std::endl;

    std::cout << "Constraints" << std::endl;
    std::cout << ctr._opt._A << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._A << std::endl;

    std::cout << "Constraints lower" << std::endl;
    std::cout << ctr._opt._lbA.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._lbA.transpose() << std::endl;

    std::cout << "Constraints upper" << std::endl;
    std::cout << ctr._opt._ubA.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._ubA.transpose() << std::endl;

    std::cout << "Lower Bounds" << std::endl;
    std::cout << ctr._opt._lb.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._lb.transpose() << std::endl;

    std::cout << "Upper Bounds" << std::endl;
    std::cout << ctr._opt._ub.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._ub.transpose() << std::endl;

    std::cout << "====" << std::endl;

    configTarget.update(state);
    inputTarget.update(state);
    taskTarget.update(pose);
    ctr.update(state);
    id.update(state);

    std::cout << "State" << std::endl;
    std::cout << state._x.transpose() << std::endl;
    std::cout << "Velocity" << std::endl;
    std::cout << state._v.transpose() << std::endl;
    std::cout << "Task Target" << std::endl;
    std::cout << taskTarget.output().transpose() << std::endl;
    std::cout << "Config Target" << std::endl;
    std::cout << configTarget.output().transpose() << std::endl;
    std::cout << "Input Target" << std::endl;
    std::cout << inputTarget.output().transpose() << std::endl;

    std::cout << "Hessian" << std::endl;
    std::cout << ctr._opt._H << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._H << std::endl;

    std::cout << "Gradient" << std::endl;
    std::cout << ctr._opt._g.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._g.transpose() << std::endl;

    std::cout << "Constraints" << std::endl;
    std::cout << ctr._opt._A << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._A << std::endl;

    std::cout << "Constraints lower" << std::endl;
    std::cout << ctr._opt._lbA.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._lbA.transpose() << std::endl;

    std::cout << "Constraints upper" << std::endl;
    std::cout << ctr._opt._ubA.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._ubA.transpose() << std::endl;

    std::cout << "Lower Bounds" << std::endl;
    std::cout << ctr._opt._lb.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._lb.transpose() << std::endl;

    std::cout << "Upper Bounds" << std::endl;
    std::cout << ctr._opt._ub.transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << id._opt._ub.transpose() << std::endl;

    return 0;
}
