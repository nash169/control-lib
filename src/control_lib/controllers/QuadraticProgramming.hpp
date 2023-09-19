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

#ifndef CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP
#define CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP

#include <functional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <optimization_lib/QpoasesOptimizer.hpp>

#include "control_lib/controllers/AbstractController.hpp"
#include "control_lib/spatial/R.hpp"

// using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

namespace control_lib {
    namespace defaults {
        struct quadratic_programming {
            // State dimension
            PARAM_SCALAR(size_t, nP, 1);

            // Control input dimension (optional)
            PARAM_SCALAR(size_t, nC, 1);

            // Slack variable dimension (optional)
            PARAM_SCALAR(size_t, nS, 0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Model>
        class QuadraticProgramming : public AbstractController<Params, spatial::R<Params::quadratic_programming::nP()>> {
        public:
            QuadraticProgramming() : _nP(Params::quadratic_programming::nP()), _nC(Params::quadratic_programming::nC()), _nS(Params::quadratic_programming::nS())
            {
                // total problem dimension
                _d = _nP + _nC + _nS;

                // init hessian matrix
                _opt.setHessianMatrix(Eigen::MatrixXd::Zero(_d, _d));

                // init gradient vector
                _opt.setGradientVector(Eigen::VectorXd::Zero(_d));

                // init lower and upper boundaries
                _opt.setVariablesBoundaries(-1e8 * Eigen::VectorXd::Ones(_d), 1e8 * Eigen::VectorXd::Ones(_d));
            }

            /* Model */
            QuadraticProgramming& setModel(const std::shared_ptr<Model>& model)
            {
                _model = model;
                return *this;
            }

            /* Objectives */
            QuadraticProgramming& accelerationMinimization(Eigen::MatrixXd& Q)
            {
                _opt._H.block(0, 0, _nP, _nP) = Q;
                return *this;
            }

            template <typename Target>
            QuadraticProgramming& accelerationTracking(const Eigen::MatrixXd& Q, const Target& target)
            {
                // Add objective
                _objectives.push_back(std::bind(&QuadraticProgramming<Params, Model>::accelerationTrackingImpl, this,
                    Q, target.acceleration().segment(0, target.acceleration().size())));

                // Run objective
                _objectives.back()();

                return *this;
            }

            void accelerationTrackingImpl(const Eigen::MatrixXd& Q, Eigen::Ref<const Eigen::VectorXd> targetAcc)
            {
                // std::cout << "qp" << std::endl;
                // std::cout << targetAcc.transpose() << std::endl;
                _opt._g.segment(0, _nP) = -2 * Q * targetAcc;
            }

            QuadraticProgramming& effortMinimization(Eigen::MatrixXd& R)
            {
                _opt._H.block(_nP, _nP, _nC, _nC) = R;
                return *this;
            }

            template <typename Target>
            QuadraticProgramming& effortTracking(const Eigen::MatrixXd& R, const Target& target)
            {
                // Add objective
                _objectives.push_back(std::bind(&QuadraticProgramming<Params, Model>::effortTrackingImpl, this,
                    R, target.effort().segment(0, target.effort().size())));

                // Run objective
                _objectives.back()();

                return *this;
            }

            void effortTrackingImpl(const Eigen::MatrixXd& R, Eigen::Ref<const Eigen::VectorXd> targetEff)
            {
                // std::cout << "qp" << std::endl;
                // std::cout << targetEff.transpose() << std::endl;
                _opt._g.segment(_nP, _nC) = -2 * R * targetEff;
            }

            QuadraticProgramming& slackVariable(Eigen::MatrixXd& W)
            {
                _opt._H.block(_nP + _nC, _nP + _nC, _nS, _nS) = W;
                return *this;
            }

            /* Equality Constraints */
            QuadraticProgramming& modelDynamics(const spatial::R<Params::quadratic_programming::nP()>& state)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = _nP;

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d); // here it does not seem to be possible to use Eigen::NoChange
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::modelDynamicsImpl, this, std::placeholders::_1, start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void modelDynamicsImpl(const spatial::R<Params::quadratic_programming::nP()>& state, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << _model->inertiaMatrix(state._x),
                    -_model->selectionMatrix(),
                    Eigen::Matrix<double, Params::quadratic_programming::nP(), Params::quadratic_programming::nS()>::Zero();

                _opt._lbA.segment(start, size) = -_model->nonLinearEffects(state._x, state._v);
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            template <typename Target>
            QuadraticProgramming& inverseKinematics(const spatial::R<Params::quadratic_programming::nP()>& state, const Target& target)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = target.velocity().size();

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::inverseKinematicsImpl, this, std::placeholders::_1, target.velocity().segment(0, size), start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void inverseKinematicsImpl(const spatial::R<Params::quadratic_programming::nP()>& state, Eigen::Ref<const Eigen::VectorXd> targetVel, const size_t& start, const size_t& size)
            {
                // std::cout << "qp" << std::endl;
                // std::cout << targetVel.transpose() << std::endl;
                _opt._A.block(start, 0, size, _d) << _dt * _model->jacobian(state._x),
                    Eigen::MatrixXd::Zero(targetVel.size(), _nP),
                    -Eigen::Matrix<double, Params::quadratic_programming::nS(), Params::quadratic_programming::nS()>::Identity();

                _opt._lbA.segment(start, size) = targetVel - _model->jacobian(state._x) * _model->velocity(); // _model->velocity() being the velocity at the previous step
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            template <typename Target>
            QuadraticProgramming& inverseDynamics(const spatial::R<Params::quadratic_programming::nP()>& state, const Target& target)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = target.acceleration().size();

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::inverseDynamicsImpl, this, std::placeholders::_1, target.acceleration().segment(0, size), start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void inverseDynamicsImpl(const spatial::R<Params::quadratic_programming::nP()>& state, Eigen::Ref<const Eigen::VectorXd> targetAcc, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << _model->jacobian(state._x),
                    Eigen::MatrixXd::Zero(targetAcc.size(), _nC),
                    -Eigen::Matrix<double, Params::quadratic_programming::nS(), Params::quadratic_programming::nS()>::Identity();
                // std::cout << "hello" << std::endl;
                _opt._lbA.segment(start, size) = targetAcc - _model->jacobianDerivative(state._x, state._v) * state._v;
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            /* Inequality Constraints */
            QuadraticProgramming& positionLimits(const spatial::R<Params::quadratic_programming::nP()>& state)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = _nP;

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::positionLimitsImpl, this, std::placeholders::_1, start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void positionLimitsImpl(const spatial::R<Params::quadratic_programming::nP()>& state, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << 0.5 * std::pow(_dt, 2) * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(_nP, _nP),
                    Eigen::Matrix<double, Params::quadratic_programming::nP(), Params::quadratic_programming::nC() + Params::quadratic_programming::nS()>::Zero();

                _opt._lbA.segment(start, size) = _model->positionLower() - state._x - _dt * state._v;
                _opt._ubA.segment(start, size) = _model->positionUpper() - state._x - _dt * state._v;
            }

            QuadraticProgramming& velocityLimits(const spatial::R<Params::quadratic_programming::nP()>& state)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = _nP;

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::velocityLimitsImpl, this, std::placeholders::_1, start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void velocityLimitsImpl(const spatial::R<Params::quadratic_programming::nP()>& state, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << _dt * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(_nP, _nP),
                    Eigen::Matrix<double, Params::quadratic_programming::nP(), Params::quadratic_programming::nC() + Params::quadratic_programming::nS()>::Zero();

                _opt._lbA.segment(start, size) = _model->velocityLower() - state._v;
                _opt._ubA.segment(start, size) = _model->velocityUpper() - state._v;
            }

            /* Variable Bounds */
            QuadraticProgramming& accelerationLimits()
            {
                _opt._lb.segment(0, _nP) = _model->accelerationLower();
                _opt._ub.segment(0, _nP) = _model->accelerationUpper();

                return *this;
            }

            QuadraticProgramming& effortLimits()
            {
                _opt._lb.segment(_nP, _nP) = _model->effortLower();
                _opt._ub.segment(_nP, _nP) = _model->effortUpper();

                return *this;
            }

            QuadraticProgramming& init()
            {
                _opt.setRecalculation(100).init();
                return *this;
            }

            /* Control */
            void update(const spatial::R<Params::quadratic_programming::nP()>& state) override
            {
                // update objectives
                for (auto& objective : _objectives)
                    objective();

                // Update constraints
                for (auto& constraint : _constraints)
                    constraint(state);

                // Optimize
                _opt.setRecalculation(100).optimize();

                // Allocate solution
                _u = _opt.solution().segment(_nP, _nC);
            }

            // Optimizer
            optimization_lib::QpoasesOptimizer<qpOASES::SQProblem> _opt;

        protected:
            using AbstractController<Params, spatial::R<Params::quadratic_programming::nP()>>::_dt;
            using AbstractController<Params, spatial::R<Params::quadratic_programming::nP()>>::_u;
            using AbstractController<Params, spatial::R<Params::quadratic_programming::nP()>>::_d;

            // Problem and slack variable dimensions
            size_t _nP, _nC, _nS;

            // Model
            std::shared_ptr<Model> _model;

            // Objectives & Constraints
            std::vector<std::function<void()>> _objectives;
            std::vector<std::function<void(const spatial::R<Params::quadratic_programming::nP()>&)>> _constraints;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP