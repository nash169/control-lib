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
#include "control_lib/spatial/RN.hpp"

using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

namespace control_lib {
    namespace defaults {
        struct quadratic_programming {
            // State dimension
            PARAM_SCALAR(size_t, nV, 1);

            // Control input dimension
            PARAM_SCALAR(size_t, nC, 1);

            // Slack variable dimension
            PARAM_SCALAR(size_t, nS, 0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Model>
        class QuadraticProgramming : public AbstractController<Params, spatial::RN<Params::quadratic_programming::nV()>> {
        public:
            QuadraticProgramming() : _nP(Params::quadratic_programming::nV()), _nC(Params::quadratic_programming::nC()), _nS(Params::quadratic_programming::nS())
            {
                // total problem dimension
                _d = _nP + _nC + _nS;

                // init hessian matrix
                _opt.setHessianMatrix(MatrixRowMajor::Zero(_d, _d));

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
            QuadraticProgramming& energyMinimization(const Eigen::MatrixXd& Q)
            {
                _opt._H.block(0, 0, _nP, _nP) = Eigen::Map<MatrixRowMajor>(Q.data(), _nP, _nP);
                return *this;
            }

            template <typename Target>
            QuadraticProgramming& dynamicsTracking(const Eigen::MatrixXd& Q, const Target& target)
            {
                _opt._H.block(0, 0, _nP, _nP) = Eigen::Map<MatrixRowMajor>(Q.data(), _nP, _nP);
                _opt._g.segment(0, _nP) = -2 * Q * target.acceleration();
                return *this;
            }

            QuadraticProgramming& controlSaturation(const Eigen::MatrixXd& R)
            {
                _opt._H.block(_nP, _nP, _nC, _nC) = Eigen::Map<MatrixRowMajor>(R.data(), _nC, _nC);
                return *this;
            }

            QuadraticProgramming& slackVariable(const Eigen::MatrixXd& W)
            {
                _opt._H.block(_nP + _nC, _nP + _nC, _nS, _nS) = Eigen::Map<MatrixRowMajor>(W.data(), _nS, _nS);
                return *this;
            }

            /* Equality Constraints */
            QuadraticProgramming& modelDynamics(const spatial::RN<Params::quadratic_programming::nV()>& state)
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

            void modelDynamicsImpl(const spatial::RN<Params::quadratic_programming::nV()>& state, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << Eigen::Map<MatrixRowMajor>(_model->inertiaMatrix(state._pos).data(), _nP, _nP),
                    Eigen::Map<MatrixRowMajor>(_model->selectionMatrix().data(), _nC, _nC),
                    Eigen::Matrix<double, Params::quadratic_programming::nV(), Params::quadratic_programming::nS(), Eigen::RowMajor>::Zero();

                _opt._lbA.segment(start, size) = _model->nonLinearEffects(state._pos, state._vel);
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            template <typename Target>
            QuadraticProgramming& inverseKinematics(const spatial::RN<Params::quadratic_programming::nV()>& state, const Target& target)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = target.dim();

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::inverseKinematicsImpl, this, std::placeholders::_1, target.velocity(), start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void inverseKinematicsImpl(const spatial::RN<Params::quadratic_programming::nV()>& state, Eigen::Ref<const Eigen::VectorXd> targetVel, const size_t& start, const size_t& size)
            {
                std::cout << "hello" << std::endl;
                _opt._A.block(start, 0, size, _d) << _dt * Eigen::Map<MatrixRowMajor>(_model->jacobian(state._pos).data(), targetVel.size(), _nP),
                    MatrixRowMajor::Zero(targetVel.size(), _nP),
                    -Eigen::Matrix<double, Params::quadratic_programming::nS(), Params::quadratic_programming::nS(), Eigen::RowMajor>::Identity();

                _opt._lbA.segment(start, size) = targetVel - _model->jacobian(state._pos) * _model->velocity(); // _model->velocity() being the velocity at the previous step
                std::cout << "hello" << std::endl;
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            template <typename Target>
            QuadraticProgramming& inverseDynamics(const spatial::RN<Params::quadratic_programming::nV()>& state, const Target& target)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = target.dim();

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&QuadraticProgramming<Params, Model>::inverseDynamicsImpl, this, std::placeholders::_1, target.acceleration(), start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void inverseDynamicsImpl(const spatial::RN<Params::quadratic_programming::nV()>& state, Eigen::Ref<const Eigen::VectorXd> targetAcc, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << Eigen::Map<MatrixRowMajor>(_model->jacobian(state._pos).data(), targetAcc.size(), _nP),
                    MatrixRowMajor::Zero(targetAcc.size(), _nC),
                    -Eigen::Matrix<double, Params::quadratic_programming::nS(), Params::quadratic_programming::nS(), Eigen::RowMajor>::Identity();

                _opt._lbA.segment(start, size) = targetAcc - _model->jacobianDerivative(state._pos, state._vel) * state._vel;
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            /* Inequality Constraints */
            QuadraticProgramming& positionLimits(const spatial::RN<Params::quadratic_programming::nV()>& state)
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

            void positionLimitsImpl(const spatial::RN<Params::quadratic_programming::nV()>& state, Eigen::Ref<const Eigen::VectorXd> targetAcc, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << 0.5 * std::pow(_dt, 2) * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Identity(_nP, _nP),
                    Eigen::Matrix<double, Params::quadratic_programming::nP(), Params::quadratic_programming::nC() + Params::quadratic_programming::nS(), Eigen::RowMajor>::Zero();

                _opt._lbA.segment(start, size) = _model->positionLower() - state._pos - _dt * state._vel;
                _opt._ubA.segment(start, size) = _model->positionUpper() - state._pos - _dt * state._vel;
            }

            QuadraticProgramming& velocityLimits(const spatial::RN<Params::quadratic_programming::nV()>& state)
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

            void velocityLimitsImpl(const spatial::RN<Params::quadratic_programming::nV()>& state, Eigen::Ref<const Eigen::VectorXd> targetAcc, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << _dt * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Identity(_nP, _nP),
                    Eigen::Matrix<double, Params::quadratic_programming::nP(), Params::quadratic_programming::nC() + Params::quadratic_programming::nS(), Eigen::RowMajor>::Zero();

                _opt._lbA.segment(start, size) = _model->velocityLower() - state._vel;
                _opt._ubA.segment(start, size) = _model->velocityUpper() - state._vel;
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
            void update(const spatial::RN<Params::quadratic_programming::nV()>& state) override
            {
                // Update constraints
                for (auto& constraint : _constraints)
                    constraint(state);

                // Optimize
                _opt.setRecalculation(100).optimize();

                // Allocate solution
                _u = _opt.solution().segment(_nP, _nP);
            }

            // Optimizer
            optimization_lib::QpoasesOptimizer<qpOASES::SQProblem> _opt;

        protected:
            using AbstractController<Params, spatial::RN<Params::quadratic_programming::nV()>>::_dt;
            using AbstractController<Params, spatial::RN<Params::quadratic_programming::nV()>>::_u;
            using AbstractController<Params, spatial::RN<Params::quadratic_programming::nV()>>::_d;

            // Problem and slack variable dimensions
            size_t _nP, _nC, _nS;

            // Model
            std::shared_ptr<Model> _model;

            // Constraints
            std::vector<std::function<void(const spatial::RN<Params::quadratic_programming::nV()>&)>> _constraints;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP