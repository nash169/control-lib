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

#ifndef CONTROLLIB_CONTROLLERS_QUADRATICCONTROL_HPP
#define CONTROLLIB_CONTROLLERS_QUADRATICCONTROL_HPP

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
        struct quadratic_control {
            // State derivative order (0, 1, 2)
            PARAM_SCALAR(size_t, oD, 0);

            // State dimension
            PARAM_SCALAR(size_t, nP, 1);

            // Control input dimension (optional)
            PARAM_SCALAR(size_t, nC, 0);

            // Slack variable dimension (optional)
            PARAM_SCALAR(size_t, nS, 0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Model>
        class QuadraticControl : public AbstractController<Params, spatial::R<Params::quadratic_control::nP()>> {
        public:
            QuadraticControl()
            {
                // total problem dimension
                _d = Params::quadratic_control::nP() + Params::quadratic_control::nC() + Params::quadratic_control::nS();

                // init hessian matrix
                _opt.setHessianMatrix(Eigen::MatrixXd::Zero(_d, _d));

                // init gradient vector
                _opt.setGradientVector(Eigen::VectorXd::Zero(_d));

                // init lower and upper boundaries
                _opt.setVariablesBoundaries(-1e8 * Eigen::VectorXd::Ones(_d), 1e8 * Eigen::VectorXd::Ones(_d));
            }

            /* Model */
            QuadraticControl& setModel(const std::shared_ptr<Model>& model)
            {
                _model = model;
                return *this;
            }

            /* Objectives */
            QuadraticControl& stateCost(const Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nP()>& cost)
            {
                _opt._H.block(0, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) = cost;
                return *this;
            }

            QuadraticControl& stateReference(const Eigen::Ref<const Eigen::Matrix<double, Params::quadratic_control::nP(), 1>>& reference)
            {
                auto gradientVector = [&]() {
                    _opt._g.segment(0, Params::quadratic_control::nP()) = -2 * _opt._H.block(0, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) * reference;
                };

                _objectives.push_back(gradientVector);
                _objectives.back()();

                return *this;
            }

            QuadraticControl& inputCost(const Eigen::Matrix<double, Params::quadratic_control::nC(), Params::quadratic_control::nC()>& cost)
            {
                if constexpr (Params::quadratic_control::nC())
                    _opt._H.block(Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nC(), Params::quadratic_control::nC()) = cost;

                return *this;
            }

            QuadraticControl& inputReference(const Eigen::Ref<const Eigen::Matrix<double, Params::quadratic_control::nC(), 1>>& reference)
            {
                if constexpr (Params::quadratic_control::nC()) {
                    auto gradientVector = [&]() {
                        _opt._g.segment(Params::quadratic_control::nP(), Params::quadratic_control::nC()) = -2 * _opt._H.block(Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nC(), Params::quadratic_control::nC()) * reference;
                    };

                    _objectives.push_back(gradientVector);
                }

                return *this;
            }

            QuadraticControl& slackCost(const Eigen::Matrix<double, Params::quadratic_control::nS(), Params::quadratic_control::nS()>& cost)
            {
                if constexpr (Params::quadratic_control::nS())
                    _opt._H.block(Params::quadratic_control::nP() + Params::quadratic_control::nC(), Params::quadratic_control::nP() + Params::quadratic_control::nC(), Params::quadratic_control::nS(), Params::quadratic_control::nS()) = cost;

                return *this;
            }

            /* Equality Constraints */
            QuadraticControl& modelConstraint()
            {
                if constexpr (Params::quadratic_control::oD() == 2) {
                    // Constraints dimensions
                    size_t start = _opt._A.rows();

                    // Resize Constraint Matrix
                    _opt._A.conservativeResize(start + Params::quadratic_control::nP(), _d);
                    _opt._lbA.conservativeResize(start + Params::quadratic_control::nP());
                    _opt._ubA.conservativeResize(start + Params::quadratic_control::nP());

                    // Add constraint (it gets update every time step)
                    auto constraintFun = [&, start](const spatial::R<Params::quadratic_control::nP()>& state) {
                        _opt._A.block(start, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) = _model->inertiaMatrix(state._x);

                        if constexpr (Params::quadratic_control::nC())
                            _opt._A.block(start, Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nC()) = -_model->selectionMatrix();

                        if constexpr (Params::quadratic_control::nS())
                            _opt._A.block(start, Params::quadratic_control::nP() + Params::quadratic_control::nC(), Params::quadratic_control::nP(), Params::quadratic_control::nS()) = Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nS()>::Zero();

                        _opt._lbA.segment(start, Params::quadratic_control::nP()) = -_model->nonLinearEffects(state._x, state._v);
                        _opt._ubA.segment(start, Params::quadratic_control::nP()) = _opt._lbA.segment(start, Params::quadratic_control::nP());
                    };

                    _constraints.push_back(constraintFun);
                }

                return *this;
            }

            QuadraticControl& inverseKinematics(const Eigen::Ref<const Eigen::VectorXd>& reference)
            {
                if constexpr (Params::quadratic_control::oD() != 0) {
                    // Constraints dimensions
                    size_t start = _opt._A.rows(), size = reference.size();

                    // Resize Constraint Matrix
                    _opt._A.conservativeResize(start + size, _d);
                    _opt._lbA.conservativeResize(start + size);
                    _opt._ubA.conservativeResize(start + size);

                    // Add constraint (it gets update every time step)
                    auto constraintFun = [&, reference, start, size](const spatial::R<Params::quadratic_control::nP()>& state) {
                        if constexpr (Params::quadratic_control::oD() == 1) {
                            _opt._A.block(start, 0, size, Params::quadratic_control::nP()) = _model->jacobian(state._x);

                            if constexpr (Params::quadratic_control::nS())
                                _opt._A.block(start, Params::quadratic_control::nP(), size, Params::quadratic_control::nS()) = -Eigen::Matrix<double, Params::quadratic_control::nS(), Params::inverse_kinematics::nS()>::Identity();

                            _opt._lbA.segment(start, size) = reference;
                            _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
                        }
                        else if constexpr (Params::quadratic_control::oD() == 2) {
                            _opt._A.block(start, 0, size, Params::quadratic_control::nP()) = _dt * _model->jacobian(state._x);

                            if constexpr (Params::quadratic_control::nC())
                                _opt._A.block(start, Params::quadratic_control::nP(), size, Params::quadratic_control::nC()) = Eigen::MatrixXd::Zero(size, Params::quadratic_control::nC());

                            if constexpr (Params::quadratic_control::nS())
                                _opt._A.block(start, Params::quadratic_control::nP() + Params::quadratic_control::nC(), size, Params::quadratic_control::nS()) = -Eigen::Matrix<double, Params::quadratic_control::nS(), Params::quadratic_control::nS()>::Identity();

                            _opt._lbA.segment(start, size) = reference - _model->jacobian(state._x) * _pstate._v;
                            _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
                        }
                    };

                    _constraints.push_back(constraintFun);
                }

                return *this;
            }

            QuadraticControl& inverseDynamics(const Eigen::Ref<const Eigen::VectorXd>& reference)
            {
                if constexpr (Params::quadratic_control::oD() == 2) {
                    // Constraints dimensions
                    size_t start = _opt._A.rows(), size = reference.size();

                    // Resize Constraint Matrix
                    _opt._A.conservativeResize(start + size, _d);
                    _opt._lbA.conservativeResize(start + size);
                    _opt._ubA.conservativeResize(start + size);

                    // Add constraint (it gets update every time step)
                    auto constraintFun = [&, reference, start, size](const spatial::R<Params::quadratic_control::nP()>& state) {
                        _opt._A.block(start, 0, size, Params::quadratic_control::nP()) = _model->jacobian(state._x);

                        if constexpr (Params::quadratic_control::nC())
                            _opt._A.block(start, Params::quadratic_control::nP(), size, Params::quadratic_control::nC()) = Eigen::MatrixXd::Zero(size, Params::quadratic_control::nC());

                        if constexpr (Params::quadratic_control::nS())
                            _opt._A.block(start, Params::quadratic_control::nP() + Params::quadratic_control::nC(), size, Params::quadratic_control::nS()) = -Eigen::Matrix<double, Params::quadratic_control::nS(), Params::quadratic_control::nS()>::Identity();

                        _opt._lbA.segment(start, size) = reference - _model->jacobianDerivative(state._x, state._v) * state._v;
                        _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
                    };

                    _constraints.push_back(constraintFun);
                }

                return *this;
            }

            /* Inequality Constraints */
            QuadraticControl& positionLimits()
            {
                if constexpr (Params::quadratic_control::oD() == 0) {
                    _opt._lb.segment(0, Params::quadratic_control::nP()) = _model->positionLower();
                    _opt._ub.segment(0, Params::quadratic_control::nP()) = _model->positionUpper();
                }
                else {
                    // Constraints dimensions
                    size_t start = _opt._A.rows();

                    // Resize Constraint Matrix
                    _opt._A.conservativeResize(start + Params::quadratic_control::nP(), _d);
                    _opt._lbA.conservativeResize(start + Params::quadratic_control::nP());
                    _opt._ubA.conservativeResize(start + Params::quadratic_control::nP());

                    // Add constraint (it gets update every time step)
                    auto constraintFun = [&, start](const spatial::R<Params::quadratic_control::nP()>& state) {
                        if constexpr (Params::quadratic_control::oD() == 1) {
                            _opt._A.block(start, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) = _dt * Eigen::Matrix<double, Params::inverse_kinematics::nP(), Params::inverse_kinematics::nP()>::Identity();

                            if constexpr (Params::quadratic_control::nS())
                                _opt._A.block(start, Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nS()) = Eigen::Matrix<double, Params::inverse_kinematics::nP(), Params::inverse_kinematics::nS()>::Zero();

                            _opt._lbA.segment(start, Params::quadratic_control::nP()) = _model->positionLower() - _pstate._x;
                            _opt._ubA.segment(start, Params::quadratic_control::nP()) = _model->positionUpper() - _pstate._x;
                        }
                        else if constexpr (Params::quadratic_control::oD() == 2) {
                            _opt._A.block(start, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) = 0.5 * std::pow(_dt, 2) * Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nP()>::Identity();

                            if constexpr (Params::quadratic_control::nC())
                                _opt._A.block(start, Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nC()) = Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nC()>::Zero();

                            if constexpr (Params::quadratic_control::nS())
                                _opt._A.block(start, Params::quadratic_control::nP() + Params::quadratic_control::nC(), Params::quadratic_control::nP(), Params::quadratic_control::nS()) = Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nS()>::Zero();

                            _opt._lbA.segment(start, Params::quadratic_control::nP()) = _model->positionLower() - _pstate._x - _dt * _pstate._v;
                            _opt._ubA.segment(start, Params::quadratic_control::nP()) = _model->positionUpper() - _pstate._x - _dt * _pstate._v;
                        }
                    };

                    _constraints.push_back(constraintFun);
                }

                return *this;
            }

            QuadraticControl& velocityLimits()
            {
                if constexpr (Params::quadratic_control::oD() == 1) {
                    _opt._lb.segment(0, Params::quadratic_control::nP()) = _model->velocityLower();
                    _opt._ub.segment(0, Params::quadratic_control::nP()) = _model->velocityUpper();
                }
                else if constexpr (Params::quadratic_control::oD() == 2) {

                    // Constraints dimensions
                    size_t start = _opt._A.rows();

                    // Resize Constraint Matrix
                    _opt._A.conservativeResize(start + Params::quadratic_control::nP(), _d);
                    _opt._lbA.conservativeResize(start + Params::quadratic_control::nP());
                    _opt._ubA.conservativeResize(start + Params::quadratic_control::nP());

                    // Add constraint (it gets update every time step)
                    auto constraintFun = [&, start](const spatial::R<Params::quadratic_control::nP()>& state) {
                        _opt._A.block(start, 0, Params::quadratic_control::nP(), Params::quadratic_control::nP()) << _dt * Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nP()>::Identity();

                        if constexpr (Params::quadratic_control::nC())
                            _opt._A.block(start, Params::quadratic_control::nP(), Params::quadratic_control::nP(), Params::quadratic_control::nC()) = Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nC()>::Zero();

                        if constexpr (Params::quadratic_control::nS())
                            _opt._A.block(start, Params::quadratic_control::nP() + Params::quadratic_control::nC(), Params::quadratic_control::nP(), Params::quadratic_control::nS()) = Eigen::Matrix<double, Params::quadratic_control::nP(), Params::quadratic_control::nS()>::Zero();

                        _opt._lbA.segment(start, Params::quadratic_control::nP()) = _model->velocityLower() - _pstate._v;
                        _opt._ubA.segment(start, Params::quadratic_control::nP()) = _model->velocityUpper() - _pstate._v;
                    };

                    _constraints.push_back(constraintFun);
                }

                return *this;
            }

            /* Variable Bounds */
            QuadraticControl& accelerationLimits()
            {
                if constexpr (Params::quadratic_control::oD() == 2) {
                    _opt._lb.segment(0, Params::quadratic_control::nP()) = _model->accelerationLower();
                    _opt._ub.segment(0, Params::quadratic_control::nP()) = _model->accelerationUpper();
                }

                return *this;
            }

            QuadraticControl& effortLimits()
            {
                if constexpr (Params::quadratic_control::nC()) {
                    _opt._lb.segment(Params::quadratic_control::nC(), Params::quadratic_control::nC()) = _model->effortLower();
                    _opt._ub.segment(Params::quadratic_control::nC(), Params::quadratic_control::nC()) = _model->effortUpper();
                }

                return *this;
            }

            QuadraticControl& init(const spatial::R<Params::quadratic_control::nP()>& state)
            {
                // Record previous state
                _pstate = state;

                // update objectives
                for (auto& objective : _objectives)
                    objective();

                // Update constraints
                for (auto& constraint : _constraints)
                    constraint(state);

                // Optimize (for the first time)
                _opt.setRecalculation(100).init();

                // Allocate solution
                _u = _opt.solution();

                return *this;
            }

            /* Control */
            void update(const spatial::R<Params::quadratic_control::nP()>& state) override
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
                _u = _opt.solution();

                // Record previous state
                _pstate = state;
            }

            // Optimizer
            optimization_lib::QpoasesOptimizer<qpOASES::SQProblem> _opt;

        protected:
            using AbstractController<Params, spatial::R<Params::quadratic_control::nP()>>::_dt;
            using AbstractController<Params, spatial::R<Params::quadratic_control::nP()>>::_u;
            using AbstractController<Params, spatial::R<Params::quadratic_control::nP()>>::_d;

            // Model
            std::shared_ptr<Model> _model;

            // State (it keeps track of the previous state)
            spatial::R<Params::quadratic_control::nP()> _pstate;

            // Objectives & Constraints
            std::vector<std::function<void()>> _objectives;
            std::vector<std::function<void(const spatial::R<Params::quadratic_control::nP()>&)>> _constraints;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICCONTROL_HPP