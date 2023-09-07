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
        struct inverse_kinematics {
            // State dimension
            PARAM_SCALAR(size_t, nP, 1);

            // Slack variable dimension
            PARAM_SCALAR(size_t, nS, 0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Model>
        class InverseKinematics : public AbstractController<Params, spatial::R<Params::inverse_kinematics::nP()>> {
        public:
            InverseKinematics() : _nP(Params::inverse_kinematics::nP()), _nS(Params::inverse_kinematics::nS())
            {
                // total problem dimension
                _d = _nP + _nS;

                // init hessian matrix
                _opt.setHessianMatrix(Eigen::MatrixXd::Zero(_d, _d));

                // init gradient vector
                _opt.setGradientVector(Eigen::VectorXd::Zero(_d));

                // init lower and upper boundaries
                _opt.setVariablesBoundaries(-1e8 * Eigen::VectorXd::Ones(_d), 1e8 * Eigen::VectorXd::Ones(_d));
            }

            /* Model
               This is necessary only to extract the Jacobian of the model.
            */
            InverseKinematics& setModel(const std::shared_ptr<Model>& model)
            {
                _model = model;
                return *this;
            }

            /* Objectives */
            InverseKinematics& velocityMinimization(Eigen::MatrixXd& Q)
            {
                _opt._H.block(0, 0, _nP, _nP) = Q;
                return *this;
            }

            template <typename Target>
            InverseKinematics& velocityTracking(const Eigen::MatrixXd& Q, const Target& target)
            {
                // Add objective
                _objectives.push_back(
                    std::bind(&InverseKinematics<Params, Model>::velocityTrackingImpl, this,
                        Q, target.output().segment(0, target.output().size()))); // hacky way to extract the reference to the variable (check if it can be improved)

                // Run objective
                _objectives.back()();

                return *this;
            }

            void velocityTrackingImpl(const Eigen::MatrixXd& Q, Eigen::Ref<const Eigen::VectorXd> target)
            {
                // std::cout << "qp" << std::endl;
                // std::cout << target.transpose() << std::endl;
                _opt._g.segment(0, _nP) = -2 * Q * target;
            }

            InverseKinematics& slackVariable(Eigen::MatrixXd& W)
            {
                _opt._H.block(_nP, _nP, _nS, _nS) = W;
                return *this;
            }

            /* Equality Constraints */
            template <typename Target>
            InverseKinematics& inverseKinematics(const spatial::R<Params::inverse_kinematics::nP()>& state, const Target& target)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = target.output().size();

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&InverseKinematics<Params, Model>::inverseKinematicsImpl, this, std::placeholders::_1, target.output().segment(0, size), start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void inverseKinematicsImpl(const spatial::R<Params::inverse_kinematics::nP()>& state, Eigen::Ref<const Eigen::VectorXd> target, const size_t& start, const size_t& size)
            {
                // std::cout << "qp" << std::endl;
                // std::cout << targetVel.transpose() << std::endl;
                _opt._A.block(start, 0, size, _d) << _model->jacobian(state._x), -Eigen::Matrix<double, Params::inverse_kinematics::nS(), Params::inverse_kinematics::nS()>::Identity();

                _opt._lbA.segment(start, size) = target;
                _opt._ubA.segment(start, size) = _opt._lbA.segment(start, size);
            }

            /* Inequality Constraints */
            InverseKinematics& positionLimits(const spatial::R<Params::inverse_kinematics::nP()>& state)
            {
                // Constraints dimensions
                size_t start = _opt._A.rows(), size = _nP;

                // Resize Constraint Matrix
                _opt._A.conservativeResize(start + size, _d);
                _opt._lbA.conservativeResize(start + size);
                _opt._ubA.conservativeResize(start + size);

                // Add constraint (it gets update every time step)
                _constraints.push_back(std::bind(&InverseKinematics<Params, Model>::positionLimitsImpl, this, std::placeholders::_1, start, size));

                // First matrix filling
                _constraints.back()(state);

                return *this;
            }

            void positionLimitsImpl(const spatial::R<Params::inverse_kinematics::nP()>& state, const size_t& start, const size_t& size)
            {
                _opt._A.block(start, 0, size, _d) << _dt * Eigen::Matrix<double, Params::inverse_kinematics::nP(), Params::inverse_kinematics::nP()>::Identity(),
                    Eigen::Matrix<double, Params::inverse_kinematics::nP(), Params::inverse_kinematics::nS()>::Zero();

                _opt._lbA.segment(start, size) = _model->positionLower() - state._x;
                _opt._ubA.segment(start, size) = _model->positionUpper() - state._x;
            }

            /* Variable Bounds */
            InverseKinematics& velocityLimits()
            {
                _opt._lb.segment(0, _nP) = _model->velocityLower();
                _opt._ub.segment(0, _nP) = _model->velocityUpper();

                return *this;
            }

            InverseKinematics& init()
            {
                _opt.setRecalculation(100).init();
                return *this;
            }

            /* Control */
            void update(const spatial::R<Params::inverse_kinematics::nP()>& state) override
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
            }

            // Optimizer (temporary public for debugging)
            optimization_lib::QpoasesOptimizer<qpOASES::SQProblem> _opt;

        protected:
            using AbstractController<Params, spatial::R<Params::inverse_kinematics::nP()>>::_dt;
            using AbstractController<Params, spatial::R<Params::inverse_kinematics::nP()>>::_u;
            using AbstractController<Params, spatial::R<Params::inverse_kinematics::nP()>>::_d;

            // Problem and slack variable dimensions
            size_t _nP, _nS;

            // Model
            std::shared_ptr<Model> _model;

            // Objectives & Constraints
            std::vector<std::function<void()>> _objectives;
            std::vector<std::function<void(const spatial::R<Params::inverse_kinematics::nP()>&)>> _constraints;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP