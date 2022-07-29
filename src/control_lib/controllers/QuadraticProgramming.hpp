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
#include <vector>

#include <optimization_lib/QpoasesOptimizer.hpp>

#include "control_lib/controllers/AbstractController.hpp"
#include "control_lib/spatial/RN.hpp"

namespace control_lib {
    namespace defaults {
        struct quadratic_programming {
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space = spatial::SE3>
        class QuadraticProgramming : public AbstractController<Params, Space> {
        public:
            QuadraticProgramming(const size_t& nP, const size_t& nS = 0)
            {
                setDimensions(nP, nS);
            }

            QuadraticProgramming() = default;

            QuadraticProgramming& setDimensions(const size_t& nP, const size_t& nS = 0)
            {
                _nP = nP;
                _nS = nS;
                _opt.setHessianMatrix(Eigen::MatrixXd::Zero(2 * nP + nS, 2 * nP + nS));
                _opt.setGradientVector(Eigen::VectorXd::Zero(2 * nP + nS));
                _opt.setVariablesBoundaries(-1e8 * Eigen::VectorXd::Ones(2 * nP + nS), 1e8 * Eigen::VectorXd::Ones(2 * nP + nS));
                return *this;
            }

            /* Objectives */
            QuadraticProgramming& energyMinimization(const Eigen::MatrixXd& Q)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS);
                Eigen::VectorXd b = Eigen::VectorXd::Zero(_nP);
                A.block(0, 0, _nP, _nP) = Q;

                addObjective(A, b);

                // std::cout << _opt.hessianMatrix() << std::endl;
                // std::cout << _opt.gradientVector() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            template <typename Target>
            QuadraticProgramming& dynamicsTracking(const Eigen::MatrixXd& Q, const Target& target)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS);
                Eigen::VectorXd b = Q * target.acceleration();
                A.block(0, 0, _nP, _nP) = Q;

                addObjective(A, b);

                // std::cout << _opt.hessianMatrix() << std::endl;
                // std::cout << _opt.gradientVector() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            QuadraticProgramming& controlSaturation(const Eigen::MatrixXd& R)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS);
                Eigen::VectorXd b = Eigen::VectorXd::Zero(_nP);
                A.block(0, _nP, _nP, _nP) = R;

                addObjective(A, b);

                // std::cout << _opt.hessianMatrix() << std::endl;
                // std::cout << _opt.gradientVector() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            QuadraticProgramming& slackVariable()
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS);
                Eigen::VectorXd b = Eigen::VectorXd::Zero(_nP);
                A.block(0, 2 * _nP, _nS, _nS) = Eigen::MatrixXd::Identity(_nS, _nS);

                addObjective(A, b);

                // std::cout << _opt.hessianMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.gradientVector().transpose() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            /* Equality Constraints */
            template <typename Model>
            QuadraticProgramming& modelDynamics(Model& model)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS);
                Eigen::VectorXd b = model.nonLinearEffects();

                A.block(0, 0, _nP, _nP) = model.inertiaMatrix();
                A.block(0, _nP, _nP, _nP) = model.selectionMatrix();

                addConstraint(A, b, b);

                // std::cout << A << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << b.transpose() << std::endl;
                // std::cout << _opt.constraintsMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.lowerConstraints().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperConstraints().transpose() << std::endl;

                return *this;
            }

            template <typename Model, typename Target>
            QuadraticProgramming& inverseKinematics(Model& model, const Target& target)
            {
                Eigen::VectorXd b = target.velocity() - model.jacobian() * model.velocity();
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(b.size(), 2 * _nP + _nS);

                A.block(0, 0, A.rows(), _nP) = _dt * model.jacobian();
                if (_nS)
                    A.block(0, 2 * _nP, _nS, _nS) = -Eigen::MatrixXd::Identity(_nS, _nS);

                addConstraint(A, b, b);

                // std::cout << A << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << b.transpose() << std::endl;
                // std::cout << _opt.constraintsMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.lowerConstraints().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperConstraints().transpose() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            template <typename Model, typename Target>
            QuadraticProgramming& inverseDynamics(Model& model, const Target& target)
            {
                Eigen::VectorXd b = target.acceleration() - model.jacobianDerivative() * model.velocity();
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(b.size(), 2 * _nP + _nS);

                A.block(0, 0, A.rows(), _nP) = model.jacobian();
                if (_nS)
                    A.block(0, 2 * _nP, _nS, _nS) = -Eigen::MatrixXd::Identity(_nS, _nS);

                addConstraint(A, b, b);

                // std::cout << A << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << b.transpose() << std::endl;
                // std::cout << _opt.constraintsMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.lowerConstraints().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperConstraints().transpose() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            /* Inequality Constraints */
            template <typename Model>
            QuadraticProgramming& positionLimits(Model& model)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS),
                                b(_nP, 2);
                A.block(0, 0, _nP, _nP) = 0.5 * std::pow(_dt, 2) * Eigen::MatrixXd::Identity(_nP, _nP);
                b.col(0) = model.positionLower() - model.state() - _dt * model.velocity();
                b.col(1) = model.positionUpper() - model.state() - _dt * model.velocity();

                addConstraint(A, b.col(0), b.col(1));

                // std::cout << A << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << b.transpose() << std::endl;
                // std::cout << _opt.constraintsMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.lowerConstraints().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperConstraints().transpose() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            template <typename Model>
            QuadraticProgramming& velocityLimits(Model& model)
            {
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(_nP, 2 * _nP + _nS),
                                b(_nP, 2);
                A.block(0, 0, _nP, _nP) = _dt * Eigen::MatrixXd::Identity(_nP, _nP);
                b.col(0).segment(0, _nP) = model.velocityLower() - model.velocity();
                b.col(1).segment(0, _nP) = model.velocityUpper() - model.velocity();

                addConstraint(A, b.col(0), b.col(1));

                // std::cout << A << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << b.transpose() << std::endl;
                // std::cout << _opt.constraintsMatrix() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.lowerConstraints().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperConstraints().transpose() << std::endl;

                // return std::make_pair(A, b);
                return *this;
            }

            /* Variable Bounds */
            template <typename Model>
            QuadraticProgramming& accelerationLimits(Model& model)
            {
                _opt.lowerBounds().segment(0, _nP) = model.accelerationLower();
                _opt.upperBounds().segment(0, _nP) = model.accelerationUpper();

                // std::cout << _opt.lowerBounds().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperBounds().transpose() << std::endl;

                return *this;
            }

            template <typename Model>
            QuadraticProgramming& effortLimits(Model& model)
            {
                _opt.lowerBounds().segment(_nP, _nP) = model.effortLower();
                _opt.upperBounds().segment(_nP, _nP) = model.effortUpper();

                // std::cout << _opt.lowerBounds().transpose() << std::endl;
                // std::cout << "-" << std::endl;
                // std::cout << _opt.upperBounds().transpose() << std::endl;

                return *this;
            }

            QuadraticProgramming& init()
            {
                _opt.setRecalculation(100).init();
                return *this;
            }

            /* Control */
            void update(const Space& x) override
            {
                _opt.setRecalculation(100).optimize();
                _u = _opt.solution().segment(_nP, _nP);
            }

        protected:
            using AbstractController<Params, Space>::_dt;
            using AbstractController<Params, Space>::_u;

            // Problem and slack variable dimensions
            size_t _nP, _nS;

            // Optimizer
            optimization_lib::QpoasesOptimizer<qpOASES::SQProblem> _opt;

            void addObjective(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
            {
                _opt.hessianMatrix() += A.transpose() * A;
                _opt.gradientVector() -= A.transpose() * b;
            }

            void addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
            {
                size_t rows = _opt.constraintsMatrix().rows();
                _opt.constraintsMatrix().conservativeResize(rows + A.rows(), 2 * _nP + _nS);
                _opt.constraintsMatrix().block(rows, 0, A.rows(), 2 * _nP + _nS) = A;
                _opt.lowerConstraints().conservativeResize(rows + A.rows());
                _opt.lowerConstraints().segment(rows, A.rows()) = lower;
                _opt.upperConstraints().conservativeResize(rows + A.rows());
                _opt.upperConstraints().segment(rows, A.rows()) = upper;
            }
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_QUADRATICPROGRAMMING_HPP