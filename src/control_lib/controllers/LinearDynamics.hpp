#ifndef CONTROLLIB_CONTROLLERS_LINEAR_DYNAMICS_HPP
#define CONTROLLIB_CONTROLLERS_LINEAR_DYNAMICS_HPP

#include "control_lib/controllers/AbstractController.hpp"

namespace control_lib {
    namespace controllers {
        class LinearDynamics : public AbstractController {
        public:
            LinearDynamics(ControlSpaces type, const size_t dim) : AbstractController(type, dim, dim)
            {
                _A = Eigen::MatrixXd::Identity(dim, dim);
            }

            const Eigen::MatrixXd& dynamicsMatrix() const { return _A; }

            LinearDynamics& setDynamicsMatrix(const Eigen::MatrixXd& A)
            {
                _A = A;
                return *this;
            }

            Eigen::VectorXd update(const Eigen::VectorXd& state)
            {
                _output = _A * (_input - _reference).getPos();
                return _output;
            }

        protected:
            Eigen::MatrixXd _A;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_LINEAR_DYNAMICS_HPP