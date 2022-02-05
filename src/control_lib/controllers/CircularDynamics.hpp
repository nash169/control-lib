#ifndef CONTROLLIB_CONTROLLERS_CIRCULAR_DYNAMICS_HPP
#define CONTROLLIB_CONTROLLERS_CIRCULAR_DYNAMICS_HPP

#include "control_lib/controllers/AbstractController.hpp"

namespace control_lib {
    namespace controllers {
        class CircularDynamics : public AbstractController {
        public:
            CircularDynamics(ControlSpaces type, const size_t dim) : AbstractController(type, dim, dim)
            {
                _rho = 1;
            }

            const double& dynamicsMatrix() const { return _rho; }

            CircularDynamics& setDynamicsMatrix(const double& rho)
            {
                _rho = rho;
                return *this;
            }

            Eigen::VectorXd update(const Eigen::VectorXd& state)
            {
                return _output;
            }

        protected:
            double _rho;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_CIRCULAR_DYNAMICS_HPP