#ifndef CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP
#define CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP

#include "control_lib/controllers/AbstractController.hpp"
#include <Eigen/Core>

namespace control_lib {
    namespace defaults {
        struct linear_dynamics {
        };

    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space = spatial::SE3>
        class LinearDynamics : public AbstractController<Params, Space> {
        public:
            LinearDynamics() : AbstractController<Params, Space>()
            {
                _A = Eigen::MatrixXd::Identity(Space::dimension(), _d);
            }

            const Eigen::MatrixXd& dynamicsMatrix() const { return _A; }

            LinearDynamics& setDynamicsMatrix(const Eigen::MatrixXd& A)
            {
                _A = A;
                return *this;
            }

            void update(const Space& x) override
            {
                _u = _A * (_xr - x);
            }

        protected:
            using AbstractController<Params, Space>::_d;
            using AbstractController<Params, Space>::_xr;
            using AbstractController<Params, Space>::_u;

            Eigen::MatrixXd _A;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_LINEARDYNAMICS_HPP