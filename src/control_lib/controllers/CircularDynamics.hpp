#ifndef CONTROLLIB_CONTROLLERS_CIRCULARDYNAMICS_HPP
#define CONTROLLIB_CONTROLLERS_CIRCULARDYNAMICS_HPP

#include "control_lib/controllers/AbstractController.hpp"

namespace control_lib {
    namespace defaults {
        struct circular_dynamics {
            PARAM_SCALAR(double, rho, 1);
        };

    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space = spatial::SE3>
        class CircularDynamics : public AbstractController<Params, Space> {
        public:
            CircularDynamics() : AbstractController<Params, Space>(), _rho(Params::circular_dynamics::rho()) {}

            const double& radius() const { return _rho; }

            CircularDynamics& setRadius(const double& rho)
            {
                _rho = rho;
                return *this;
            }

            void update(const Space& x) override
            {
            }

        protected:
            double _rho;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_CIRCULARDYNAMICS_HPP