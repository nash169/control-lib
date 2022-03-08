#ifndef CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP
#define CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP

#include "control_lib/spatial/SE3.hpp"
#include "control_lib/tools/macros.hpp"
#include <Eigen/Core>
#include <memory>

namespace control_lib {
    namespace defaults {
        struct controller {
            // Output space dimension (by default SE3 dimensionality)
            PARAM_SCALAR(double, d, 6);

            // Integration time step controller
            PARAM_SCALAR(double, dt, 0.001);

            // Reference vector at the moment by default in SE3
            PARAM_VECTOR(double, xr, 0, 0, 0, 0, 0, 0);
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space>
        class AbstractController {
        public:
            AbstractController() : _d(Params::controller::d()),
                                   _dt(Params::controller::dt()),
                                   _xr(Params::controller::xr()),
                                   _u(Eigen::VectorXd::Zero(_d)) {}

            const size_t& dimension() const { return _d; };

            const double& step() const { return _dt; };

            const Space& reference() const { return _xr; };

            AbstractController& setDimension(const size_t& d)
            {
                _d = d;
                return *this;
            }

            AbstractController& setStep(const double& dt)
            {
                _dt = dt;
                return *this;
            }

            AbstractController& setReference(const Space& x)
            {
                _xr = x;
                return *this;
            }

            virtual void update(const Space& x) = 0;

            const Eigen::VectorXd& action(const Space& x)
            {
                update(x);
                return _u;
            }

        protected:
            // Output dimension
            size_t _d;

            // Integration time step
            double _dt;

            // Operation space
            Space _xr;

            // Control state
            Eigen::VectorXd _u;
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_ABSTRACCONTROLLER_HPP