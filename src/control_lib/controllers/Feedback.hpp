#ifndef CONTROLLIB_CONTROLLERS_FEEDBACK_HPP
#define CONTROLLIB_CONTROLLERS_FEEDBACK_HPP

#include "control_lib/controllers/AbstractController.hpp"
#include <type_traits>

namespace control_lib {
    namespace defaults {
        struct feedback {
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space = spatial::SE3>
        class Feedback : public AbstractController<Params, Space> {
        public:
            Feedback() : AbstractController<Params, Space>() {}

            Feedback& setStiffness(const Eigen::MatrixXd& K)
            {
                _K = K;
                return *this;
            }

            Feedback& setDamping(const Eigen::MatrixXd& D)
            {
                _D = D;
                return *this;
            }

            Feedback& setIntegral(const Eigen::MatrixXd& I)
            {
                _I = I;

                _err.setZero();
                _eng.setZero();

                return *this;
            }

            void update(const Space& x) override
            {
                _u.setZero();
                Eigen::Matrix<double, Space::dimension(), 1> err;

                // Proportional
                if (_K.size()) {
                    err = (x - _xr);
                    _u -= _K * err;
                }

                // Derivative
                if (_D.size())
                    _u -= _D * (x._vel - _xr._vel);

                // Integral
                if (_I.size()) {
                    if (!err.size())
                        err = (x - _xr);
                    _eng += (_err + err) * _dt * 0.5;
                    _u -= _I * _eng;
                    _err = err;
                }
            }

        protected:
            using AbstractController<Params, Space>::_d;
            using AbstractController<Params, Space>::_dt;
            using AbstractController<Params, Space>::_xr;
            using AbstractController<Params, Space>::_u;

            Eigen::MatrixXd _K, _D, _I;

            // Error and energy
            Eigen::Matrix<double, Space::dimension(), 1> _err, _eng;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_FEEDBACK_HPP

// template <typename... Args>
// Eigen::VectorXd action(Args&&... args)
// {
//     // Keep track of the iteration
//     size_t i = 0;

//     // Action vector and error
//     Eigen::VectorXd u = Eigen::VectorXd::Zero(_d);
//     Eigen::Matrix<double, Space::dimension(), 1> err;

//     // Generate control action
//     ([&](auto& arg) {
//         if constexpr (std::is_same_v<std::remove_reference_t<decltype(arg)>, Space>) {
//             // Proportional
//             if (_K.size()) {
//                 err = (arg - AbstractController<Params, Space>::_xr).log();
//                 u -= _K * err;
//             }

//             if (_I.size()) {
//                 if (!err.size())
//                     err = (arg - AbstractController<Params, Space>::_xr).log();
//                 _eng += (_err + err) * AbstractController<Params, Space>::_dt * 0.5;
//                 u -= _I * _eng;
//                 _err = err;
//             }
//         }
//         else if (_D.size()) {
//             u -= _D * (arg - AbstractController<Params, Space>::_xr.velocity());
//         }

//         // Increment
//         ++i;
//     }(args),
//         ...);

//     return u;
// }
