#ifndef CONTROLLIB_CONTROLLERS_FEEDBACK2_HPP
#define CONTROLLIB_CONTROLLERS_FEEDBACK2_HPP

#include <type_traits>

#include "control_lib/controllers/AbstractController2.hpp"

namespace control_lib {
    namespace defaults {
        struct feedback {
        };
    } // namespace defaults

    namespace controllers {
        template <typename Params, typename Space>
        class Feedback2 : public AbstractController2<Params, Space> {
        public:
            Feedback2() : AbstractController2<Params, Space>() {}

            Feedback2& setStiffness(const Eigen::MatrixXd& K)
            {
                _K = K;
                return *this;
            }

            Feedback2& setDamping(const Eigen::MatrixXd& D)
            {
                _D = D;
                return *this;
            }

            Feedback2& setIntegral(const Eigen::MatrixXd& I)
            {
                _I = I;

                _err.setZero();
                _eng.setZero();

                return *this;
            }

            template <typename... Args>
            Eigen::VectorXd action(Args&&... args)
            {
                // Keep track of the iteration
                size_t i = 0;

                // Action vector and error
                Eigen::VectorXd u = Eigen::VectorXd::Zero(_d);
                Eigen::Matrix<double, Space::dimension(), 1> err;

                // Generate control action
                ([&](auto& arg) {
                    if constexpr (std::is_same_v<std::remove_reference_t<decltype(arg)>, Space>) {
                        // Proportional
                        if (_K.size()) {
                            // std::cout << "iteration: " << i << std::endl;
                            // std::cout << "proportional" << std::endl;
                            err = (arg - _xr).log();
                            u -= _K * err;
                        }

                        if (_I.size()) {
                            // std::cout << "iteration: " << i << std::endl;
                            // std::cout << "integral" << std::endl;
                            if (!err.size())
                                err = (arg - _xr).log();
                            _eng += (_err + err) * _dt * 0.5;
                            u -= _I * _eng;
                            _err = err;
                        }
                    }
                    else if (_D.size()) {
                        // std::cout << "iteration: " << i << std::endl;
                        // std::cout << "derivative" << std::endl;
                        u -= _D * (arg - _xr.velocity());
                    }

                    // Increment
                    ++i;
                }(args),
                    ...);

                return u;
            }

        protected:
            using AbstractController2<Params, Space>::_d;

            using AbstractController2<Params, Space>::_dt;

            using AbstractController2<Params, Space>::_xr;

            Eigen::MatrixXd _K, _D, _I;

            // Error and energy
            Eigen::Matrix<double, Space::dimension(), 1> _err, _eng;
        };
    } // namespace controllers

} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_FEEDBACK2_HPP