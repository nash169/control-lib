#ifndef CONTROLLIB_CONTROLLERS_LQR_HPP
#define CONTROLLIB_CONTROLLERS_LQR_HPP

#include "control_lib/controllers/Feedback.hpp"

namespace control_lib {
    namespace controllers {
        class Lqr : public Feedback {
        public:
            Lqr(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01) : Feedback(type, input_dim, output_dim, time_step)
            {
            }
            Lqr()
            {
            }

            void optimalGains(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
            {
            }

        protected:
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_LQR_HPP