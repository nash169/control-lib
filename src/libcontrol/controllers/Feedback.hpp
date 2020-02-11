#ifndef LIBCONTROL_CONTROLLERS_FEEDBACK_HPP
#define LIBCONTROL_CONTROLLERS_FEEDBACK_HPP

#include <libcontrol/AbstractController.hpp>

namespace libcontrol {
    namespace controllers {
        struct Gains {
            Eigen::MatrixXd p_matrix,
                d_matrix,
                i_matrix;
        };

        class Feedback : public AbstractController {
        public:
            Feedback(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01);
            Feedback() {}
            ~Feedback() {}

            void setGains(const std::string& type, const Eigen::MatrixXd& mat);

            Eigen::VectorXd update(const Eigen::VectorXd& state);

        protected:
            ControlState _error;

            Eigen::VectorXd _integral_error;

            Gains _gains;
        };
    } // namespace controllers
} // namespace libcontrol

#endif // LIBCONTROL_CONTROLLERS_FEEDBACK_HPP