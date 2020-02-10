#include "libcontrol/controllers/Feedback.hpp"

#include <iostream>

namespace libcontrol {
    namespace controllers {
        Feedback::Feedback(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step) : AbstractController(type, input_dim, output_dim, time_step)
        {
            // This might be moved in the abstract class
            _error = std::unique_ptr<ControlState>(new ControlState(input_dim, type));

            // Init integral error
            _integral_error = Eigen::VectorXd::Zero(input_dim);
        }

        Feedback::Feedback()
        {
        }

        Feedback::~Feedback()
        {
        }

        void Feedback::setGains(const std::string& type, const Eigen::MatrixXd& mat)
        {
            if (!type.compare("p"))
                _gains.p_matrix = std::make_unique<Eigen::MatrixXd>(mat);
            else if (!type.compare("d"))
                _gains.d_matrix = std::make_unique<Eigen::MatrixXd>(mat);
            else
                _gains.i_matrix = std::make_unique<Eigen::MatrixXd>(mat);
        }

        Eigen::VectorXd Feedback::update(const Eigen::VectorXd& state)
        {

            // This might change if we integrate the delay inside the controller
            _output.setZero();

            // Memorize position error for the integral part
            auto error_prev = _error->getPos();

            // This should be speeded up using the information from the reference
            setInput(state);

            // Update error
            _error = std::make_unique<ControlState>(*_input - *_reference);

            if (_gains.p_matrix != nullptr)
                _output += -*_gains.p_matrix * _error->getPos();

            if (_gains.d_matrix != nullptr)
                _output += -*_gains.d_matrix * _error->_velocity.value();

            if (_gains.i_matrix != nullptr) {
                _integral_error += (error_prev + _error->getPos()) * _time_step / 2;
                _output += -*_gains.i_matrix * _integral_error;
            }

            return _output;
        }
    } // namespace controllers
} // namespace libcontrol
