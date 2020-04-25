#ifndef CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP
#define CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP

#include "control_lib/tools/math.hpp"
#include "control_lib/utils/ControlState.hpp"

namespace control_lib {
    namespace controllers {
        class AbstractController {
        public:
            AbstractController(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01) : _output_dim(output_dim), _time_step(time_step)
            {
                // Init input and reference
                _input = utils::ControlState(input_dim, type);
                _reference = utils::ControlState(input_dim, type);

                // Init output
                _output = Eigen::VectorXd::Zero(output_dim);
            }

            AbstractController() {}

            virtual ~AbstractController() {}

            void init();

            Eigen::VectorXd getOutput()
            {
                return _output;
            }

            void setControlDimension(const size_t output_dim)
            {
                _output_dim = output_dim;
            }

            void setInput(const Eigen::VectorXd& input)
            {
                _input.setState(input);
            }

            void setReference(const Eigen::VectorXd& reference)
            {
                _reference.setState(reference);
            }

            void setTimeStep(const double time_step)
            {
                _time_step = time_step;
            }

            virtual Eigen::VectorXd update(const Eigen::VectorXd& state) = 0;

        protected:
            // Controller frequency
            double _time_step;

            // Input state and reference
            utils::ControlState _input, _reference;

            // Output dimension and state
            size_t _output_dim;
            Eigen::VectorXd _output;
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_ABSTRACTCONTROLLER_HPP