#ifndef LIBCONTROL_ABSTRACTCONTROLLER_HPP
#define LIBCONTROL_ABSTRACTCONTROLLER_HPP

#include <libcontrol/ControlState.hpp>
#include <libcontrol/utils/math.hpp>

namespace libcontrol {
    class AbstractController {
    public:
        AbstractController(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01) : _output_dim(output_dim), _time_step(time_step)
        {
            // Init input and reference
            _input = ControlState(input_dim, type);
            _reference = ControlState(input_dim, type);

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
        ControlState _input, _reference;

        // Output dimension and state
        size_t _output_dim;
        Eigen::VectorXd _output;
    };
} // namespace libcontrol

#endif // LIBCONTROL_ABSTRACTCONTROLLER_HPP