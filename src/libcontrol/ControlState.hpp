#ifndef LIBCONTROL_CONTROLSTATE_HPP
#define LIBCONTROL_CONTROLSTATE_HPP

#include <Corrade/Containers/EnumSet.h>

#include "libcontrol/utils/math.hpp"

namespace libcontrol {

    // The control space defines how to measure distance between points
    enum class ControlSpace : unsigned int {
        LINEAR = 1 << 0, // Flat Euclidean space
        EULERANGLE = 1 << 1, // Euler angles space
        ANGLEAXIS = 1 << 2, // Rotation matrix (SO3) space
        QUATERNION = 1 << 3, // Quaternion space
        ORIENTATION = EULERANGLE | ANGLEAXIS | QUATERNION
    };

    using ControlSpaces = Corrade::Containers::EnumSet<ControlSpace>;
    CORRADE_ENUMSET_OPERATORS(ControlSpaces)

    struct ControlState {
    public:
        ControlState(size_t dim, ControlSpaces type) : _dim(dim), _type(type) {}
        ControlState() {}

        ~ControlState() {}

        ControlState operator-(ControlState const& obj)
        {
            // Here goes some check of dimension and type
            ControlState state(this->getDim(), this->getType());

            if (this->_pose.has_value())
                state._pose = this->_pose.value() - obj._pose.value();

            if (this->_orientation.has_value()) {
                if (this->_type & ControlSpace::EULERANGLE)
                    state._orientation = utils::euler_error(this->_orientation.value(), obj._orientation.value());
                else if (this->_type & ControlSpace::ANGLEAXIS)
                    state._orientation = utils::rotation_error(this->_orientation.value(), obj._orientation.value());
                else if (this->_type & ControlSpace::QUATERNION)
                    state._orientation.value() = utils::quaternion_error(this->_orientation.value(), obj._orientation.value());
            }

            if (this->_coordinate.has_value())
                state._coordinate = this->_coordinate.value() - obj._coordinate.value();

            if (this->_velocity.has_value())
                state._velocity = this->_velocity.value() - obj._velocity.value();

            if (this->_acceleration.has_value())
                state._acceleration = this->_acceleration.value() - obj._acceleration.value();

            if (this->_effort.has_value())
                state._effort = this->_effort.value() - obj._effort.value();

            return state;
        }

        size_t getDim() const { return _dim; }

        ControlSpaces getType() const { return _type; }

        // Find a better name for this (and pass ref)
        Eigen::VectorXd getPos()
        {
            size_t dim = 0;

            if (_pose.has_value())
                dim += _pose->size();

            if (_orientation.has_value())
                dim += _orientation->size();

            if (_coordinate.has_value())
                dim += _coordinate->size();

            Eigen::VectorXd pos(dim);

            int curr_index = 0;
            if (_pose.has_value()) {
                pos.head(_pose->size()) = _pose.value();
                curr_index += _pose->size();
            }
            if (_orientation.has_value())
                pos.segment(curr_index, _orientation->size()) = _orientation.value();
            if (_coordinate.has_value())
                pos.tail(_coordinate->size()) = _coordinate.value();

            return pos;
        }

        // Set the state (just for reference)
        void setState(const Eigen::VectorXd& state)
        {
            size_t dof = _dim;

            // Assign coordinates (assume coordinates always present.. to change)
            if (_type & ControlSpace::LINEAR) {
                // Check the presence of orientation
                if (_type & ControlSpace::ORIENTATION) {
                    // Check presence of quaternions
                    if (_type & ControlSpace::QUATERNION) {
                        // First 3 entries pose and last 4 orientation
                        _pose = state.head(3);
                        _orientation = state.segment(3, 4);
                        // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                        if (_dim > 7) {
                            _coordinate = state.segment(7, dof - 7);
                        }
                        // Reduce the dimension of degree of freedom
                        dof -= 1;
                    }
                    else {
                        // First 3 entries pose and last 3 orientation
                        _pose = state.head(3);
                        _orientation = state.segment(3, 3);
                        // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                        if (_dim > 6)
                            _coordinate = state.segment(6, dof - 6);
                    }
                }
                else
                    // If no orientation fill the coordinate with dof
                    _coordinate = state.head(dof);
            }
            else if (_type & ControlSpace::QUATERNION) {
                // First 3 entries pose and last 4 orientation
                _orientation = state.head(4);
                // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                if (_dim > 4)
                    _coordinate = state.segment(4, dof - 4);
                // Reduce the dimension of degree of freedom
                dof -= 1;
            }
            else {
                // First 3 entries pose and last 3 orientation
                _orientation = state.head(3);
                // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                if (_dim > 3)
                    _coordinate = state.segment(3, dof - 3);
            }

            // Assign velocity
            if (state.size() > _dim)
                _velocity = state.segment(_dim, dof);

            // Assign acceleration
            if (state.size() > (_dim + dof))
                _acceleration = state.segment(_dim + dof, dof);

            // Assign effort
            if (state.size() > _dim + 2 * dof)
                _effort = state.segment(_dim + 2 * dof, dof);
        }

        // Faster way of setting the input state once the reference is defined
        void setState(const Eigen::VectorXd& state, const ControlState& ref)
        {
        }

        std::optional<Eigen::VectorXd> _pose, // Rigid body pose
            _orientation, // Rigid body orientation
            _coordinate, // Coordinates
            _velocity, // Linear & Angular velocity
            _acceleration, // Linear & Angular acceleration
            _effort; // force & torque

    private:
        // Space type
        ControlSpaces _type;
        // Coordinate space dimension
        size_t _dim;
    };

} // namespace libcontrol

#endif // LIBCONTROL_CONTROLSTATE_HPP