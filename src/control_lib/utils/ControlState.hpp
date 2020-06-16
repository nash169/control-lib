#ifndef CONTROLLIB_UTILS_CONTROLSTATE_HPP
#define CONTROLLIB_UTILS_CONTROLSTATE_HPP

#include <Corrade/Containers/EnumSet.h>

#include "control_lib/tools/math.hpp"

namespace control_lib {

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

    namespace utils {
        struct ControlState {
        public:
            ControlState(size_t dim, ControlSpaces type) : _dim(dim), _type(type) {}
            ControlState() {}

            ~ControlState() {}

            // Find a better name for this (and pass ref)
            Eigen::VectorXd getPos()
            {
                size_t dim = 0;

                if (_pose.size())
                    dim += _pose.size();

                if (_orientation.size())
                    dim += _orientation.size();

                if (_coordinate.size())
                    dim += _coordinate.size();

                Eigen::VectorXd pos(dim);

                int curr_index = 0;
                if (_pose.size()) {
                    pos.head(_pose.size()) = _pose;
                    curr_index += _pose.size();
                }
                if (_orientation.size())
                    pos.segment(curr_index, _orientation.size()) = _orientation;
                if (_coordinate.size())
                    pos.tail(_coordinate.size()) = _coordinate;

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

            ControlState operator-(ControlState const& obj)
            {
                // Here goes some check of dimension and type
                ControlState state(this->_dim, this->_type);

                if (this->_pose.size())
                    state._pose = this->_pose - obj._pose;

                if (this->_orientation.size()) {
                    if (this->_type & ControlSpace::EULERANGLE)
                        state._orientation = tools::eulerError(this->_orientation, obj._orientation);
                    else if (this->_type & ControlSpace::ANGLEAXIS)
                        state._orientation = tools::rotationError(this->_orientation, obj._orientation);
                    else if (this->_type & ControlSpace::QUATERNION)
                        state._orientation = tools::quaternionError(this->_orientation, obj._orientation);
                }

                if (this->_coordinate.size())
                    state._coordinate = this->_coordinate - obj._coordinate;

                if (this->_velocity.size())
                    state._velocity = this->_velocity - obj._velocity;

                if (this->_acceleration.size())
                    state._acceleration = this->_acceleration - obj._acceleration;

                if (this->_effort.size())
                    state._effort = this->_effort - obj._effort;

                return state;
            }

            // Variables
            Eigen::VectorXd _pose, // Cartesian space position
                _orientation, // Cartesian space orientation
                _coordinate, // Generalized coordinates
                _velocity, // Velocity
                _acceleration, // Acceleration
                _effort; // Forces

            // Space type
            ControlSpaces _type;

            // Coordinate space dimension
            size_t _dim;
        };
    } // namespace utils
} // namespace control_lib

#endif // CONTROLLIB_UTILS_CONTROLSTATE_HPP