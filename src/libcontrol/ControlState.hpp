#ifndef LIBCONTROL_CONTROLSTATE_HPP
#define LIBCONTROL_CONTROLSTATE_HPP

#include "libcontrol/utils/math.hpp"
#include <Corrade/Containers/EnumSet.h>
#include <Eigen/Core>
#include <iostream>
#include <memory>

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
            ControlState state;

            if (this->_pose != nullptr)
                state._pose = std::make_unique<Eigen::VectorXd>(*(this->_pose.get()) - *(obj._pose.get()));

            if (this->_orientation != nullptr) {
                if (this->_type & ControlSpace::EULERANGLE)
                    state._orientation = std::make_unique<Eigen::VectorXd>(utils::euler_error(*(this->_orientation.get()), *(obj._orientation.get())));
                else if (this->_type & ControlSpace::ANGLEAXIS)
                    state._orientation = std::make_unique<Eigen::VectorXd>(utils::rotation_error(*(this->_orientation.get()), *(obj._orientation.get())));
                else if (this->_type & ControlSpace::QUATERNION)
                    state._orientation = std::make_unique<Eigen::VectorXd>(utils::quaternion_error(*(this->_orientation.get()), *(obj._orientation.get())));
            }

            if (this->_coordinate != nullptr)
                state._coordinate = std::make_unique<Eigen::VectorXd>(*(this->_coordinate.get()) - *(obj._coordinate.get()));

            if (this->_velocity != nullptr)
                state._velocity = std::make_unique<Eigen::VectorXd>(*(this->_velocity.get()) - *(obj._velocity.get()));

            if (this->_acceleration != nullptr)
                state._acceleration = std::make_unique<Eigen::VectorXd>(*(this->_acceleration.get()) - *(obj._acceleration.get()));

            if (this->_effort != nullptr)
                state._effort = std::make_unique<Eigen::VectorXd>(*(this->_effort.get()) - *(obj._effort.get()));
        }

        ControlSpaces getType() const { return _type; }

        // Set the state (just for reference)
        void setState(const Eigen::VectorXd& state)
        {
            size_t dof = _dim;

            // Assign coordinates (assume coordinates always present.. to change)
            if (_type & ControlSpace::LINEAR) {
                std::cout << "linear" << std::endl;
                // Check the presence of orientation
                if (_type & ControlSpace::ORIENTATION) {
                    std::cout << "orientation" << std::endl;
                    // Check presence of quaternions
                    if (_type & ControlSpace::QUATERNION) {
                        std::cout << "quaternion" << std::endl;
                        // First 3 entries pose and last 4 orientation
                        _pose = std::make_unique<Eigen::VectorXd>(state.head(3));
                        _orientation = std::make_unique<Eigen::VectorXd>(state.segment(3, 4));
                        // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                        if (_dim > 7) {
                            _coordinate = std::make_unique<Eigen::VectorXd>(state.segment(7, dof - 7));
                        }
                        // Reduce the dimension of degree of freedom
                        dof -= 1;
                    }
                    else {
                        std::cout << "not quaternion" << std::endl;
                        // First 3 entries pose and last 3 orientation
                        _pose = std::make_unique<Eigen::VectorXd>(state.head(3));
                        _orientation = std::make_unique<Eigen::VectorXd>(state.segment(3, 3));
                        // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                        if (_dim > 6)
                            _coordinate = std::make_unique<Eigen::VectorXd>(state.segment(6, dof - 6));
                    }
                }
                else
                    // If no orientation fill the coordinate with dof
                    _coordinate = std::make_unique<Eigen::VectorXd>(state.head(dof));
            }
            else if (_type & ControlSpace::QUATERNION) {
                // First 3 entries pose and last 4 orientation
                _orientation = std::make_unique<Eigen::VectorXd>(state.head(4));
                // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                if (_dim > 4)
                    _coordinate = std::make_unique<Eigen::VectorXd>(state.segment(4, dof - 4));
                // Reduce the dimension of degree of freedom
                dof -= 1;
            }
            else {
                // First 3 entries pose and last 3 orientation
                _orientation = std::make_unique<Eigen::VectorXd>(state.head(3));
                // Fill the coordinate part of the state with body (deformable) motion (LINEAR space)
                if (_dim > 3)
                    _coordinate = std::make_unique<Eigen::VectorXd>(state.segment(3, dof - 3));
            }

            // Assign velocity
            if (state.size() > _dim)
                _velocity = std::make_unique<Eigen::VectorXd>(state.segment(_dim, dof));

            // Assign acceleration
            if (state.size() > (_dim + dof))
                _acceleration = std::make_unique<Eigen::VectorXd>(state.segment(_dim + dof, dof));

            // Assign effort
            if (state.size() > _dim + 2 * dof)
                _effort = std::make_unique<Eigen::VectorXd>(state.segment(_dim + 2 * dof, dof));
        }

        // Faster way of setting the input state once the reference is defined
        void setState(const Eigen::VectorXd& state, const ControlState& ref)
        {
        }

        // Find a better name for this (and pass ref)
        Eigen::VectorXd getPos()
        {
            size_t dim = 0;

            if (_pose != nullptr)
                dim += _pose->size();

            if (_orientation != nullptr)
                dim += _orientation->size();

            if (_coordinate != nullptr)
                dim += _coordinate->size();

            Eigen::VectorXd pos(dim);
            pos << *(_pose.get()), *(_orientation.get()), *(_coordinate.get());

            return pos;
        }

        const Eigen::VectorXd& getVel()
        {
            return *(_velocity.get());
        }

        const Eigen::VectorXd& getAcc()
        {
            return *(_acceleration.get());
        }

        const Eigen::VectorXd& getEff()
        {
            return *(_effort.get());
        }

        std::unique_ptr<Eigen::VectorXd> _pose, // Rigid body pose
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