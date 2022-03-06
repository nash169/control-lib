#ifndef CONTROLLIB_SPATIAL_SO3_HPP
#define CONTROLLIB_SPATIAL_SO3_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        struct SO3 {
            /* Init via translation and orientation */
            SO3(const Eigen::Matrix3d& rot) : _rot(rot) {}

            /* Init via vector representation */
            SO3(const Eigen::Vector3d& omega) : _rot(Eigen::AngleAxisd(omega.tail(3).norm(), omega.tail(3).normalized())) {}

            /* Default constructor */
            SO3() = default;

            /* Space elements difference */
            Eigen::Vector3d operator-(SO3 const& obj) const { return obj.actionInverse(*this).log(); }

            /* Space dimension */
            constexpr static size_t dimension() { return 3; }

            /* rotation */
            Eigen::Matrix3d _rot;

            /* Tangent and contagent plane elements (optionals) */
            Eigen::Vector3d _vel, _acc, _eff;

        protected:
            SO3 action(const SO3& rot) const { return SO3(Eigen::Matrix3d(_rot * rot._rot)); }

            SO3 actionInverse(const SO3& rot) const { return SO3(Eigen::Matrix3d(_rot.transpose() * rot._rot)); }

            Eigen::Vector3d log(const SO3& rot) const
            {
                Eigen::AngleAxisd aa(rot._rot);

                return aa.angle() * aa.axis();
            }

            Eigen::Vector3d log() { return log(*this); }
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SO3_HPP