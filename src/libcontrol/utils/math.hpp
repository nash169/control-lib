#ifndef LIBCONTROL_UTILS_MATH_HPP
#define LIBCONTROL_UTILS_MATH_HPP

#include <Eigen/Geometry>

namespace libcontrol {
    namespace utils {
        inline Eigen::Vector3d euler_error(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
        }

        inline Eigen::Vector3d rotation_error(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
            Eigen::Matrix3d R_current = Eigen::AngleAxisd(curr.norm(), curr.normalized()).toRotationMatrix(),
                            R_desired = Eigen::AngleAxisd(ref.norm(), ref.normalized()).toRotationMatrix();

            Eigen::AngleAxisd aa = Eigen::AngleAxisd(R_current * R_desired.transpose());

            return aa.axis() * aa.angle();
        }

        inline Eigen::Vector4d quaternion_error(const Eigen::Vector4d& curr, const Eigen::Vector4d& ref)
        {
            Eigen::Quaterniond q_current = Eigen::Quaterniond(curr), q_desired = Eigen::Quaterniond(ref);
            return (q_current.inverse() * q_desired).coeffs();
        }
    } // namespace utils
} // namespace libcontrol

#endif // LIBCONTROL_UTILS_MATH_HPP