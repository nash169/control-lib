#ifndef CONTROLLIB_SPATIAL_SE3_HPP
#define CONTROLLIB_SPATIAL_SE3_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        struct SE3 {
            SE3(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) : _rot(rot), _trans(trans) {}

            const Eigen::Matrix3d& rotation() const { return _rot; }

            const Eigen::Vector3d& translation() const { return _trans; }

            SE3 action(const SE3& pose) const { return SE3(_rot * pose.rotation(), _trans + _rot * pose.translation()); }

            SE3 actionInverse(const SE3& pose) { return SE3(_rot.transpose() * pose.rotation(), _rot.transpose() * (pose.translation() - _trans)); }

            Eigen::Matrix<double, 6, 1> log(const SE3& pose)
            {
                Eigen::AngleAxisd aa(pose.rotation());

                Eigen::Vector3d omega = aa.angle() * aa.axis();

                Eigen::Matrix3d omega_skew;
                omega_skew << 0, -omega(2), omega(1),
                    omega(2), 0, -omega(0),
                    -omega(1), omega(0), 0;

                double theta = omega.norm(), A = std::sin(theta) / theta, B = (1 - std::cos(theta)) / std::pow(theta, 2);

                return (Eigen::Matrix<double, 6, 1>() << (Eigen::Matrix3d::Identity() - 0.5 * omega_skew + (1 - 0.5 * A / B) / std::pow(theta, 2) * omega_skew * omega_skew) * pose.translation(), omega).finished();
            }

        protected:
            Eigen::Vector3d _trans;
            Eigen::Matrix3d _rot;
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SE3_HPP