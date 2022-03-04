#ifndef CONTROLLIB_SPATIAL_SE3_HPP
#define CONTROLLIB_SPATIAL_SE3_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        struct SE3 {
            /* Init via translation and orientation */
            SE3(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, const Eigen::Matrix<double, 6, 1>& = Eigen::VectorXd::Zero(6)) : _rot(rot), _trans(trans)
            {
                _vel.setZero();
            }

            /* Init via vector representation */
            SE3(const Eigen::Matrix<double, 6, 1>& x) : _trans(x.head(3)), _rot(Eigen::AngleAxisd(x.tail(3).norm(), x.tail(3).normalized())) {}

            const Eigen::Matrix3d& rotation() const { return _rot; }

            const Eigen::Vector3d& translation() const { return _trans; }

            const Eigen::Matrix<double, 6, 1> velocity() const { return _vel.size() ? _vel : Eigen::VectorXd::Zero(6); }

            SE3 action(const SE3& pose) const { return SE3(_rot * pose.rotation(), _trans + _rot * pose.translation()); }

            SE3 actionInverse(const SE3& pose) const { return SE3(_rot.transpose() * pose.rotation(), _rot.transpose() * (pose.translation() - _trans)); }

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

            Eigen::Matrix<double, 6, 1> log() { return log(*this); }

            SE3 operator-(SE3 const& obj) const { return obj.actionInverse(*this); }

            constexpr static size_t dimension() { return 6; }

        protected:
            // R3
            Eigen::Vector3d _trans;

            // SO3
            Eigen::Matrix3d _rot;

            // SE3 Lie Algebra (not really)... maybe add acceleration and force
            Eigen::Matrix<double, 6, 1> _vel;
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SE3_HPP