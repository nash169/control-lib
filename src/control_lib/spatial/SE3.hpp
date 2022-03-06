#ifndef CONTROLLIB_SPATIAL_SE3_HPP
#define CONTROLLIB_SPATIAL_SE3_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        struct SE3 {
            /* Init via translation and orientation */
            SE3(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) : _rot(rot), _trans(trans) {}

            /* Init via vector representation */
            SE3(const Eigen::Matrix<double, 6, 1>& x) : _trans(x.head(3)), _rot(Eigen::AngleAxisd(x.tail(3).norm(), x.tail(3).normalized())) {}

            /* Default constructor */
            SE3() = default;

            /* Space elements difference */
            Eigen::Matrix<double, 6, 1> operator-(SE3 const& obj) const { return obj.actionInverse(*this).log(); }

            /* Space dimension */
            constexpr static size_t dimension() { return 6; }

            /* Translation & rotation */
            Eigen::Vector3d _trans;
            Eigen::Matrix3d _rot;

            /* Tangent and contagent plane elements (optionals) */
            Eigen::Matrix<double, 6, 1> _vel, _acc, _eff;

        protected:
            SE3 action(const SE3& pose) const { return SE3(_rot * pose._rot, _trans + _rot * pose._trans); }

            SE3 actionInverse(const SE3& pose) const { return SE3(_rot.transpose() * pose._rot, _rot.transpose() * (pose._trans - _trans)); }

            Eigen::Matrix<double, 6, 1> log(const SE3& pose) const
            {
                Eigen::AngleAxisd aa(pose._rot);

                Eigen::Vector3d omega = aa.angle() * aa.axis();

                Eigen::Matrix3d omega_skew;
                omega_skew << 0, -omega(2), omega(1),
                    omega(2), 0, -omega(0),
                    -omega(1), omega(0), 0;

                double theta = omega.norm(), A = std::sin(theta) / theta, B = (1 - std::cos(theta)) / std::pow(theta, 2);

                return (Eigen::Matrix<double, 6, 1>() << (Eigen::Matrix3d::Identity() - 0.5 * omega_skew + (1 - 0.5 * A / B) / std::pow(theta, 2) * omega_skew * omega_skew) * pose._trans, omega).finished();
            }

            Eigen::Matrix<double, 6, 1> log() { return log(*this); }
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SE3_HPP