#ifndef CONTROLLIB_SPATIAL_SpecialOrthogonal_HPP
#define CONTROLLIB_SPATIAL_SpecialOrthogonal_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <int N = Eigen::Dynamic, bool Left = false>
        struct SpecialOrthogonal {
            // /* Init via translation and orientation */
            // SpecialOrthogonal(const Eigen::Matrix<double, N, N>& rot) : _rot(rot) {}

            /* Default constructor */
            SpecialOrthogonal() = default;

            /* Space dimension */
            constexpr static size_t dimension() { return (N - 1) / 2; }

            /* Action */
            static Eigen::Matrix<double, N, N> action(const Eigen::Matrix<double, N, N>& x, const Eigen::Matrix<double, N, N>& y)
            {
                return Left ? y * x : x * y;
            }

            /* Action Inverse */
            static Eigen::Matrix<double, N, N> actionInverse(const Eigen::Matrix<double, N, N>& x, const Eigen::Matrix<double, N, N>& y)
            {
                return Left ? y * x.transpose() : x.transpose() * y;
            }

            Eigen::Matrix<double, N, 1> logarithm(const SpecialOrthogonal& rot) const
            {
                return Eigen::VectorXd::Zero(N);
            }

            Eigen::Matrix<double, N, 1> logarithm() { return logarithm(*this); }
        };

        // template <>
        // Eigen::Vector3d SpecialOrthogonal<3>::logarithm(const SpecialOrthogonal& rot) const
        // {
        //     Eigen::AngleAxisd aa(rot._rot);

        //     return aa.angle() * aa.axis();
        // }

        // template <>
        // Eigen::Vector3d SpecialOrthogonal<3, true>::logarithm(const SpecialOrthogonal& rot) const
        // {
        //     Eigen::AngleAxisd aa(rot._rot);

        //     return aa.angle() * aa.axis();
        // }

    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_SpecialOrthogonal_HPP