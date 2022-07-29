#ifndef CONTROLLIB_SPATIAL_STATE_H
#define CONTROLLIB_SPATIAL_STATE_H

#include <Eigen/Core>

namespace control_lib {
    namespace spatial {
        template <size_t N>
        struct RN;

        struct SE3;
        struct SO3;
    } // namespace spatial

    template <typename Space>
    struct State;

    template <size_t N>
    struct State<spatial::RN<N>> {
        Eigen::Matrix<double, N, 1> _pos, _vel, _acc, _eff;
    };

    template <>
    struct State<spatial::SE3> {
        /* Translation & rotation */
        Eigen::Vector3d trans;
        Eigen::Matrix3d rot;

        /* Tangent and contagent plane elements (optionals) */
        Eigen::Matrix<double, 6, 1> vel, acc, eff;
    };

    template <>
    struct State<spatial::SO3> {
        /* rotation */
        Eigen::Matrix3d rot;

        /* Tangent and contagent plane elements (optionals) */
        Eigen::Vector3d vel, acc, eff;
    };

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_STATE_H