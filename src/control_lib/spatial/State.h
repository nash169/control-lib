#ifndef CONTROLLIB_SPATIAL_STATE_H
#define CONTROLLIB_SPATIAL_STATE_H

#include <Eigen/Core>

namespace control_lib {
    // Forward declaration of available manifolds
    namespace spatial {
        template <size_t N>
        struct RN;

        struct SE3;

        struct SO3;
    } // namespace spatial

    // Generic State
    template <typename Space>
    struct State;

    // Euclidean space specialization
    template <size_t N>
    struct State<spatial::RN<N>> {
        Eigen::Matrix<double, N, 1> _pos, _vel, _acc, _eff;
    };

    // Special Euclidean group specialization
    template <>
    struct State<spatial::SE3> {
        State() = default;

        State(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
            : trans(translation), rot(rotation) {}

        /* Translation & rotation */
        Eigen::Vector3d trans;
        Eigen::Matrix3d rot;

        /* Tangent and cotangent plane elements (optionals) */
        Eigen::Matrix<double, 6, 1> vel, acc, eff;
    };

    // Special Orthogonal group specialization
    template <>
    struct State<spatial::SO3> {
        /* rotation */
        Eigen::Matrix3d rot;

        /* Tangent and cotangent plane elements (optionals) */
        Eigen::Vector3d vel, acc, eff;
    };

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_STATE_H