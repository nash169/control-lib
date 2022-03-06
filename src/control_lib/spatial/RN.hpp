#ifndef CONTROLLIB_SPATIAL_RN_HPP
#define CONTROLLIB_SPATIAL_RN_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <size_t N>
        struct RN {
            /* Init via translation and orientation */
            RN(const Eigen::Matrix<double, N, 1>& pos) : _pos(pos) {}

            RN() = default;

            /* Space elements difference */
            Eigen::Matrix<double, N, 1> operator-(RN const& obj) const { return _pos - obj._pos; }

            /* Space dimension */
            constexpr static size_t dimension() { return N; }

            /* Tangent and contagent plane elements (optionals) */
            Eigen::Matrix<double, N, 1> _pos, _vel, _acc, _eff;
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_RN_HPP