#ifndef CONTROLLIB_SPATIAL_X_HPP
#define CONTROLLIB_SPATIAL_X_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace spatial {
        template <typename... Manifolds>
        struct R {
            /* Init via translation and orientation */
            X() : {}

            X() = default;

            // /* Space dimension */
            // constexpr static size_t dimension() { return N; }
        };
    } // namespace spatial

} // namespace control_lib

#endif // CONTROLLIB_SPATIAL_X_HPP