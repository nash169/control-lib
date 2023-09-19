# Control Library
Geometry-based controller suite. 
The geometry used for each controller adapts automatically to the underlying space on which it operates.
The library is fully templated for blazing fast performance and great versatility.

## Authors/Maintainers
- Bernardo Fichera (bernardo.fichera@gmail.com)

## Available Controller
- Feedback
- Quadratic Programming Controller
- Linear Quadratic Regulator (testing)

### Feedback
Classical feedback control. The proportional, derivative or integral terms can be selectively activated by setting the respective gains matrices. Support generic manifolds.

### Quadratic Programming Controller
Quadratic Programming control. Support only Euclidean space. This controller requires [optmization-lib](https://github.com/nash169/optimization-lib).

$$\min_{\mathbf{z}} \quad \frac{1}{2} \mathbf{z}^T \mathbf{W} \mathbf{z} + \mathbf{w}^T \mathbf{z}$$

$$\text{s.t.} \quad \mathbf{C}_E \mathbf{z} + \mathbf{c}_E = \mathbf{0}, \quad \mathbf{C}_I \mathbf{z} + \mathbf{c}_I \ge \mathbf{0}.$$
$$\mathbf{W} = \begin{bmatrix}
        \mathbf{Q} & 0     & 0     \\
        0     & \mathbf{R} & 0     \\
        0     & 0     & \mathbf{I} \\
    \end{bmatrix},
    \qquad
    \v{w}^T = \begin{bmatrix}
        -\ddot{\mathbf{q}}_d^T \mathbf{Q} & 0 & 0
    \end{bmatrix},$$

**Model Based**

**Model Free**

### Linear Quadratic Regulator (testing)
Linear Quadratic Regulator. Support only Euclidean space.

## Available Spaces
- Euclidean
- Special Orthogonal Group
- Special Euclidean Group