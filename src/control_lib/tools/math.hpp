#ifndef CONTROLLIB_TOOLS_MATH_HPP
#define CONTROLLIB_TOOLS_MATH_HPP

#include <Eigen/Geometry>

namespace control_lib {
    namespace tools {
        inline Eigen::Vector3d eulerError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
        }

        inline Eigen::Vector3d rotationError(const Eigen::Vector3d& curr, const Eigen::Vector3d& ref)
        {
            Eigen::Matrix3d R_current = Eigen::AngleAxisd(curr.norm(), curr.normalized()).toRotationMatrix(),
                            R_desired = Eigen::AngleAxisd(ref.norm(), ref.normalized()).toRotationMatrix();

            Eigen::AngleAxisd aa = Eigen::AngleAxisd(R_current * R_desired.transpose());

            return aa.axis() * aa.angle();
        }

        inline Eigen::Vector4d quaternionError(const Eigen::Vector4d& curr, const Eigen::Vector4d& ref)
        {
            Eigen::Quaterniond q_current = Eigen::Quaterniond(curr), q_desired = Eigen::Quaterniond(ref);
            return (q_current.inverse() * q_desired).coeffs();
        }

        inline Eigen::MatrixXd kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
        {
            Eigen::MatrixXd C(A.rows() * B.rows(), A.cols() * B.cols());

            for (size_t i = 0; i < A.rows(); i++) {
                for (size_t j = 0; j < A.cols(); j++)
                    C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
            }
        }

        Eigen::MatrixXd solveVectorized(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);

        Eigen::MatrixXd bartelsStewart(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W);
    } // namespace tools
} // namespace control_lib

#endif // CONTROLLIB_TOOLS_MATH_HPP