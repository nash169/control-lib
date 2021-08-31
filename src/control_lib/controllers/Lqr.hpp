#ifndef CONTROLLIB_CONTROLLERS_LQR_HPP
#define CONTROLLIB_CONTROLLERS_LQR_HPP

#include "control_lib/controllers/Feedback.hpp"

namespace control_lib {
    struct LinearModel {
        Eigen::MatrixXd A, B, C, D;
    };

    namespace controllers {
        class Lqr : public Feedback {
        public:
            Lqr(ControlSpaces type, const size_t input_dim, const size_t output_dim, const double time_step = 0.01)
                : Feedback(type, input_dim, output_dim, time_step) {}

            Lqr() = default;

            void setModel(const LinearModel& model)
            {
                _linear_model = model;
            }

            void optimalGains(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, size_t max_iter = 100, double tol = 1e-3)
            {
                // check if linear model is available
                bool stable;
                size_t state_dim = _linear_model.B.rows(), input_dim = _linear_model.B.cols(), iter = 0;
                double alpha = 1.1, beta = 0.9, max_eig = _linear_model.A.eigenvalues().real().maxCoeff(), shift, shiftrcv;

                Eigen::MatrixXd G = Eigen::MatrixXd::Zero(input_dim, state_dim), Gp, As;

                if (max_eig >= 0) {
                    stable = false;
                    shift = alpha * max_eig;
                    As = _linear_model.A - shift * Eigen::MatrixXd::Identity(state_dim, state_dim);
                    shiftrcv = 0;
                }
                else {
                    stable = true;
                    As = _linear_model.A;
                }

                while (iter <= max_iter) {
                    Gp = G;
                    G = R.colPivHouseholderQr() //selfadjointView<Eigen::Upper>().llt()
                            .solve(tools::bartelsStewart(As - _linear_model.B * G, -Q - G.transpose() * R * G));

                    if ((G - Gp).lpNorm<Eigen::Infinity>() / G.lpNorm<Eigen::Infinity>() < tol)
                        break;

                    if (!stable) {
                        max_eig = beta * (As - _linear_model.B * G).eigenvalues().real().array().abs().minCoeff();
                        shiftrcv = shiftrcv + max_eig;

                        if (shiftrcv > shift) {
                            stable = true;
                            As = _linear_model.A;
                        }
                        else
                            As = As + max_eig * Eigen::MatrixXd::Identity(state_dim, state_dim);
                    }
                }
            }

        protected:
            LinearModel _linear_model;
            Eigen::MatrixXd _gain_matrix;
        };
    } // namespace controllers
} // namespace control_lib

#endif // CONTROLLIB_CONTROLLERS_LQR_HPP