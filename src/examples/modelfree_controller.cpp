#include <control_lib/Control.hpp>
#include <iostream>

using namespace control_lib;

int main(int argc, char const* argv[])
{
    // Examples state
    // 3 pose + 4 quaternion + 5 generalized coordinates -> dimension = 12

    size_t dim = 12, ctr_dim = 4;
    Eigen::VectorXd q = Eigen::VectorXd::Random(2 * dim - 1), q_ref = Eigen::VectorXd::Random(2 * dim - 1);

    // Reference state creation
    q << 0.9572, 0.4854, 0.8003, 0.1419, 0.4218, 0.9157, 0.7922, 0.9595, 0.6557, 0.0357, 0.8491, 0.9340,
        0.7094, 0.7547, 0.2760, 0.6797, 0.6551, 0.1626, 0.1190, 0.4984, 0.9597, 0.3404, 0.5853;

    // Reference state createio
    q_ref << 0.6948, 0.3171, 0.9502, 0.0344, 0.4387, 0.3816, 0.7655, 0.7952, 0.1869, 0.4898, 0.4456, 0.6463,
        0.7513, 0.2551, 0.5060, 0.6991, 0.8909, 0.9593, 0.5472, 0.1386, 0.1493, 0.2575, 0.8407;

    // Controller creation
    double step = 0.001;
    Eigen::MatrixXd p_gains = Eigen::MatrixXd::Random(ctr_dim, dim),
                    d_gains = Eigen::MatrixXd::Random(ctr_dim, dim - 1),
                    i_gains = Eigen::MatrixXd::Random(ctr_dim, dim);

    controllers::Feedback ctr(ControlSpace::LINEAR | ControlSpace::QUATERNION, dim, ctr_dim, step);

    ctr.setReference(q_ref);
    ctr.setGains("p", p_gains).setGains("d", d_gains).setGains("i", i_gains);

    std::cout << "Control ouput (step 1)" << std::endl;
    std::cout << ctr.update(q).transpose() << std::endl;

    std::cout << "Control ouput (step 2)" << std::endl;
    std::cout << ctr.update(q).transpose() << std::endl;

    return 0;
}