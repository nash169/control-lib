#include <iostream>
#include <libcontrol/AbstractController.hpp>

using namespace libcontrol;

int main(int argc, char const* argv[])
{
    // size_t dim = 9;
    // Eigen::VectorXd vec = Eigen::VectorXd::Random(36);

    // ControlState state(dim, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);

    // state.setState(vec);

    // std::cout << "Generalized state" << std::endl;
    // std::cout << vec.transpose() << std::endl;

    // std::cout << "Generalized pose" << std::endl;
    // std::cout << state._pose->transpose() << std::endl;

    // std::cout << "Generalized orientation" << std::endl;
    // std::cout << state._orientation->transpose() << std::endl;

    // std::cout << "Generalized coordinate" << std::endl;
    // std::cout << state._coordinate->transpose() << std::endl;

    // std::cout << "Generalized velocity" << std::endl;
    // std::cout << state._velocity->transpose() << std::endl;

    // std::cout << "Generalized acceleration" << std::endl;
    // std::cout << state._acceleration->transpose() << std::endl;

    // std::cout << "Generalized effort" << std::endl;
    // std::cout << state._effort->transpose() << std::endl;

    // auto test = state._velocity - state._acceleration;

    return 0;
}
