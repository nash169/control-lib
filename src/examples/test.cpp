#include <iostream>
#include <libcontrol/AbstractController.hpp>

using namespace libcontrol;

int main(int argc, char const* argv[])
{
    size_t dim = 9;
    Eigen::VectorXd vec1 = Eigen::VectorXd::Random(36), vec2 = Eigen::VectorXd::Random(36);

    ControlState state1(dim, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);
    ControlState state2(dim, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);

    state1.setState(vec1);
    state2.setState(vec2);

    auto test = state1 - state2;

    // std::cout << "Generalized state1" << std::endl;
    // std::cout << vec1.transpose() << std::endl;

    std::cout << "Generalized pose" << std::endl;
    std::cout << test._pose.transpose() << std::endl;

    std::cout << "Generalized orientation" << std::endl;
    std::cout << test._orientation.transpose() << std::endl;

    std::cout << "Generalized coordinate" << std::endl;
    std::cout << test._coordinate.transpose() << std::endl;

    std::cout << "Generalized velocity" << std::endl;
    std::cout << test._velocity.transpose() << std::endl;

    std::cout << "Generalized acceleration" << std::endl;
    std::cout << test._acceleration.transpose() << std::endl;

    std::cout << "Generalized effort" << std::endl;
    std::cout << test._effort.transpose() << std::endl;

    return 0;
}
