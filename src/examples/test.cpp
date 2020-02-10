#include <Corrade/Containers/Optional.h>
#include <iostream>
#include <libcontrol/AbstractController.hpp>

using namespace libcontrol;
using namespace Corrade;

int main(int argc, char const* argv[])
{
    Eigen::VectorXd x(3);
    x << 1, 2, 3;

    std::optional<Eigen::VectorXd> b;
    b = x;
    Corrade::Containers::Optional<Eigen::VectorXd> a;
    a = x;

    std::cout << b->size() << std::endl;

    size_t dim = 9;
    Eigen::VectorXd vec1 = Eigen::VectorXd::Random(36), vec2 = Eigen::VectorXd::Random(36);

    ControlState state1(dim, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);
    ControlState state2(dim, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);

    state1.setState(vec1);
    state2.setState(vec2);

    auto test = state1 - state2;

    std::cout << "Generalized state1" << std::endl;
    std::cout << vec1.transpose() << std::endl;

    std::cout << "Generalized pose" << std::endl;
    std::cout << test._pose.value().transpose() << std::endl;

    std::cout << "Generalized orientation" << std::endl;
    std::cout << test._orientation.value().transpose() << std::endl;

    std::cout << "Generalized coordinate" << std::endl;
    std::cout << test._coordinate.value().transpose() << std::endl;

    std::cout << "Generalized velocity" << std::endl;
    std::cout << test._velocity.value().transpose() << std::endl;

    std::cout << "Generalized acceleration" << std::endl;
    std::cout << test._acceleration.value().transpose() << std::endl;

    std::cout << "Generalized effort" << std::endl;
    std::cout << test._effort.value().transpose() << std::endl;

    return 0;
}
