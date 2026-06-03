#include <array>
#include <iostream>
#include <sstream>
#include <string>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

// 格式化打印向量
template<class Vector>
std::string pretty_print(const Vector& array) {
    std::ostringstream ss;
    for (size_t i = 0; i < array.size(); ++i) {
        if (i) {
            ss << ", ";
        }
        ss << array[i];
    }
    return ss.str();
}

int main() {
    InputParameter<3> input;
    input.current_position = {0.0, 0.0, 0.5};
    input.current_velocity = {0.0, -2.2, -0.5};
    input.current_acceleration = {0.0, 2.5, -0.5};

    input.target_position = {5.0, -2.0, -3.5};
    input.target_velocity = {0.0, -0.5, -2.0};
    input.target_acceleration = {0.0, 0.0, 0.5};

    input.max_velocity = {3.0, 1.0, 3.0};
    input.max_acceleration = {3.0, 2.0, 1.0};
    input.max_jerk = {4.0, 3.0, 2.0};

    // 负方向不对称约束
    input.min_velocity = {-2.0, -0.5, -3.0};
    input.min_acceleration = {-2.0, -2.0, -2.0};

    Ruckig<3> ruckig;
    Trajectory<3> trajectory;

    const Result result = ruckig.calculate(input, trajectory);
    if (result == Result::ErrorInvalidInput) {
        std::cout << "Invalid input!" << std::endl;
        return -1;
    }

    std::cout << "Trajectory duration: " << trajectory.get_duration() << " [s]." << std::endl;

    const double new_time = 1.0;
    std::array<double, 3> new_position, new_velocity, new_acceleration;
    trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);
    std::cout << "Position at time " << new_time << " [s]: " << pretty_print(new_position) << std::endl;

    std::array<Bound, 3> position_extrema;
    trajectory.get_position_extrema(position_extrema);
    std::cout << "Position extremas for DoF 3 are " << position_extrema[2].min << " (min) to "
              << position_extrema[2].max << " (max)" << std::endl;

    return 0;
}
