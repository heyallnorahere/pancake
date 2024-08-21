#include <rclcpp/rclcpp.hpp>

#include "pancake/swerve/swerve.h"

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pancake::swerve::Swerve>());
    rclcpp::shutdown();
    return 0;
}