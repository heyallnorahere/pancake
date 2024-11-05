#include "pancakepch.h"
#include "pancake/robot/robot.h"

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pancake::robot::Robot>());
    rclcpp::shutdown();

    return 0;
}