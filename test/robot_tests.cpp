#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <pancake/client/client.h>
#include <pancake/robot/robot.h>
#include <pancake/msg/input.hpp>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

TEST(pancake, input_test) {
    pancake::robot::Robot node;

    pancake::msg::Input input;
    input.id = (uint8_t)pancake::client::GamepadInput::A;
    input.down = true;
    input.pressed = true;
    input.up = false;
    node.InputReceived(input);

    auto id = (pancake::client::GamepadInput)input.id;
    ASSERT_TRUE(pancake::robot::Robot::IsInputDown(id));
    ASSERT_TRUE(pancake::robot::Robot::IsInputPressed(id));
    ASSERT_FALSE(pancake::robot::Robot::IsInputUp(id));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int retval = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return retval;
}