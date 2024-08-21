#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/msg/input.hpp"

namespace pancake::swerve {
    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve();

    private:
        void InputReceived(const pancake::msg::Input& input);

        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_Subscriber;
    };
}; // namespace pancake::swerve