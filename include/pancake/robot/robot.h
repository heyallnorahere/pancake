#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/msg/input.hpp"
#include "pancake/msg/swerve_request.hpp"
#include "pancake/client/client.h"

namespace pancake::robot {
    class Robot : public rclcpp::Node {
    public:
        static bool IsInputDown(pancake::client::GamepadInput input);
        static bool IsInputUp(pancake::client::GamepadInput input);
        static bool IsInputPressed(pancake::client::GamepadInput input);
        static float GetInputAxis(pancake::client::GamepadInput input, size_t axis);

        Robot();

    private:
        void InputReceived(const pancake::msg::Input& input);

        void Update();
        void UpdateInput();

        rclcpp::Publisher<pancake::msg::SwerveRequest>::SharedPtr m_RequestPublisher;
        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_InputSubscriber;
        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
    };
} // namespace pancake::robot