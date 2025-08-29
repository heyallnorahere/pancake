#pragma once
#include "pancake/client/client.h"

#include <std_msgs/msg/bool.hpp>

namespace pancake::robot {
    class Robot : public rclcpp::Node {
    public:
        static bool IsInputDown(pancake::client::GamepadInput input);
        static bool IsInputUp(pancake::client::GamepadInput input);
        static bool IsInputPressed(pancake::client::GamepadInput input);
        static float GetInputAxis(pancake::client::GamepadInput input, size_t axis);

        Robot();
        ~Robot() = default;

        Robot(const Robot&) = delete;
        Robot& operator=(const Robot&) = delete;

        void InputReceived(const pancake::msg::Input& input);

    private:
        void ProcessInput();

        void Update();
        void UpdateInput();

        rclcpp::Publisher<pancake::msg::SwerveRequest>::SharedPtr m_RequestPublisher;
        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_InputSubscriber;
        rclcpp::TimerBase::SharedPtr m_UpdateTimer;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_KillPublisher, m_ReloadPublisher;
    };
} // namespace pancake::robot
