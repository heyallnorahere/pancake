#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/msg/input.hpp"
#include "pancake/msg/chassis_speeds.hpp"
#include "pancake/client/client.h"

namespace pancake::robot {
    struct InputState {
        bool Pressed;
        bool Up, Down;
        std::array<float, 2> Axes;
    };

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

        rclcpp::Publisher<pancake::msg::ChassisSpeeds>::SharedPtr m_SpeedPublisher;
        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_InputSubscriber;
        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
    };
} // namespace pancake::robot