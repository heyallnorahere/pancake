#pragma once
#include <rclcpp/node.hpp>

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
        ~Robot() = default;

        Robot(const Robot&) = delete;
        Robot& operator=(const Robot&) = delete;

        void InputReceived(const pancake::msg::Input& input);

    private:
        void InputMessageReceived(std::shared_ptr<rclcpp::SerializedMessage> message);
        void ProcessInput();
        
        void Update();
        void UpdateInput();

        rclcpp::GenericPublisher::SharedPtr m_RequestPublisher;
        rclcpp::GenericSubscription::SharedPtr m_InputSubscriber;
        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
    };
} // namespace pancake::robot