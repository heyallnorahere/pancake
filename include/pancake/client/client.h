#pragma once
#include <rclcpp/rclcpp.hpp>

namespace pancake::client {
    class Client : public rclcpp::Node {
    public:
        Client();
        ~Client();

    private:
        void Update();
        
        rclcpp::TimerBase::SharedPtr m_Timer;
    };
} // namespace pancake::client