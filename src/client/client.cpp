#include <rclcpp/rclcpp.hpp>

#include "pancake/client/client.h"

#include <chrono>

using namespace std::chrono_literals;

namespace pancake::client {
    Client::Client() : Node("client") {
        static constexpr uint32_t desiredFPS = 60;
        static constexpr std::chrono::duration<double> interval = 1s / desiredFPS;

        // todo: create client window

        m_Timer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(interval),
                                    std::bind(&Client::Update, this));
    }

    Client::~Client() {
        // todo: destroy client window
    }

    void Client::Update() {
        RCLCPP_INFO(get_logger(), "Update");

        // todo: update
    }
} // namespace pancake::client