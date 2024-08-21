#include "pancake/swerve/swerve.h"

namespace pancake::swerve {
    Swerve::Swerve() : Node("swerve") {
        m_Subscriber = create_subscription<pancake::msg::Input>(
            "/pancake/swerve/control", 10,
            std::bind(&Swerve::InputReceived, this, std::placeholders::_1));
    }

    Swerve::~Swerve() {
        // todo: shut down swerve stuff
    }

    void Swerve::InputReceived(const pancake::msg::Input& input) {
        RCLCPP_INFO(get_logger(), "Input received: %u", (uint32_t)input.id);
        RCLCPP_INFO(get_logger(), "\tDown? %s", input.down ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tUp? %s", input.up ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tPressed? %s", input.pressed ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tAxis 0: %f", input.axes[0]);
        RCLCPP_INFO(get_logger(), "\tAxis 1: %f", input.axes[1]);
    }
}; // namespace pancake::swerve