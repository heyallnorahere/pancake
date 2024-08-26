#include "pancake/robot/robot.h"

#include "pancake/vector2.h"

#include <chrono>
#include <numbers>

using namespace std::chrono_literals;

namespace pancake::robot {
    struct InputState {
        bool Pressed;
        bool Up, Down;
        std::array<float, 2> Axes;
    };

    static std::unordered_map<pancake::client::GamepadInput, InputState> s_Gamepad;

    static const float s_LinearControlVelocity = 10.f;                             // m/s
    static const float s_AngularControlVelocity = std::numbers::pi_v<float> * 2.f; // rad/s

    bool Robot::IsInputDown(pancake::client::GamepadInput input) {
        if (!s_Gamepad.contains(input)) {
            return false;
        }

        return s_Gamepad.at(input).Down;
    }

    bool Robot::IsInputUp(pancake::client::GamepadInput input) {
        if (!s_Gamepad.contains(input)) {
            return false;
        }

        return s_Gamepad.at(input).Up;
    }

    bool Robot::IsInputPressed(pancake::client::GamepadInput input) {
        if (!s_Gamepad.contains(input)) {
            return false;
        }

        return s_Gamepad.at(input).Pressed;
    }

    float Robot::GetInputAxis(pancake::client::GamepadInput input, size_t axis) {
        if (!s_Gamepad.contains(input)) {
            return false;
        }

        return s_Gamepad.at(input).Axes[axis];
    }

    Robot::Robot() : Node("robot") {
        m_RequestPublisher =
            create_publisher<pancake::msg::SwerveRequest>("/pancake/swerve/request", 10);

        m_InputSubscriber = create_subscription<pancake::msg::Input>(
            "/pancake/client/control", 10,
            std::bind(&Robot::InputReceived, this, std::placeholders::_1));

        m_UpdateTimer = create_wall_timer(30ms, std::bind(&Robot::Update, this));
    }

    void Robot::InputReceived(const pancake::msg::Input& input) {
        RCLCPP_INFO(get_logger(), "Input received: %u", (uint32_t)input.id);

        auto& state = s_Gamepad[(pancake::client::GamepadInput)input.id];
        std::memcpy(state.Axes.data(), input.axes.data(), state.Axes.size() * sizeof(float));

        state.Down = input.down;
        state.Up = input.up;
        state.Pressed = input.pressed;

        Update();
    }

    void Robot::Update() {
        Vector2 direction;
        direction.X = GetInputAxis(pancake::client::GamepadInput::LeftStick, 1);
        direction.Y = -GetInputAxis(pancake::client::GamepadInput::LeftStick, 0);
        float angularDirection = -GetInputAxis(pancake::client::GamepadInput::RightStick, 0);

        static constexpr float deadzone = 0.01f;
        if (direction.Length() < deadzone) {
            direction.X = direction.Y = 0.f;
        }

        if (std::abs(angularDirection) < deadzone) {
            angularDirection = 0.f;
        }

        Vector2 linear = direction.Normalize() * s_LinearControlVelocity;
        float angular = angularDirection * s_AngularControlVelocity;

        pancake::msg::SwerveRequest request;
        request.velocity.x = linear.X;
        request.velocity.y = linear.Y;
        request.velocity.angular_velocity = angular;
        request.absolute = true;
        m_RequestPublisher->publish(request);

        UpdateInput();
    }

    void Robot::UpdateInput() {
        for (auto& [id, state] : s_Gamepad) {
            state.Down = state.Up = false;
        }
    }
}; // namespace pancake::robot