#pragma once
#include <rclcpp/node.hpp>

#include "pancake/msg/input.hpp"

#include <unordered_map>

// no SDL header in client.h
struct SDL_Window;
struct SDL_Renderer;
struct SDL_GamepadButtonEvent;
struct SDL_GamepadAxisEvent;
struct SDL_Gamepad;

namespace pancake::client {
    enum class GamepadInput : uint8_t {
        A,
        B,
        X,
        Y,
        DPadUp,
        DPadDown,
        DPadRight,
        DPadLeft,
        LeftStick,
        RightStick,
        LeftBumper,
        RightBumper,
        LeftTrigger,
        RightTrigger,
        Start,
        Select,
        Touchpad
    };

    class Client : public rclcpp::Node {
    public:
        Client();
        ~Client();

    private:
        void Update();

        void SendButton(const SDL_GamepadButtonEvent& event);
        void SendAxis(const SDL_GamepadAxisEvent& event);

        SDL_Window* m_Window;
        SDL_Renderer* m_Renderer;
        std::unordered_map<uint32_t, SDL_Gamepad*> m_Gamepads;
        bool m_SDLInitialized;

        rclcpp::TimerBase::SharedPtr m_Timer;
        rclcpp::Publisher<pancake::msg::Input>::SharedPtr m_Publisher;
    };
} // namespace pancake::client