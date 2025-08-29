#pragma once
#include <rclcpp/node.hpp>

#include "pancake/client/view.h"

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

        template <typename _Ty, typename... Args>
        void AddView(Args&&... args) {
            static_assert(std::is_base_of_v<View, _Ty>, "Passed type is not a view!");

            View* view = new _Ty(std::forward<Args>(args)...);
            std::string id = view->GetID();

            m_Views.insert(std::make_pair(id, std::shared_ptr<View>(view)));
        }

        SDL_Window* m_Window;
        SDL_Renderer* m_Renderer;
        std::unordered_map<uint32_t, SDL_Gamepad*> m_Gamepads;
        bool m_HasVideo;

        rclcpp::TimerBase::SharedPtr m_Timer;
        rclcpp::Publisher<pancake::msg::Input>::SharedPtr m_Publisher;

        std::unordered_map<std::string, std::shared_ptr<View>> m_Views;
    };
} // namespace pancake::client
