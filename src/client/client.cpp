#include "pancake/client/client.h"

#include "pancake/client/tuner.h"

#include <chrono>
#include <unordered_map>
#include <optional>
#include <limits>

#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_sdlrenderer3.h>

using namespace std::chrono_literals;

namespace pancake::client {
    struct GamepadInputInfo {
        std::optional<SDL_GamepadButton> AssociatedButton;
        std::vector<SDL_GamepadAxis> AssociatedAxes;
    };

    static const std::unordered_map<GamepadInput, GamepadInputInfo> s_InputMapping = {
        { GamepadInput::A, { SDL_GAMEPAD_BUTTON_SOUTH, {} } },
        { GamepadInput::B, { SDL_GAMEPAD_BUTTON_EAST, {} } },
        { GamepadInput::X, { SDL_GAMEPAD_BUTTON_WEST, {} } },
        { GamepadInput::Y, { SDL_GAMEPAD_BUTTON_NORTH, {} } },
        { GamepadInput::DPadUp, { SDL_GAMEPAD_BUTTON_DPAD_UP, {} } },
        { GamepadInput::DPadDown, { SDL_GAMEPAD_BUTTON_DPAD_DOWN, {} } },
        { GamepadInput::DPadRight, { SDL_GAMEPAD_BUTTON_DPAD_RIGHT, {} } },
        { GamepadInput::DPadLeft, { SDL_GAMEPAD_BUTTON_DPAD_LEFT, {} } },
        { GamepadInput::LeftStick,
          { SDL_GAMEPAD_BUTTON_LEFT_STICK, { SDL_GAMEPAD_AXIS_LEFTX, SDL_GAMEPAD_AXIS_LEFTY } } },
        { GamepadInput::RightStick,
          { SDL_GAMEPAD_BUTTON_RIGHT_STICK,
            { SDL_GAMEPAD_AXIS_RIGHTX, SDL_GAMEPAD_AXIS_RIGHTY } } },
        { GamepadInput::LeftBumper, { SDL_GAMEPAD_BUTTON_LEFT_SHOULDER, {} } },
        { GamepadInput::RightBumper, { SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, {} } },
        { GamepadInput::LeftTrigger, { {}, { SDL_GAMEPAD_AXIS_LEFT_TRIGGER } } },
        { GamepadInput::RightTrigger, { {}, { SDL_GAMEPAD_AXIS_RIGHT_TRIGGER } } },
        { GamepadInput::Start, { SDL_GAMEPAD_BUTTON_START, {} } },
        { GamepadInput::Select, { SDL_GAMEPAD_BUTTON_GUIDE, {} } },
        { GamepadInput::Touchpad, { SDL_GAMEPAD_BUTTON_TOUCHPAD, {} } }
    };

    static std::optional<GamepadInput> FindButton(SDL_GamepadButton button) {
        for (const auto& [id, info] : s_InputMapping) {
            if (info.AssociatedButton == button) {
                return id;
            }
        }

        return {};
    }

    static std::optional<GamepadInput> FindAxis(SDL_GamepadAxis axis, size_t& index) {
        for (const auto& [id, info] : s_InputMapping) {
            for (size_t i = 0; i < info.AssociatedAxes.size(); i++) {
                if (info.AssociatedAxes[i] == axis) {
                    index = i;
                    return id;
                }
            }
        }

        return {};
    }

    Client::Client()
        : Node("client"), m_Window(nullptr), m_Renderer(nullptr), m_SDLInitialized(false) {
        static constexpr uint32_t desiredFPS = 60;
        static constexpr std::chrono::duration<double> interval = 1s / desiredFPS;
        static constexpr uint32_t flags =
            SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_OPENGL;

        SDL_SetHint("SDL_HINT_JOYSTICK_THREAD", "1");
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD) != 0) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize SDL!");
            return;
        }

        m_Window = SDL_CreateWindow("Pancake swerve client", 1600, 900, flags);
        m_Renderer = SDL_CreateRenderer(m_Window, nullptr);
        m_SDLInitialized = true;

        SDL_SetRenderVSync(m_Renderer, SDL_TRUE);
        SDL_ShowWindow(m_Window);

        m_Publisher = create_publisher<pancake::msg::Input>("/pancake/client/control", 10);
        m_Timer = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(interval),
                                    std::bind(&Client::Update, this));

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        ImGui_ImplSDL3_InitForSDLRenderer(m_Window, m_Renderer);
        ImGui_ImplSDLRenderer3_Init(m_Renderer);

        AddView<Tuner>(this);
    }

    Client::~Client() {
        if (m_SDLInitialized) {
            ImGui_ImplSDLRenderer3_Shutdown();
            ImGui_ImplSDL3_Shutdown();
            ImGui::DestroyContext();
        }

        if (m_Window != nullptr) {
            SDL_DestroyWindow(m_Window);
        }

        for (auto [id, gamepad] : m_Gamepads) {
            SDL_CloseGamepad(gamepad);
        }

        SDL_Quit();
    }

    void Client::Update() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);

            switch (event.type) {
            case SDL_EVENT_QUIT:
                rclcpp::shutdown();
                break;
            case SDL_EVENT_GAMEPAD_ADDED:
                m_Gamepads[(uint32_t)event.gdevice.which] = SDL_OpenGamepad(event.gdevice.which);
                RCLCPP_INFO(get_logger(), "Connected controller: %s",
                            SDL_GetGamepadNameForID(event.gdevice.which));

                break;
            case SDL_EVENT_GAMEPAD_REMOVED:
                SDL_CloseGamepad(m_Gamepads[(uint32_t)event.gdevice.which]);
                m_Gamepads.erase((uint32_t)event.gdevice.which);

                break;
            case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
            case SDL_EVENT_GAMEPAD_BUTTON_UP:
                SendButton(event.gbutton);
                break;
            case SDL_EVENT_GAMEPAD_AXIS_MOTION:
                SendAxis(event.gaxis);
                break;
            }
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        for (const auto& [id, view] : m_Views) {
            view->Update();
        }

        SDL_SetRenderDrawColorFloat(m_Renderer, 0.f, 0.f, 0.f, 1.f);
        SDL_RenderClear(m_Renderer);

        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), m_Renderer);

        SDL_RenderPresent(m_Renderer);
    }

    void Client::SendButton(const SDL_GamepadButtonEvent& event) {
        auto button = (SDL_GamepadButton)event.button;
        auto gamepad = SDL_GetGamepadFromID(event.which);

        const char* buttonName = SDL_GetGamepadStringForButton(button);
        const char* gamepadName = SDL_GetGamepadName(gamepad);

        RCLCPP_INFO(get_logger(), "%s sent button %s: %s", gamepadName,
                    event.state == SDL_PRESSED ? "down" : "up", buttonName);

        auto id = FindButton(button);
        if (!id.has_value()) {
            RCLCPP_WARN(get_logger(), "Could not find binding for button: %s", buttonName);

            return;
        }

        pancake::msg::Input input;
        input.id = (uint8_t)id.value();
        input.down = event.state == SDL_PRESSED;
        input.up = event.state == SDL_RELEASED;
        input.pressed = SDL_GetGamepadButton(gamepad, button) != 0;

        const auto& info = s_InputMapping.at(id.value());
        for (size_t i = 0; i < info.AssociatedAxes.size(); i++) {
            auto axis = info.AssociatedAxes[i];

            int64_t value = SDL_GetGamepadAxis(gamepad, axis);
            input.axes[i] = (float)value / std::numeric_limits<int16_t>::max();
        }

        m_Publisher->publish(input);
    }

    void Client::SendAxis(const SDL_GamepadAxisEvent& event) {
        auto axis = (SDL_GamepadAxis)event.axis;
        auto gamepad = SDL_GetGamepadFromID(event.which);

        const char* axisName = SDL_GetGamepadStringForAxis(axis);
        RCLCPP_INFO(get_logger(), "Axis %s: %f", axisName,
                    (float)event.value / std::numeric_limits<int16_t>::max());

        size_t index;
        auto id = FindAxis(axis, index);
        if (!id.has_value()) {
            const char* axisName = SDL_GetGamepadStringForAxis(axis);
            RCLCPP_WARN(get_logger(), "Could not get binding for axis: %s", axisName);

            return;
        }

        pancake::msg::Input input;
        input.id = (uint8_t)id.value();
        input.down = input.up = false;

        const auto& info = s_InputMapping.at(id.value());
        input.pressed = info.AssociatedButton.has_value()
                            ? SDL_GetGamepadButton(gamepad, info.AssociatedButton.value()) != 0
                            : false;

        for (size_t i = 0; i < info.AssociatedAxes.size(); i++) {
            int64_t value;
            if (i == index) {
                value = event.value;
            } else {
                auto currentAxis = info.AssociatedAxes[i];
                value = SDL_GetGamepadAxis(gamepad, currentAxis);
            }

            input.axes[i] = (float)value / std::numeric_limits<int16_t>::max();
        }

        m_Publisher->publish(input);
    }
} // namespace pancake::client