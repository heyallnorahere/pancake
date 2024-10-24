#pragma once
#include <rclcpp/node.hpp>

#include "pancake/swerve/drivetrain.h"
#include "pancake/msg/module_state.hpp"

#include <memory>
#include <chrono>
#include <optional>

namespace pancake::swerve {
    struct ModuleTelemetry {
        rclcpp::GenericPublisher::SharedPtr Target, State;
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve() = default;

        Swerve(const Swerve&) = delete;
        Swerve& operator=(const Swerve&) = delete;

    private:
        void Update();

        std::shared_ptr<Drivetrain> m_Drivetrain;

        rclcpp::GenericSubscription::SharedPtr m_RequestSubscriber;
        rclcpp::GenericSubscription::SharedPtr m_ResetSubscriber;
        rclcpp::GenericPublisher::SharedPtr m_OdometryPublisher;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::optional<std::chrono::high_resolution_clock::time_point> m_LastUpdate;

        std::vector<ModuleTelemetry> m_ModuleTelemetry;
    };
} // namespace pancake::swerve