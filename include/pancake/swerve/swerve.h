#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/swerve/drivetrain.h"

#include <memory>
#include <chrono>
#include <optional>

namespace pancake::swerve {
    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve() = default;

        Swerve(const Swerve&) = delete;
        Swerve& operator=(const Swerve&) = delete;

    private:
        void Update();

        std::shared_ptr<Drivetrain> m_Drivetrain;

        rclcpp::Subscription<pancake::msg::SwerveRequest>::SharedPtr m_RequestSubscriber;
        rclcpp::Subscription<pancake::msg::OdometryState>::SharedPtr m_ResetSubscriber;
        rclcpp::Publisher<pancake::msg::OdometryState>::SharedPtr m_OdometryPublisher;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::optional<std::chrono::high_resolution_clock::time_point> m_LastUpdate;
    };
} // namespace pancake::swerve