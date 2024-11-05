#pragma once

#include "pancake/swerve/drivetrain.h"

namespace pancake::swerve {
    struct ModuleTelemetry {
        rclcpp::Publisher<pancake::msg::RobotTransform>::SharedPtr Meta;
        rclcpp::Publisher<pancake::msg::ModuleState>::SharedPtr Target, State;
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve() = default;

        Swerve(const Swerve&) = delete;
        Swerve& operator=(const Swerve&) = delete;

    private:
        void Update();

        rclcpp::Service<pancake::srv::PIDSVA>::SharedPtr CreateModuleGainService(
            const std::string& path, MotorConstants<float>* constants);

        void ModuleGains(MotorConstants<float>* constants,
                         std::shared_ptr<pancake::srv::PIDSVA_Request> request,
                         std::shared_ptr<pancake::srv::PIDSVA_Response> response);

        std::shared_ptr<Drivetrain> m_Drivetrain;

        rclcpp::Subscription<pancake::msg::SwerveRequest>::SharedPtr m_RequestSubscriber;
        rclcpp::Subscription<pancake::msg::OdometryState>::SharedPtr m_ResetSubscriber;
        rclcpp::Publisher<pancake::msg::OdometryState>::SharedPtr m_OdometryPublisher;
        rclcpp::Publisher<pancake::msg::DrivetrainMeta>::SharedPtr m_MetaPublisher;

        rclcpp::Service<pancake::srv::PIDSVA>::SharedPtr m_DriveTuning, m_RotationTuning;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::optional<std::chrono::high_resolution_clock::time_point> m_LastUpdate;

        std::vector<ModuleTelemetry> m_ModuleTelemetry;
    };
} // namespace pancake::swerve