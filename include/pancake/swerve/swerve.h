#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/msg/input.hpp"
#include "pancake/swerve/swerve_module.h"
#include "pancake/swerve/pid_controller.h"

#include <memory>
#include <chrono>

namespace pancake::swerve {
    struct Vector2 {
        float X, Y;
    };

    struct SwerveModuleMeta {
        std::shared_ptr<SwerveModule> Module;
        Vector2 CenterOffset;
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve();

    private:
        void InputReceived(const pancake::msg::Input& input);

        void Update();

        void AddModules();
        void AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset);

        std::vector<SwerveModuleMeta> m_Modules;
        PID<float> m_DrivePID, m_RotationPID;
        float m_DriveGearRatio, m_RotationGearRatio;
        float m_WheelRadius;
        float m_ChassisRotation;

        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_Subscriber;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::chrono::high_resolution_clock::time_point m_LastUpdate;
    };
}; // namespace pancake::swerve