#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/vector2.h"
#include "pancake/swerve/swerve_module.h"
#include "pancake/swerve/pid_controller.h"
#include "pancake/msg/chassis_speeds.hpp"

#include <memory>
#include <chrono>

namespace pancake::swerve {
    struct SwerveModuleMeta {
        std::shared_ptr<SwerveModule> Module;
        Vector2 CenterOffset;
    };

    struct ChassisSpeeds {
        Vector2 Linear; // m/s
        float Angular;  // rad/s
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve();

        void SetSpeeds(const pancake::msg::ChassisSpeeds& speeds);

    private:
        void Update();

        void AddModules();
        void AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset);

        std::vector<SwerveModuleMeta> m_Modules;
        PID<float> m_DrivePID, m_RotationPID;
        float m_DriveGearRatio, m_RotationGearRatio;
        float m_WheelRadius;

        pancake::msg::ChassisSpeeds m_ChassisSpeeds;
        float m_ChassisRotation;

        rclcpp::Subscription<pancake::msg::ChassisSpeeds>::SharedPtr m_Subscriber;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::chrono::high_resolution_clock::time_point m_LastUpdate;
    };
}; // namespace pancake::swerve