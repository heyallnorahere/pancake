#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pancake/msg/input.hpp"
#include "pancake/swerve/swerve_module.h"
#include "pancake/swerve/pid_controller.h"

#include <memory>

namespace pancake::swerve {
    struct SwerveModuleMeta {
        std::shared_ptr<SwerveModule> Module;
        float RotationalOffset;
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve();

    private:
        void InputReceived(const pancake::msg::Input& input);

        void Update();

        void AddModules();
        void AddModule(uint8_t driveID, uint8_t rotationID, float rotationalOffset);

        std::vector<SwerveModuleMeta> m_Modules;
        PID<float> m_DrivePID, m_RotationPID;
        float m_DriveGearRatio, m_RotationGearRatio;

        rclcpp::Subscription<pancake::msg::Input>::SharedPtr m_Subscriber;
    };
}; // namespace pancake::swerve