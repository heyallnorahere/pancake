#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pancake/vector2.h"
#include "pancake/swerve/swerve_module.h"
#include "pancake/swerve/pid_controller.h"
#include "pancake/msg/swerve_request.hpp"
#include "pancake/msg/odometry_state.hpp"

#include <memory>
#include <chrono>

namespace pancake::swerve {
    struct SwerveModuleMeta {
        std::shared_ptr<SwerveModule> Module;
        Vector2 CenterOffset;
    };

    class Swerve : public rclcpp::Node {
    public:
        Swerve();
        ~Swerve();

        void SetRequest(const pancake::msg::SwerveRequest& request);
        void ResetOdometry(const std::optional<pancake::msg::OdometryState>& state = {});

        void Update();

    private:
        void AddModules();
        void AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset);

        std::vector<SwerveModuleMeta> m_Modules;
        PID<float> m_DrivePID, m_RotationPID;
        float m_DriveGearRatio, m_RotationGearRatio;
        float m_WheelRadius;

        pancake::msg::SwerveRequest m_Request;
        pancake::msg::OdometryState m_Odometry;

        rclcpp::Subscription<pancake::msg::SwerveRequest>::SharedPtr m_RequestSubscriber;
        rclcpp::Subscription<pancake::msg::OdometryState>::SharedPtr m_ResetSubscriber;
        rclcpp::Publisher<pancake::msg::OdometryState>::SharedPtr m_OdometryPublisher;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::chrono::high_resolution_clock::time_point m_LastUpdate;
    };
}; // namespace pancake::swerve