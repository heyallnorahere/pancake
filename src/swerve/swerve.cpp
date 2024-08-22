#include "pancake/swerve/swerve.h"

#include <numbers>

using namespace std::chrono_literals;

namespace pancake::swerve {
    Swerve::Swerve() : Node("swerve") {
        AddModules();

        m_ChassisSpeeds.x = 0.f;
        m_ChassisSpeeds.y = 0.f;
        m_ChassisSpeeds.angular_velocity = 0.f;
        m_ChassisRotation = 0.f;

        m_Subscriber = create_subscription<pancake::msg::ChassisSpeeds>(
            "/pancake/swerve/speed", 10,
            std::bind(&Swerve::SetSpeeds, this, std::placeholders::_1));

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
    }

    Swerve::~Swerve() {
        // todo: shut down swerve stuff
    }

    void Swerve::SetSpeeds(const pancake::msg::ChassisSpeeds& input) {
        RCLCPP_INFO(get_logger(), "Setting speeds:");
        RCLCPP_INFO(get_logger(), "\tX velocity: %f m/s", input.x);
        RCLCPP_INFO(get_logger(), "\tY velocity: %f m/s", input.y);
        RCLCPP_INFO(get_logger(), "\tLinear velocity: %f m/s",
                    std::sqrt(input.x * input.x + input.y * input.y));
        RCLCPP_INFO(get_logger(), "\tLinear velocity angle: %f degrees",
                    std::atan2(input.y, input.x) * 180.f / std::numbers::pi_v<float>);
        RCLCPP_INFO(get_logger(), "\tAngular velocity: %s rad/s", input.angular_velocity);
    }

    void Swerve::Update() {
        auto now = std::chrono::high_resolution_clock::now();
        auto delta = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_LastUpdate);
        m_LastUpdate = now;

        for (const auto& meta : m_Modules) {
            meta.Module->Update();

            const auto& state = meta.Module->GetState();
            float rotationalOffset = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);
            float wheelAngle = rotationalOffset + state.WheelAngle;
            float wheelAngularVelocity = state.WheelAngularVelocity;
            float moduleVelocityLength = wheelAngularVelocity * m_WheelRadius;

            Vector2 moduleVelocity;
            moduleVelocity.X = moduleVelocityLength * std::cos(wheelAngle);
            moduleVelocity.Y = moduleVelocityLength * std::sin(wheelAngle);

            Vector2 perpendicularToCenter;
            perpendicularToCenter.X = -meta.CenterOffset.Y;
            perpendicularToCenter.Y = meta.CenterOffset.X;

            float dot = moduleVelocity.X * perpendicularToCenter.X +
                        moduleVelocity.Y * perpendicularToCenter.Y;

            m_ChassisRotation += dot / m_Modules.size();
        }
    }

    void Swerve::AddModules() {
        // todo: set pid
        m_DrivePID.Proportional = 0.f;
        m_DrivePID.Integral = 0.f;
        m_DrivePID.Derivative = 0.f;

        m_RotationPID.Proportional = 0.f;
        m_RotationPID.Integral = 0.f;
        m_RotationPID.Derivative = 0.f;

        // measurements taken from CAD
        // https://cad.onshape.com/documents/a3570f35688a1cf16e8e4419/v/4001f8a0b9bee7a60796b187/e/a1fbf0c2138da401bd4bce14?renderMode=0&uiState=66c665ab45ca2447cfe6c702
        m_DriveGearRatio = 2.f / 5.f;
        m_RotationGearRatio = 1.f / 48.f;
        m_WheelRadius = 1.5f * 0.0254f;

        AddModule(0, 1, { 0.f, 0.f });
    }

    void Swerve::AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset) {
        static const std::string network = "can0";

        SwerveMotor driveMotor;
        driveMotor.Motor = std::make_shared<rev::SparkMax>(driveID, network);
        driveMotor.ControllerPID = m_DrivePID;
        driveMotor.GearRatio = m_DriveGearRatio;

        SwerveMotor rotationMotor;
        rotationMotor.Motor = std::make_shared<rev::SparkMax>(rotationID, network);
        rotationMotor.ControllerPID = m_RotationPID;
        rotationMotor.GearRatio = m_RotationGearRatio;

        SwerveModuleMeta meta;
        meta.Module = std::make_shared<SwerveModule>(driveMotor, rotationMotor);
        meta.CenterOffset = centerOffset;

        m_Modules.push_back(meta);
    }
}; // namespace pancake::swerve