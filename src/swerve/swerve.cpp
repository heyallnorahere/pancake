#include "pancake/swerve/swerve.h"

using namespace std::chrono_literals;

namespace pancake::swerve {
    static float Dot(const Vector2& a, const Vector2& b) { return a.X * b.X + a.Y * b.Y; }

    Swerve::Swerve() : Node("swerve"), m_ChassisRotation(0.f) {
        AddModules();

        m_Subscriber = create_subscription<pancake::msg::Input>(
            "/pancake/swerve/control", 10,
            std::bind(&Swerve::InputReceived, this, std::placeholders::_1));

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
    }

    Swerve::~Swerve() {
        // todo: shut down swerve stuff
    }

    void Swerve::InputReceived(const pancake::msg::Input& input) {
        RCLCPP_INFO(get_logger(), "Input received: %u", (uint32_t)input.id);
        RCLCPP_INFO(get_logger(), "\tDown? %s", input.down ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tUp? %s", input.up ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tPressed? %s", input.pressed ? "Yes" : "No");
        RCLCPP_INFO(get_logger(), "\tAxis 0: %f", input.axes[0]);
        RCLCPP_INFO(get_logger(), "\tAxis 1: %f", input.axes[1]);
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

            m_ChassisRotation += Dot(moduleVelocity, perpendicularToCenter) / m_Modules.size();
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
        m_DriveGearRatio = 0.4f;
        m_RotationGearRatio = 3.f;
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