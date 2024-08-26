#include "pancake/swerve/swerve.h"

#include <numbers>

using namespace std::chrono_literals;

namespace pancake::swerve {
    Swerve::Swerve() : Node("swerve") {
        AddModules();

        m_RequestSubscriber = create_subscription<pancake::msg::SwerveRequest>(
            "/pancake/swerve/request", 10,
            std::bind(&Swerve::SetRequest, this, std::placeholders::_1));

        m_OdometryPublisher =
            create_publisher<pancake::msg::OdometryState>("/pancake/odometry/state", 10);

        m_ResetSubscriber = create_subscription<pancake::msg::OdometryState>(
            "/pancake/odometry/reset", 10,
            std::bind(&Swerve::ResetOdometry, this, std::placeholders::_1));

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
    }

    Swerve::~Swerve() {
        // todo: shut down swerve stuff
    }

    void Swerve::SetRequest(const pancake::msg::SwerveRequest& request) {
        Vector2 linear;
        linear.X = request.velocity.x;
        linear.Y = request.velocity.y;

        RCLCPP_INFO(get_logger(), "Setting request:");
        RCLCPP_INFO(get_logger(), "\tRequest type: %s", request.absolute ? "Absolute" : "Relative");
        RCLCPP_INFO(get_logger(), "\tX velocity: %f m/s", linear.X);
        RCLCPP_INFO(get_logger(), "\tY velocity: %f m/s", linear.Y);
        RCLCPP_INFO(get_logger(), "\tLinear velocity: %f m/s", linear.Length());
        RCLCPP_INFO(get_logger(), "\tLinear velocity angle: %f degrees",
                    std::atan2(linear.Y, linear.X) * 180.f / std::numbers::pi_v<float>);
        RCLCPP_INFO(get_logger(), "\tAngular velocity: %f rad/s",
                    request.velocity.angular_velocity);

        m_Request = request;
    }

    void Swerve::ResetOdometry(const std::optional<pancake::msg::OdometryState>& state) {
        m_Odometry = state.value_or(pancake::msg::OdometryState());
    }

    void Swerve::Update() {
        auto now = std::chrono::high_resolution_clock::now();
        auto delta = std::chrono::duration_cast<std::chrono::duration<float>>(now - m_LastUpdate);
        m_LastUpdate = now;

        Vector2 requestedLinearVelocity;
        requestedLinearVelocity.X = m_Request.velocity.x;
        requestedLinearVelocity.Y = m_Request.velocity.y;

        m_Odometry.velocity.x = 0.f;
        m_Odometry.velocity.y = 0.f;
        m_Odometry.velocity.angular_velocity = 0.f;

        for (const auto& meta : m_Modules) {
            float rotationalOffset = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);
            float distanceToCenter = meta.CenterOffset.Length();

            // radians/s * m = m/s around arc
            float angularRotationVelocity = m_Request.velocity.angular_velocity * distanceToCenter;
            float moduleRotation = rotationalOffset;

            if (m_Request.absolute) {
                moduleRotation += m_Odometry.transform.rotation;
            }

            static const float piOver2 = std::numbers::pi_v<float> / 2.f;
            Vector2 perpendicular = meta.CenterOffset.Rotate(piOver2).Normalize();

            // we want this rotated INVERSELY by the chassis rotation
            Vector2 linear = requestedLinearVelocity.Rotate(-moduleRotation);
            Vector2 angular = angularRotationVelocity * perpendicular;
            Vector2 velocity = linear + angular;

            ModuleState target;
            target.WheelAngle = std::atan2(velocity.Y, velocity.X);
            target.WheelAngularVelocity = velocity.Length();

            meta.Module->SetTarget(target);
            meta.Module->Update();

            const auto& state = meta.Module->GetState();
            float wheelAngle = rotationalOffset + state.WheelAngle;
            float wheelAngularVelocity = state.WheelAngularVelocity;
            float moduleVelocityLength = wheelAngularVelocity * m_WheelRadius;

            Vector2 moduleVelocity;
            moduleVelocity.X = moduleVelocityLength * std::cos(wheelAngle);
            moduleVelocity.Y = moduleVelocityLength * std::sin(wheelAngle);

            auto absoluteModuleVelocity = moduleVelocity.Rotate(m_Odometry.transform.rotation);
            float velocityDot = moduleVelocity.Dot(perpendicular);

            m_Odometry.velocity.x += absoluteModuleVelocity.X / m_Modules.size();
            m_Odometry.velocity.y += absoluteModuleVelocity.Y / m_Modules.size();
            m_Odometry.velocity.angular_velocity +=
                velocityDot * distanceToCenter / m_Modules.size();
        }

        m_Odometry.transform.x += m_Odometry.velocity.x * delta.count();
        m_Odometry.transform.y += m_Odometry.velocity.y * delta.count();
        m_Odometry.transform.rotation += m_Odometry.velocity.angular_velocity * delta.count();
        m_OdometryPublisher->publish(m_Odometry);
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
        m_WheelRadius = 1.5f * 0.0254f; // in meters

        float angle = std::numbers::pi_v<float> * 1.75f;
        AddModule(0, 1, { std::cos(angle), std::sin(angle) });
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