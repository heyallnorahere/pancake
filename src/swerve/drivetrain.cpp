#include "pancake/swerve/drivetrain.h"

#include <rclcpp/rclcpp.hpp>

#include <numbers>

namespace pancake::swerve {
    Drivetrain::Drivetrain(bool sim) : m_Sim(sim) { AddModules(); }

    Drivetrain::~Drivetrain() {
        // we want motors to be dealt with before sim handlers
        m_Modules.clear();
        m_SimHandlers.clear();
    }

    void Drivetrain::SetRequest(const pancake::msg::SwerveRequest& request) {
        Vector2 linear;
        linear.X = request.velocity.x;
        linear.Y = request.velocity.y;

        auto logger = rclcpp::get_logger("swerve");
        RCLCPP_INFO(logger, "Setting request:");
        RCLCPP_INFO(logger, "\tRequest type: %s", request.absolute ? "absolute" : "relative");

        RCLCPP_INFO(logger, "\tX velocity: %f m/s", linear.X);
        RCLCPP_INFO(logger, "\tY velocity: %f m/s", linear.Y);
        RCLCPP_INFO(logger, "\tLinear velocity: %f m/s", linear.Length());
        RCLCPP_INFO(logger, "\tLinear velocity angle: %f degrees",
                    std::atan2(linear.Y, linear.X) * 180.f / std::numbers::pi_v<float>);

        RCLCPP_INFO(logger, "\tAngular velocity: %f rad/s", request.velocity.angular_velocity);

        m_Request = request;
    }

    void Drivetrain::ResetOdometry(const std::optional<pancake::msg::OdometryState>& state) {
        m_Odometry = state.value_or(pancake::msg::OdometryState());
    }

    void Drivetrain::Update(const std::chrono::duration<float>& delta) {
        for (const auto& handler : m_SimHandlers) {
            handler->Update(delta);
        }

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
            Vector2 relativeLinear = requestedLinearVelocity.Rotate(-moduleRotation);
            Vector2 relativeAngular = { 0.f, angularRotationVelocity };
            Vector2 relativeVelocity = relativeLinear + relativeAngular;

            ModuleState target;
            target.WheelAngle = std::atan2(relativeVelocity.Y, relativeVelocity.X);
            target.WheelAngularVelocity = relativeVelocity.Length() / m_WheelRadius;

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
    }

    void Drivetrain::AddModules() {
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
        AddModule(1, 2, { std::cos(angle), std::sin(angle) });
    }

    void Drivetrain::AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset) {
        static const std::string network = "can0";

        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> handlers;
        if (m_Sim) {
            handlers = { std::make_shared<rev::sim::SparkMaxSim>(driveID),
                         std::make_shared<rev::sim::SparkMaxSim>(rotationID) };
        }

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
        meta.MotorIDs = { driveID, rotationID };

        m_Modules.push_back(meta);
        m_SimHandlers.insert(m_SimHandlers.end(), handlers.begin(), handlers.end());
    }
} // namespace pancake::swerve