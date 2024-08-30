#include "pancake/swerve/drivetrain.h"

#include <rclcpp/rclcpp.hpp>

#include <numbers>

namespace pancake::swerve {
    Drivetrain::Drivetrain(const Config& config, bool sim) : m_Config(config), m_Sim(sim) {
        for (const auto& desc : m_Config.Modules) {
            AddModule(desc);
        }
    }

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
            target.WheelAngularVelocity = relativeVelocity.Length() / m_Config.WheelRadius;

            meta.Module->SetTarget(target);
            meta.Module->Update();

            const auto& state = meta.Module->GetState();
            float wheelAngle = rotationalOffset + state.WheelAngle;
            float wheelAngularVelocity = state.WheelAngularVelocity;
            float moduleVelocityLength = wheelAngularVelocity * m_Config.WheelRadius;

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

    void Drivetrain::AddModule(const SwerveModuleDesc& desc) {
        static const std::string network = "can0";

        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> handlers;
        if (m_Sim) {
            handlers = { std::make_shared<rev::sim::SparkMaxSim>(desc.Drive),
                         std::make_shared<rev::sim::SparkMaxSim>(desc.Rotation) };
        }

        SwerveMotor driveMotor;
        driveMotor.Motor = std::make_shared<rev::SparkMax>(desc.Drive, network);
        driveMotor.Constants = m_Config.Drive.Constants;
        driveMotor.GearRatio = m_Config.Drive.GearRatio;

        SwerveMotor rotationMotor;
        rotationMotor.Motor = std::make_shared<rev::SparkMax>(desc.Rotation, network);
        rotationMotor.Constants = m_Config.Rotation.Constants;
        rotationMotor.GearRatio = m_Config.Rotation.GearRatio;

        SwerveModuleMeta meta;
        meta.Module = std::make_shared<SwerveModule>(driveMotor, rotationMotor);
        meta.CenterOffset = desc.CenterOffset;
        meta.MotorIDs = { desc.Drive, desc.Rotation };

        m_Modules.push_back(meta);
        m_SimHandlers.insert(m_SimHandlers.end(), handlers.begin(), handlers.end());
    }
} // namespace pancake::swerve