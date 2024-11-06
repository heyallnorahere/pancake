#include "pancakepch.h"
#include "pancake/swerve/drivetrain.h"

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
            float centerOffsetAngle = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);
            float distanceToCenter = meta.CenterOffset.Length();

            // radians/s * m = m/s around arc
            float angularRotationVelocity = m_Request.velocity.angular_velocity * distanceToCenter;
            float moduleRotation = meta.RotationalOffset;

            if (m_Request.absolute) {
                moduleRotation += m_Odometry.transform.rotation;
            }

            // these vectors are in module space
            Vector2 relativeLinear = requestedLinearVelocity.Rotate(-moduleRotation);
            Vector2 relativeAngular = Vector2(0.f, angularRotationVelocity)
                                          .Rotate(centerOffsetAngle - meta.RotationalOffset);
            Vector2 relativeVelocity = relativeLinear + relativeAngular;

            auto time = std::chrono::system_clock::now().time_since_epoch();
            auto seconds = std::chrono::duration_cast<std::chrono::duration<float>>(time);

            float moduleVelocity = relativeVelocity.Length();
            ModuleState target;
            target.WheelAngularVelocity = moduleVelocity / m_Config.WheelRadius;
            target.WheelAngle = moduleVelocity < std::numeric_limits<float>::epsilon()
                                    ? meta.Module->GetState().WheelAngle
                                    : std::atan2(relativeVelocity.Y, relativeVelocity.X);

            meta.Module->SetTarget(target);
            meta.Module->Update();

            const auto& state = meta.Module->GetState();
            float robotWheelAngle = meta.RotationalOffset + state.WheelAngle;
            float wheelAngularVelocity = state.WheelAngularVelocity;
            float moduleVelocityLength = wheelAngularVelocity * m_Config.WheelRadius;

            Vector2 robotModuleVelocity;
            robotModuleVelocity.X = moduleVelocityLength * std::cos(robotWheelAngle);
            robotModuleVelocity.Y = moduleVelocityLength * std::sin(robotWheelAngle);

            auto absoluteModuleVelocity = robotModuleVelocity.Rotate(m_Odometry.transform.rotation);

            m_Odometry.velocity.x += absoluteModuleVelocity.X / m_Modules.size();
            m_Odometry.velocity.y += absoluteModuleVelocity.Y / m_Modules.size();
            m_Odometry.velocity.angular_velocity +=
                robotModuleVelocity.Dot(meta.CenterOffset.Rotate(std::numbers::pi_v<float> / 2.f)) /
                m_Modules.size();
        }

        m_Odometry.transform.x += m_Odometry.velocity.x * delta.count();
        m_Odometry.transform.y += m_Odometry.velocity.y * delta.count();
        m_Odometry.transform.rotation += m_Odometry.velocity.angular_velocity * delta.count();
    }

    void Drivetrain::AddModule(const SwerveModuleDesc& desc) {
        SwerveModuleMeta meta;
        meta.CenterOffset = desc.CenterOffset;
        meta.RotationalOffset = desc.RotationalOffset;
        meta.MotorIDs = { desc.Drive, desc.Rotation };

        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> handlers;
        if (m_Sim) {
            for (uint8_t id : meta.MotorIDs) {
                handlers.push_back(std::make_shared<rev::sim::SparkMaxSim>(id));
            }
        }

        SwerveMotor driveMotor;
        driveMotor.Motor = std::make_shared<rev::SparkMax>(desc.Drive, m_Config.Network);
        driveMotor.Constants = m_Config.Drive.Constants;
        driveMotor.GearRatio = m_Config.Drive.GearRatio;

        SwerveMotor rotationMotor;
        rotationMotor.Motor = std::make_shared<rev::SparkMax>(desc.Rotation, m_Config.Network);
        rotationMotor.Constants = m_Config.Rotation.Constants;
        rotationMotor.GearRatio = m_Config.Rotation.GearRatio;

        meta.Module =
            std::make_shared<SwerveModule>(driveMotor, rotationMotor, m_Config.EncoderConfig);

        m_Modules.push_back(meta);
        m_SimHandlers.insert(m_SimHandlers.end(), handlers.begin(), handlers.end());
    }
} // namespace pancake::swerve