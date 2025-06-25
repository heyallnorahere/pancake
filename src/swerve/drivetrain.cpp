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
        RCLCPP_DEBUG(logger, "Setting request:");
        RCLCPP_DEBUG(logger, "\tRequest type: %s", request.absolute ? "absolute" : "relative");

        RCLCPP_DEBUG(logger, "\tX velocity: %f m/s", linear.X);
        RCLCPP_DEBUG(logger, "\tY velocity: %f m/s", linear.Y);
        RCLCPP_DEBUG(logger, "\tLinear velocity: %f m/s", linear.Length());
        RCLCPP_DEBUG(logger, "\tLinear velocity angle: %f degrees",
                     std::atan2(linear.Y, linear.X) * 180.f / std::numbers::pi_v<float>);

        RCLCPP_DEBUG(logger, "\tAngular velocity: %f rad/s", request.velocity.angular_velocity);

        m_Request = request;
    }

    void Drivetrain::ResetOdometry(const std::optional<pancake::msg::OdometryState>& state) {
        m_Odometry = state.value_or(pancake::msg::OdometryState());
    }

    void Drivetrain::Update(const std::chrono::duration<float>& delta) {
        // update sim handlers, if there are any
        for (const auto& handler : m_SimHandlers) {
            handler->Update(delta);
        }

        // data from swerve request
        Vector2 requestedLinearVelocity;
        requestedLinearVelocity.X = m_Request.velocity.x;
        requestedLinearVelocity.Y = m_Request.velocity.y;
        float requestedAngularVelocity = m_Request.velocity.angular_velocity;

        // reset instantaneous odometry
        m_Odometry.velocity.x = 0.f;
        m_Odometry.velocity.y = 0.f;
        m_Odometry.velocity.angular_velocity = 0.f;

        for (const auto& meta : m_Modules) {
            float centerOffsetAngle = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);

            // v = omega * R
            float distanceToCenter = meta.CenterOffset.Length();
            float desiredRotationalLinear = requestedAngularVelocity * distanceToCenter;

            // offset from up on the left stick to the forward vector on the module
            float controllerSpaceRotation = meta.RotationalOffset;
            if (m_Request.absolute) {
                // if the speed request is absolute, then the left stick is talking in terms of the
                // field, not the robot heading
                controllerSpaceRotation += m_Odometry.transform.rotation;
            }

            // convert the requested linear velocity to module space
            // rotate the vector so that +x is on the module's x axis
            Vector2 relativeLinear = requestedLinearVelocity.Rotate(-controllerSpaceRotation);

            // +y in module space is in the +theta direction (counter-clockwise).
            // however, this vector is in the rotation space of the cartesian theta angle of the
            // vector offset from the robot's center of rotation. we need to rotate it so that its
            // completely in module space
            float offsetDifference = meta.RotationalOffset - centerOffsetAngle;
            Vector2 relativeAngular =
                Vector2(0.f, desiredRotationalLinear).Rotate(-offsetDifference);

            // total desired module space velocity vector
            Vector2 relativeVelocity = relativeLinear + relativeAngular;

            // v = omega * R
            // omega = v/R
            float moduleVelocity = relativeVelocity.Length();
            float moduleAngularVelocity = moduleVelocity / m_Config.WheelRadius;

            // convert module linear velocity from x,y to theta,r
            ModuleState target;
            if (moduleVelocity < std::numeric_limits<float>::epsilon()) {
                target.WheelAngularVelocity = 0.f;
                target.WheelAngle = meta.Module->GetState().WheelAngle;
            } else {
                target.WheelAngularVelocity = moduleAngularVelocity;
                target.WheelAngle = std::atan2(relativeVelocity.Y, relativeVelocity.X);
            }

            // update module
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
                absoluteModuleVelocity.Dot(meta.CenterOffset) / m_Modules.size();
        }

        m_Odometry.transform.x += m_Odometry.velocity.x * delta.count();
        m_Odometry.transform.y += m_Odometry.velocity.y * delta.count();
        m_Odometry.transform.rotation += m_Odometry.velocity.angular_velocity * delta.count();
        m_Odometry.transform.rotation =
            std::fmod(m_Odometry.transform.rotation, std::numbers::pi_v<float> / 2.f);
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
        driveMotor.VoltageLimit = m_Config.Drive.VoltageLimit;
        driveMotor.VoltageDeadzone = m_Config.Drive.VoltageDeadzone;

        SwerveMotor rotationMotor;
        rotationMotor.Motor = std::make_shared<rev::SparkMax>(desc.Rotation, m_Config.Network);
        rotationMotor.Constants = m_Config.Rotation.Constants;
        rotationMotor.GearRatio = m_Config.Rotation.GearRatio;
        rotationMotor.VoltageLimit = m_Config.Rotation.VoltageLimit;
        rotationMotor.VoltageDeadzone = m_Config.Rotation.VoltageDeadzone;

        meta.Module =
            std::make_shared<SwerveModule>(driveMotor, rotationMotor, m_Config.EncoderConfig);

        m_Modules.push_back(meta);
        m_SimHandlers.insert(m_SimHandlers.end(), handlers.begin(), handlers.end());
    }
} // namespace pancake::swerve