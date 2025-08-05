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

    static void DumpRequest(const pancake::msg::SwerveRequest& request) {
        Vector2 linear;
        linear.X = request.velocity.x;
        linear.Y = request.velocity.y;

        static const float rad2Deg = 180.f / std::numbers::pi_v<float>;
        float linearVelocity = linear.Length();
        float linearVelocityAngle = std::atan2(linear.Y, linear.X) * rad2Deg;
        float angularVelocity = request.velocity.angular_velocity * rad2Deg;

        auto logger = rclcpp::get_logger("swerve");
        RCLCPP_DEBUG(logger, "Setting request:");
        RCLCPP_DEBUG(logger, "\tRequest type: %s", request.absolute ? "absolute" : "relative");

        RCLCPP_DEBUG(logger, "\tX velocity: %f m/s", linear.X);
        RCLCPP_DEBUG(logger, "\tY velocity: %f m/s", linear.Y);
        RCLCPP_DEBUG(logger, "\tLinear velocity: %f m/s", linearVelocity);
        RCLCPP_DEBUG(logger, "\tLinear velocity angle: %f degrees", linearVelocityAngle);
        RCLCPP_DEBUG(logger, "\tAngular velocity: %f degrees/s", angularVelocity);
    }

    void Drivetrain::SetRequest(const pancake::msg::SwerveRequest& request) {
        // copy down request
        m_Request = request;

        // make sure we keep logs
        DumpRequest(request);
    }

    void Drivetrain::ResetOdometry(const std::optional<pancake::msg::OdometryState>& state) {
        // reset odometry
        // this mainly just resets the heading of the robot
        m_Odometry = state.value_or(pancake::msg::OdometryState());
    }

    Vector2 Drivetrain::CalculateLinearVelocity(const SwerveModuleMeta& meta) const {
        // offset from the robot's heading to the forward vector on the module
        float robotSpaceRotation = meta.RotationalOffset;

        // if the speed request is absolute, then the left stick is talking in terms of the
        // field, not the robot heading, so we offset the rotation offset by the offset from
        // "forward" on the field to the robot's heading
        if (m_Request.absolute) {
            robotSpaceRotation += m_Odometry.transform.rotation;
        }

        // vector from the input swerve request
        Vector2 desiredLinear;
        desiredLinear.X = m_Request.velocity.x;
        desiredLinear.Y = m_Request.velocity.y;

        // convert the requested linear velocity to module space
        // rotate the vector so that +x is on the module's x axis
        return desiredLinear.Rotate(-robotSpaceRotation);
    }

    Vector2 Drivetrain::CalculateAngularVelocity(const SwerveModuleMeta& meta) const {
        // desired angular velocity from the input swerve request
        float desiredAngular = m_Request.velocity.angular_velocity;

        // compute instantaneous linear velocity along edge of drivetrain as distance along an arc
        // C = theta * R
        // dC/dt = dtheta/dt * R
        // V = omega * R
        float distanceToCenter = meta.CenterOffset.Length();
        float desiredRotationalLinear = desiredAngular * distanceToCenter;

        // +y in module space is in the +theta direction (counter-clockwise).
        auto angularVector = Vector2(0.f, desiredRotationalLinear);

        // cartesian right-handed angle of center offset vector
        float centerOffsetAngle = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);

        // however, this vector is in the rotation space of the cartesian theta angle of the
        // vector offset from the robot's center of rotation. we need to rotate it so that its
        // completely in module space
        float offsetDifference = meta.RotationalOffset - centerOffsetAngle;
        return angularVector.Rotate(-offsetDifference);
    }

    void Drivetrain::Update(const std::chrono::duration<float>& delta) {
        // update sim handlers, if there are any
        for (const auto& handler : m_SimHandlers) {
            handler->Update(delta);
        }

        // reset instantaneous odometry
        m_Odometry.velocity.x = 0.f;
        m_Odometry.velocity.y = 0.f;
        m_Odometry.velocity.angular_velocity = 0.f;

        for (const auto& meta : m_Modules) {
            // calculate velocity vectors
            auto moduleLinear = CalculateLinearVelocity(meta);
            auto moduleAngular = CalculateAngularVelocity(meta);

            // total desired module space velocity vector
            Vector2 moduleVelocity = moduleLinear + moduleAngular;

            // use the distance along an arc formula to convert the ground velocity of the module to
            // the angular velocity of the wheel
            float desiredAngularVelocity = moduleVelocity.Length() / m_Config.WheelRadius;

            // convert module linear velocity from x,y to theta,r
            ModuleState target;
            if (desiredAngularVelocity < std::numeric_limits<float>::epsilon()) {
                target.WheelAngularVelocity = 0.f;
                target.WheelAngle = meta.Module->GetState().WheelAngle;
            } else {
                target.WheelAngularVelocity = desiredAngularVelocity;
                target.WheelAngle = std::atan2(moduleVelocity.Y, moduleVelocity.X);
            }

            // update module with new target
            meta.Module->SetTarget(target);
            meta.Module->Update();

            // retrieve data from module state
            const auto& state = meta.Module->GetState();
            float robotSpaceWheelAngle = meta.RotationalOffset + state.WheelAngle;
            float wheelAngularVelocity = state.WheelAngularVelocity;
            float moduleVelocityLength = wheelAngularVelocity * m_Config.WheelRadius;

            Vector2 robotModuleVelocity;
            robotModuleVelocity.X = moduleVelocityLength * std::cos(robotSpaceWheelAngle);
            robotModuleVelocity.Y = moduleVelocityLength * std::sin(robotSpaceWheelAngle);
            auto absoluteModuleVelocity = robotModuleVelocity.Rotate(m_Odometry.transform.rotation);

            size_t moduleCount = m_Modules.size();
            m_Odometry.velocity.x += absoluteModuleVelocity.X / moduleCount;
            m_Odometry.velocity.y += absoluteModuleVelocity.Y / moduleCount;
            m_Odometry.velocity.angular_velocity += std::sin(state.WheelAngle) *
                                                    moduleVelocityLength *
                                                    meta.CenterOffset.Length() / moduleCount;
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
                RCLCPP_INFO(rclcpp::get_logger("swerve"), "Creating sim handler for controller: %d",
                            (uint32_t)id);

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
            std::make_shared<SwerveModule>(driveMotor, rotationMotor, m_Config.EncoderConfig,
                                           m_Config.Features.UseCosineCompensation);

        m_Modules.push_back(meta);
        m_SimHandlers.insert(m_SimHandlers.end(), handlers.begin(), handlers.end());
    }
} // namespace pancake::swerve
