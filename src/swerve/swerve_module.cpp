#include "pancakepch.h"
#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
    static float Signum(float x) { return x > 0.f ? 1.f : -1.f; }

    float SwerveModule::NormalizeAngle(float angle) {
        const float pi = std::numbers::pi_v<float>;
        while (std::abs(angle) > pi) {
            angle -= pi * 2.f * Signum(angle);
        }

        return angle;
    }

    static float ErrorBound(float gearRatio, float value) {
        return SwerveModule::NormalizeAngle(value) / gearRatio;
    }

    SwerveModule::SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation,
                               const RotationEncoderConfig& encoderConfig)
        : m_Drive(drive), m_Rotation(rotation), m_DriveController(drive.Constants.Feedback),
          m_RotationController(rotation.Constants.Feedback),
          m_DriveFeedforward(drive.Constants.Feedforward),
          m_RotationFeedforward(rotation.Constants.Feedforward), m_EncoderConfig(encoderConfig) {
        m_Target.WheelAngle = 0.f;
        m_Target.WheelAngularVelocity = 0.f;

        m_DriveEncoder = std::make_shared<rev::Encoder>(m_Drive.Motor);
        m_RotationEncoder = std::make_shared<rev::Encoder>(m_Rotation.Motor);
        m_DutyCycleEncoder =
            std::make_shared<rev::Encoder>(m_Rotation.Motor, rev::EncoderMode::DutyCycle);

        // set error bound with rotation gear ratio
        // see SwerveModule::Update
        m_RotationController.SetErrorBound(
            std::bind(ErrorBound, rotation.GearRatio, std::placeholders::_1));
    }

    void SwerveModule::Update() {
        // angular velocity of the wheel well
        float rotationMotorVelocity = m_RotationEncoder->GetMotorVelocity();
        float wheelWellVelocity = rotationMotorVelocity * m_Rotation.GearRatio;

        // retrieve position of the wheel well
        // if the duty cycle encoder does not report the output position, but instead the motor
        // position, use a gear ratio
        // in practice, this will never happen, because most duty cycle encoders report from 0 to 1
        // rotations
        float encoderPosition = m_DutyCycleEncoder->GetMotorPosition() / m_EncoderConfig.GearRatio;
        float wheelPosition = m_EncoderConfig.Mode != RotationEncoderMode::Output
                                  ? encoderPosition * m_Rotation.GearRatio
                                  : encoderPosition;

        // clamp from -pi to pi so that the motors don't do extra work to get to the same position
        wheelPosition = NormalizeAngle(wheelPosition);

        // cosine compensation
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
        float angleDifference = m_Target.WheelAngle - wheelPosition;
        float cosAngleDifference = std::cos(angleDifference);

        // calculate setpoint for drive PID controller
        // we compensate for wheel well velocity outside of the gear ratio compensation;
        // transmission from the motor to the wheel well has a 1:1 gear ratio
        float velocityFactor = cosAngleDifference / m_Drive.GearRatio;
        float desiredMotorVelocity =
            m_Target.WheelAngularVelocity * velocityFactor - wheelWellVelocity;

        // controller setpoints
        m_RotationController.SetSetpoint(m_Target.WheelAngle);
        m_DriveController.SetSetpoint(desiredMotorVelocity);

        // calculate drive voltage with the desired and actual motor velocity
        float driveMotorVelocity = m_DriveEncoder->GetMotorVelocity();
        float driveFeedforward = m_DriveFeedforward.Evaluate(desiredMotorVelocity);
        float drivePID = m_DriveController.Evaluate(driveMotorVelocity);
        float driveVoltage = drivePID + driveFeedforward;

        // voltage limits
        driveVoltage = std::clamp(driveVoltage, -m_Drive.VoltageLimit, m_Drive.VoltageLimit);

        // motors cant overcome static friction at low voltage
        // why waste energy. just dont give any power when we dont need to
        if (std::abs(driveVoltage) < m_Drive.VoltageDeadzone) {
            driveVoltage = 0.f;
        }

        // pass through voltage to the motor controller
        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage, driveVoltage);

        // same shebang as the drive motor except we use an error bound
        // the error bound corrects for the gear ratio, and normalizes the angle between -pi and pi
        float rotationPID = m_RotationController.Evaluate(wheelPosition);
        float rotationFeedforward =
            m_RotationFeedforward.Evaluate(ErrorBound(m_Rotation.GearRatio, angleDifference));

        float rotationVoltage = rotationPID + rotationFeedforward;
        rotationVoltage =
            std::clamp(rotationVoltage, -m_Rotation.VoltageLimit, m_Rotation.VoltageLimit);

        if (std::abs(rotationVoltage) < m_Rotation.VoltageDeadzone) {
            rotationVoltage = 0.f;
        }

        m_Rotation.Motor->Setpoint(rev::SetpointType::Voltage, rotationVoltage);

        // read back instantaneous measurements
        m_State.WheelAngularVelocity = (driveMotorVelocity - wheelWellVelocity) * m_Drive.GearRatio;
        m_State.WheelAngle = wheelPosition;
    }

    void SwerveModule::SetTarget(const ModuleState& target) {
        // set target and normalize wheel angle
        m_Target = target;
        m_Target.WheelAngle = NormalizeAngle(m_Target.WheelAngle);

        const float pi = std::numbers::pi_v<float>;
        float angleDifference = NormalizeAngle(m_Target.WheelAngle - m_State.WheelAngle);

        // if the wheel has to move more than 90 degrees to get to the target angle, there is
        // definitely a faster way to get there

        // subtract 180 degrees and invert the velocity to achieve the same force vector while not
        // forcing the motors to perform unnecessary work

        if (std::abs(angleDifference) > pi / 2.f) {
            m_Target.WheelAngle -= pi * Signum(angleDifference);
            m_Target.WheelAngularVelocity *= -1.f;
        }
    }

    void SwerveModule::Retune(const MotorConstants<float>& drive,
                              const MotorConstants<float>& rotation) {
        m_DriveController.GetPID() = drive.Feedback;
        m_DriveFeedforward.GetSVA() = drive.Feedforward;

        m_RotationController.GetPID() = rotation.Feedback;
        m_RotationFeedforward.GetSVA() = rotation.Feedforward;
    }
} // namespace pancake::swerve
