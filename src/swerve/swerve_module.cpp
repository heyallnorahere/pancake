#include "pancakepch.h"
#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
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
    }

    static float Signum(float x) { return x > 0.f ? 1.f : -1.f; }

    static float NormalizeAngle(float angle) {
        const float pi = std::numbers::pi_v<float>;
        while (std::abs(angle) > pi) {
            angle -= pi * 2.f * Signum(angle);
        }

        return angle;
    }

    void SwerveModule::Update() {
        float rotationMotorVelocity = m_RotationEncoder->GetMotorVelocity();
        float wheelWellVelocity = rotationMotorVelocity * m_Rotation.GearRatio;

        float desiredRotation = m_Target.WheelAngle / m_Rotation.GearRatio;
        float desiredVelocity =
            m_Target.WheelAngularVelocity / m_Drive.GearRatio - wheelWellVelocity;

        m_RotationController.SetSetpoint(desiredRotation);
        m_DriveController.SetSetpoint(desiredVelocity);

        float driveMotorVelocity = m_DriveEncoder->GetMotorVelocity();
        float driveFeedforward = m_DriveFeedforward.Evaluate(desiredVelocity);
        float drivePID = m_DriveController.Evaluate(driveMotorVelocity);
        float driveVoltage = drivePID + driveFeedforward;

        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage, driveVoltage);

        float encoderPosition =
            NormalizeAngle(m_DutyCycleEncoder->GetMotorPosition() / m_EncoderConfig.GearRatio);

        float rotationMotorPosition = encoderPosition;
        float wheelPosition = encoderPosition;
        if (m_EncoderConfig.Mode == RotationEncoderMode::Output) {
            rotationMotorPosition /= m_Rotation.GearRatio;
        } else {
            wheelPosition *= m_Rotation.GearRatio;
        }

        float rotationPID = m_RotationController.Evaluate(rotationMotorPosition);
        float rotationFeedforward =
            m_RotationFeedforward.Evaluate(desiredRotation - rotationMotorPosition);

        float rotationVoltage = rotationPID + rotationFeedforward;
        m_Rotation.Motor->Setpoint(rev::SetpointType::Voltage, rotationVoltage);

        m_State.WheelAngularVelocity = (driveMotorVelocity - wheelWellVelocity) * m_Drive.GearRatio;
        m_State.WheelAngle = wheelPosition;
    }

    void SwerveModule::SetTarget(const ModuleState& target) {
        m_Target = target;
        m_Target.WheelAngle = NormalizeAngle(m_Target.WheelAngle);

        const float pi = std::numbers::pi_v<float>;
        float angleDifference = NormalizeAngle(m_Target.WheelAngle - m_State.WheelAngle);
        if (std::abs(angleDifference) > pi / 2.f) {
            m_Target.WheelAngle -= pi * Signum(m_Target.WheelAngle);
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