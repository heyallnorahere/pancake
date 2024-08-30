#include "pancake/swerve/swerve_module.h"

#include <algorithm>
#include <numbers>

namespace pancake::swerve {
    SwerveModule::SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation)
        : m_Drive(drive), m_Rotation(rotation), m_DriveController(drive.Constants.Feedback),
          m_RotationController(rotation.Constants.Feedback),
          m_DriveFeedforward(drive.Constants.Feedforward),
          m_RotationFeedforward(rotation.Constants.Feedforward) {
        m_Target.WheelAngle = 0.f;
        m_Target.WheelAngularVelocity = 0.f;
    }

    static float Signum(float x) { return x > 0.f ? 1.f : -1.f; }

    void SwerveModule::Update() {
        const auto& driveEncoder = m_Drive.Motor->GetEncoder();
        const auto& rotationEncoder = m_Rotation.Motor->GetEncoder();

        float rotationMotorVelocity = rotationEncoder.GetMotorVelocity();
        float wheelWellVelocity = rotationMotorVelocity * m_Rotation.GearRatio;

        float desiredRotation = m_Target.WheelAngle / m_Rotation.GearRatio;
        float desiredVelocity =
            m_Target.WheelAngularVelocity / m_Drive.GearRatio + wheelWellVelocity;

        m_RotationController.SetSetpoint(desiredRotation);
        m_DriveController.SetSetpoint(desiredVelocity);

        float driveMotorVelocity = driveEncoder.GetMotorVelocity();
        float driveFeedforward = m_DriveFeedforward.Evaluate(desiredVelocity);

        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage,
                                std::clamp(m_DriveController.Evaluate(driveMotorVelocity),
                                           -driveFeedforward, driveFeedforward));

        float rotationMotorPosition = rotationEncoder.GetMotorPosition();
        float rotationFeedforward =
            m_RotationFeedforward.Evaluate(Signum(desiredRotation - rotationMotorPosition) *
                                           std::numbers::pi_v<float> * 2.f); // ???

        m_Rotation.Motor->Setpoint(rev::SetpointType::Voltage,
                                   std::clamp(m_RotationController.Evaluate(rotationMotorPosition),
                                              -rotationFeedforward, rotationFeedforward));

        m_State.WheelAngularVelocity = (driveMotorVelocity - wheelWellVelocity) * m_Drive.GearRatio;
        m_State.WheelAngle = rotationMotorPosition * m_Drive.GearRatio;
    }

    void SwerveModule::SetTarget(const ModuleState& target) { m_Target = target; }
} // namespace pancake::swerve