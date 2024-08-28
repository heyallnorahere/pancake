#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
    SwerveModule::SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation)
        : m_Drive(drive), m_Rotation(rotation), m_DriveController(drive.ControllerPID),
          m_RotationController(rotation.ControllerPID) {
        m_Target.WheelAngle = 0.f;
        m_Target.WheelAngularVelocity = 0.f;
    }

    void SwerveModule::Update() {
        const auto& driveEncoder = m_Drive.Motor->GetEncoder();
        const auto& rotationEncoder = m_Rotation.Motor->GetEncoder();

        float rotationMotorVelocity = rotationEncoder.GetMotorVelocity();
        float wheelWellVelocity = -rotationMotorVelocity * m_Rotation.GearRatio;

        m_RotationController.SetSetpoint(-m_Target.WheelAngle / m_Rotation.GearRatio);
        m_DriveController.SetSetpoint(m_Target.WheelAngularVelocity / m_Drive.GearRatio +
                                      wheelWellVelocity);

        float driveMotorVelocity = driveEncoder.GetMotorVelocity();
        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage,
                                m_DriveController.Evaluate(driveMotorVelocity));

        float rotationMotorPosition = rotationEncoder.GetMotorPosition();
        m_Rotation.Motor->Setpoint(rev::SetpointType::Voltage,
                                   m_RotationController.Evaluate(rotationMotorPosition));

        m_State.WheelAngularVelocity = (driveMotorVelocity - wheelWellVelocity) * m_Drive.GearRatio;
        m_State.WheelAngle = -rotationMotorPosition * m_Drive.GearRatio;
    }

    void SwerveModule::SetTarget(const ModuleState& target) { m_Target = target; }
} // namespace pancake::swerve