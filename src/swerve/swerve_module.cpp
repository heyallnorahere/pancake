#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
    SwerveModule::SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation)
        : m_Drive(drive), m_Rotation(rotation), m_DriveController(drive.ControllerPID),
          m_RotationController(rotation.ControllerPID), m_State({}), m_Target({}) {}

    void SwerveModule::Update() {
        float rotationMotorVelocity = /* placeholder */ 0.f;
        float wheelWellVelocity = -rotationMotorVelocity * m_Rotation.GearRatio;

        m_RotationController.SetSetpoint(-m_Target.WheelAngle / m_Rotation.GearRatio);
        m_DriveController.SetSetpoint(m_Target.WheelAngularVelocity / m_Drive.GearRatio +
                                      wheelWellVelocity);

        float driveMotorVelocity = /* placeholder */ 0.f;
        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage,
                                m_DriveController.Evaluate(driveMotorVelocity));

        float rotationMotorPosition = /* placeholder */ 0.f;
        m_Rotation.Motor->Setpoint(rev::SetpointType::Velocity,
                                   m_RotationController.Evaluate(rotationMotorPosition));

        m_State.WheelAngularVelocity = (driveMotorVelocity - wheelWellVelocity) * m_Drive.GearRatio;
        m_State.WheelAngle = -rotationMotorPosition * m_Drive.GearRatio;
    }

    void SwerveModule::SetTarget(const ModuleState& target) {
        m_Target = target;
        Update();
    }
}; // namespace pancake::swerve