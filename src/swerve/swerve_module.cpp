#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
    SwerveModule::SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation)
        : m_Drive(drive), m_Rotation(rotation), m_DriveController(drive.ControllerPID),
          m_RotationController(rotation.ControllerPID) {
        m_State.TargetWheelAngularVelocity = 0.f;
        m_State.TargetWheelAngle = 0.f;
    }

    void SwerveModule::Update() {
        float rotationMotorVelocity = /* placeholder */ 0.f;
        float wheelWellVelocity = rotationMotorVelocity * m_Rotation.GearRatio *
                                  -1.f; // rotation axle causes opposite direction rotation

        m_RotationController.SetSetpoint(m_State.TargetWheelAngle / m_Rotation.GearRatio);
        m_DriveController.SetSetpoint(m_State.TargetWheelAngularVelocity / m_Drive.GearRatio +
                                      wheelWellVelocity);

        float driveMotorVelocity = /* placeholder */ 0.f;
        m_Drive.Motor->Setpoint(rev::SetpointType::Voltage,
                                m_DriveController.Evaluate(driveMotorVelocity));

        m_Rotation.Motor->Setpoint(rev::SetpointType::Velocity,
                                   m_RotationController.Evaluate(rotationMotorVelocity));
    }
}; // namespace pancake::swerve