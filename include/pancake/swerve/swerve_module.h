#pragma once
#include <memory>

#include <librevfree.h>

#include "pancake/swerve/pid_controller.h"

namespace pancake::swerve {
    struct SwerveMotor {
        std::shared_ptr<rev::SparkMax> Motor;
        PID<float> ControllerPID;
        float GearRatio;
    };

    struct ModuleState {
        float TargetWheelAngle;
        float TargetWheelAngularVelocity;
    };

    class SwerveModule {
    public:
        SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation);

        void Update();
        void SetState(const ModuleState& state);

    private:
        ModuleState m_State;

        SwerveMotor m_Drive, m_Rotation;
        PIDController<float> m_DriveController, m_RotationController;
    };
}; // namespace pancake::swerve