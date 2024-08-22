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
        float WheelAngle;
        float WheelAngularVelocity;
    };

    class SwerveModule {
    public:
        SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation);

        void Update();
        void SetTarget(const ModuleState& target);

        const ModuleState& GetState() const { return m_State; }
        const ModuleState& GetTarget() const { return m_Target; }

    private:
        ModuleState m_State, m_Target;

        SwerveMotor m_Drive, m_Rotation;
        PIDController<float> m_DriveController, m_RotationController;
    };
}; // namespace pancake::swerve