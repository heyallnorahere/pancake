#pragma once
#include <memory>

#include <librevfree.h>

#include "pancake/swerve/pid_controller.h"
#include "pancake/swerve/feedforward.h"

namespace pancake::swerve {
    template <typename _Ty>
    struct MotorConstants {
        PID<float> Feedback;
        SVA<float> Feedforward; // not used for rotation
    };

    struct SwerveMotor {
        std::shared_ptr<rev::SparkMax> Motor;
        float GearRatio;
        MotorConstants<float> Constants;
    };

    struct ModuleState {
        float WheelAngle;
        float WheelAngularVelocity;
    };

    enum class RotationEncoderMode { Motor, Output };

    struct RotationEncoderConfig {
        RotationEncoderMode Mode;
        float GearRatio;
    };

    class SwerveModule {
    public:
        SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation,
                     const RotationEncoderConfig& encoderConfig);
        ~SwerveModule() = default;

        SwerveModule(const SwerveModule&) = delete;
        SwerveModule& operator=(const SwerveModule&) = delete;

        void Update();
        void SetTarget(const ModuleState& target);

        const ModuleState& GetState() const { return m_State; }
        const ModuleState& GetTarget() const { return m_Target; }

    private:
        ModuleState m_State, m_Target;

        SwerveMotor m_Drive, m_Rotation;
        PIDController<float> m_DriveController, m_RotationController;
        Feedforward<float> m_DriveFeedforward;
        RotationEncoderConfig m_EncoderConfig;

        std::shared_ptr<rev::Encoder> m_DriveEncoder;
        std::shared_ptr<rev::Encoder> m_RotationEncoder, m_DutyCycleEncoder;
    };
}; // namespace pancake::swerve