#pragma once

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
        float GearRatio, VoltageLimit, VoltageDeadzone;
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

    /*
     * Controls a set of two motors that operate one swerve module.
     *
     */
    class SwerveModule {
    public:
        /*
         * Normalizes an angle to the range -pi to pi.
         */
        static float NormalizeAngle(float angle);

        /*
         * Creates a new instance. This object does not need to know if it's in a simulation or not.
         */
        SwerveModule(const SwerveMotor& drive, const SwerveMotor& rotation,
                     const RotationEncoderConfig& encoderConfig, bool useCosineCompensation);

        ~SwerveModule() = default;

        SwerveModule(const SwerveModule&) = delete;
        SwerveModule& operator=(const SwerveModule&) = delete;

        /*
         * Periodic update of module.
         */
        void Update();

        /*
         * Sets the desired state of the module. (internal: see m_Target)
         */
        void SetTarget(const ModuleState& target);

        /*
         * Returns the CURRENT state of the module.
         */
        const ModuleState& GetState() const { return m_State; }

        /*
         * Returns the DESIRED state of the module.
         */
        const ModuleState& GetTarget() const { return m_Target; }

        /*
         * Updates the tuning constants. (PID & SVA)
         */
        void Retune(const MotorConstants<float>& drive, const MotorConstants<float>& rotation);

    private:
        ModuleState m_State, m_Target;

        SwerveMotor m_Drive, m_Rotation;
        PIDController<float> m_DriveController, m_RotationController;
        Feedforward<float> m_DriveFeedforward, m_RotationFeedforward;
        RotationEncoderConfig m_EncoderConfig;
        bool m_UseCosineCompensation;

        std::shared_ptr<rev::Encoder> m_DriveEncoder;
        std::shared_ptr<rev::Encoder> m_RotationEncoder, m_DutyCycleEncoder;
    };
}; // namespace pancake::swerve
