#pragma once

#include <librevfree.h>
#include <librevfree/sim/SparkMaxSim.h>

#include "pancake/vector2.h"
#include "pancake/swerve/pid_controller.h"
#include "pancake/swerve/swerve_module.h"

namespace pancake::swerve {
    struct SwerveModuleMeta {
        std::shared_ptr<SwerveModule> Module;
        std::vector<uint8_t> MotorIDs;
        Vector2 CenterOffset;
        float RotationalOffset;
    };

    struct SwerveModuleDesc {
        uint8_t Drive, Rotation;
        Vector2 CenterOffset;
        float RotationalOffset;
    };

    struct SwerveFunction {
        MotorConstants<float> Constants;
        float GearRatio;
        float VoltageLimit, VoltageDeadzone;
    };

    struct SwerveFeatures {
        bool UseCosineCompensation;
    };

    /*
     * Manages the position and targets of swerve modules.
     * This is the highest "swerve" class and should be instantiated as part of the robot behavior.
     * Receives swerve requests and changes targets of swerve modules accordingly.
     */
    class Drivetrain {
    public:
        struct Config {
            std::string Network;
            float WheelRadius;
            SwerveFunction Drive, Rotation;
            std::vector<SwerveModuleDesc> Modules;
            RotationEncoderConfig EncoderConfig;
            SwerveFeatures Features;
        };

        Drivetrain(const Config& config, bool sim);
        ~Drivetrain();

        Drivetrain(const Drivetrain&) = delete;
        Drivetrain& operator=(const Drivetrain&) = delete;

        /*
         * Requests a target trajectory of the drivetrain.
         */
        void SetRequest(const pancake::msg::SwerveRequest& request);

        /*
         * Resets the odometry state to the provided state, or the default if none is provided.
         */
        void ResetOdometry(const std::optional<pancake::msg::OdometryState>& state = {});

        /*
         * Periodic update. Should be run often enough to reasonably control drivetrain.
         * delta: Interval since last update. For first call, set to 0.
         */
        void Update(const std::chrono::duration<float>& delta);

        /*
         * Returns information on the robot's position, heading, and velocities.
         */
        const pancake::msg::OdometryState& GetOdometry() const { return m_Odometry; }

        /*
         * Returns metadata and control objects of swerve modules.
         */
        const std::vector<SwerveModuleMeta>& GetModules() const { return m_Modules; }

        /*
         * Returns a reference to the drivetrain config.
         * Use with caution: modification of values may not reflect behavioral updates.
         */
        Config& GetConfig() { return m_Config; }

        /*
         * Returns an immutable reference to the drivetrain config.
         */
        const Config& GetConfig() const { return m_Config; }

    private:
        void AddModule(const SwerveModuleDesc& desc);

        Vector2 CalculateLinearVelocity(const SwerveModuleMeta& meta) const;
        Vector2 CalculateAngularVelocity(const SwerveModuleMeta& meta) const;

        std::vector<SwerveModuleMeta> m_Modules;
        Config m_Config;

        pancake::msg::SwerveRequest m_Request;
        pancake::msg::OdometryState m_Odometry;

        bool m_Sim;
        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> m_SimHandlers;
    };
} // namespace pancake::swerve
