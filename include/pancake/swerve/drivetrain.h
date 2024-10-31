#pragma once

#include <optional>
#include <vector>
#include <chrono>

#include <librevfree.h>
#include <librevfree/sim/SparkMaxSim.h>

#include "pancake/vector2.h"
#include "pancake/swerve/pid_controller.h"
#include "pancake/swerve/swerve_module.h"
#include "pancake/msg/odometry_state.hpp"
#include "pancake/msg/swerve_request.hpp"

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
    };

    class Drivetrain {
    public:
        struct Config {
            std::string Network;
            float WheelRadius;
            SwerveFunction Drive, Rotation;
            std::vector<SwerveModuleDesc> Modules;
            RotationEncoderConfig EncoderConfig;
        };

        Drivetrain(const Config& config, bool sim);
        ~Drivetrain();

        Drivetrain(const Drivetrain&) = delete;
        Drivetrain& operator=(const Drivetrain&) = delete;

        void SetRequest(const pancake::msg::SwerveRequest& request);
        void ResetOdometry(const std::optional<pancake::msg::OdometryState>& state = {});

        void Update(const std::chrono::duration<float>& delta);

        const pancake::msg::OdometryState& GetOdometry() const { return m_Odometry; }
        const std::vector<SwerveModuleMeta>& GetModules() const { return m_Modules; }

        float GetWheelRadius() const { return m_Config.WheelRadius; }

        Config& GetConfig() { return m_Config; }
        const Config& GetConfig() const { return m_Config; }

    private:
        void AddModule(const SwerveModuleDesc& desc);

        std::vector<SwerveModuleMeta> m_Modules;
        Config m_Config;

        pancake::msg::SwerveRequest m_Request;
        pancake::msg::OdometryState m_Odometry;

        bool m_Sim;
        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> m_SimHandlers;
    };
} // namespace pancake::swerve