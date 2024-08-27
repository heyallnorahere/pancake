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
    };

    class Drivetrain {
    public:
        Drivetrain(bool sim);
        ~Drivetrain();

        Drivetrain(const Drivetrain&) = delete;
        Drivetrain& operator=(const Drivetrain&) = delete;

        void SetRequest(const pancake::msg::SwerveRequest& request);
        void ResetOdometry(const std::optional<pancake::msg::OdometryState>& state = {});

        void Update(const std::chrono::duration<float>& delta);

        const pancake::msg::OdometryState& GetOdometry() const { return m_Odometry; }

    private:
        void AddModules();
        void AddModule(uint8_t driveID, uint8_t rotationID, const Vector2& centerOffset);

        std::vector<SwerveModuleMeta> m_Modules;
        PID<float> m_DrivePID, m_RotationPID;
        float m_DriveGearRatio, m_RotationGearRatio;
        float m_WheelRadius;

        pancake::msg::SwerveRequest m_Request;
        pancake::msg::OdometryState m_Odometry;

        bool m_Sim;
        std::vector<std::shared_ptr<rev::sim::SparkMaxSim>> m_SimHandlers;
    };
} // namespace pancake::swerve