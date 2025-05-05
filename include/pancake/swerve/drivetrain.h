#pragma once

#include <librevfree.h>
#include <librevfree/sim/SparkMaxSim.h>

#include "pancake/vector2.h"
#include "pancake/swerve/pid_controller.h"
#include "pancake/swerve/swerve_module.h"

#include <nlohmann/json.hpp>

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

    template <typename _Ty>
    void from_json(const nlohmann::json& src, PID<_Ty>& dst) {
        src["P"].get_to(dst.Proportional);
        src["I"].get_to(dst.Integral);
        src["D"].get_to(dst.Derivative);
    }

    template <typename _Ty>
    void from_json(const nlohmann::json& src, SVA<_Ty>& dst) {
        src["S"].get_to(dst.Sign);
        src["V"].get_to(dst.Velocity);
        src["A"].get_to(dst.Acceleration);
    }

    template <typename _Ty>
    void from_json(const nlohmann::json& src, MotorConstants<_Ty>& dst) {
        src["PID"].get_to(dst.Feedback);
        src["Feedforward"].get_to(dst.Feedforward);
    }

    void from_json(const nlohmann::json& src, SwerveFunction& dst) {
        src["Constants"].get_to(dst.Constants);
        src["GearRatio"].get_to(dst.GearRatio);
    }

    void from_json(const nlohmann::json& src, SwerveModuleDesc& dst) {
        src["CenterOffset"].get_to(dst.CenterOffset);
        src["RotationalOffset"].get_to(dst.RotationalOffset);
        src["DriveID"].get_to(dst.Drive);
        src["RotationID"].get_to(dst.Rotation);
    }

    static const std::unordered_map<std::string, RotationEncoderMode> s_ModeMap = {
        { "Motor", RotationEncoderMode::Motor }, { "Output", RotationEncoderMode::Output }
    };

    void from_json(const nlohmann::json& src, RotationEncoderConfig& dst) {
        src["GearRatio"].get_to(dst.GearRatio);

        auto modeName = src["Mode"].get<std::string>();
        auto it = s_ModeMap.find(modeName);

        if (it != s_ModeMap.end()) {
            dst.Mode = it->second;
        } else {
            throw std::runtime_error("Invalid encoder mode!");
        }
    }

    void from_json(const nlohmann::json& src, Drivetrain::Config& dst) {
        src["Network"].get_to(dst.Network);
        src["WheelRadius"].get_to(dst.WheelRadius);
        src["Drive"].get_to(dst.Drive);
        src["Rotation"].get_to(dst.Rotation);
        src["Modules"].get_to(dst.Modules);
        src["RotationEncoder"].get_to(dst.EncoderConfig);
    }

    template <typename _Ty>
    void to_json(nlohmann::json& dst, const PID<_Ty>& src) {
        dst["P"] = src.Proportional;
        dst["I"] = src.Integral;
        dst["D"] = src.Derivative;
    }

    template <typename _Ty>
    void to_json(nlohmann::json& dst, const SVA<_Ty>& src) {
        dst["S"] = src.Sign;
        dst["V"] = src.Velocity;
        dst["A"] = src.Acceleration;
    }

    template <typename _Ty>
    void to_json(nlohmann::json& dst, const MotorConstants<_Ty>& src) {
        dst["PID"] = src.Feedback;
        dst["Feedforward"] = src.Feedforward;
    }

    void to_json(nlohmann::json& dst, const SwerveFunction& src) {
        dst["Constants"] = src.Constants;
        dst["GearRatio"] = src.GearRatio;
    }

    void to_json(nlohmann::json& dst, const SwerveModuleDesc& src) {
        dst["CenterOffset"] = src.CenterOffset;
        dst["RotationalOffset"] = src.RotationalOffset;
        dst["DriveID"] = src.Drive;
        dst["RotationID"] = src.Rotation;
    }

    void to_json(nlohmann::json& dst, const RotationEncoderConfig& src) {
        dst["GearRatio"] = src.GearRatio;

        bool modeFound = false;
        for (const auto& [name, mode] : s_ModeMap) {
            if (mode == src.Mode) {
                dst["Mode"] = name;

                modeFound = true;
                break;
            }
        }

        if (!modeFound) {
            throw std::runtime_error("Invalid encoder mode!");
        }
    }

    void to_json(nlohmann::json& dst, const Drivetrain::Config& src) {
        dst["Network"] = src.Network;
        dst["WheelRadius"] = src.WheelRadius;
        dst["Drive"] = src.Drive;
        dst["Rotation"] = src.Rotation;
        dst["Modules"] = src.Modules;
        dst["RotationEncoder"] = src.EncoderConfig;
    }
} // namespace pancake::swerve