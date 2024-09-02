#include "pancake/swerve/swerve.h"

#include "pancake/config.h"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace pancake {
    void from_json(const nlohmann::json& src, Vector2& dst) {
        src["X"].get_to(dst.X);
        src["Y"].get_to(dst.Y);
    }

    void to_json(nlohmann::json& dst, const Vector2& src) {
        dst["X"] = src.X;
        dst["Y"] = src.Y;
    }
}; // namespace pancake

namespace pancake::swerve {
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
        src["DriveID"].get_to(dst.Drive);
        src["RotationID"].get_to(dst.Rotation);
    }

    void from_json(const nlohmann::json& src, Drivetrain::Config& dst) {
        src["WheelRadius"].get_to(dst.WheelRadius);
        src["Drive"].get_to(dst.Drive);
        src["Rotation"].get_to(dst.Rotation);
        src["Modules"].get_to(dst.Modules);
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
        dst["DriveID"] = src.Drive;
        dst["RotationID"] = src.Rotation;
    }

    void to_json(nlohmann::json& dst, const Drivetrain::Config& src) {
        dst["WheelRadius"] = src.WheelRadius;
        dst["Drive"] = src.Drive;
        dst["Rotation"] = src.Rotation;
        dst["Modules"] = src.Modules;
    }

    Swerve::Swerve() : Node("swerve") {
        Drivetrain::Config config;
        config.Drive = config.Rotation = {};

        // measurements taken from CAD
        // https://cad.onshape.com/documents/a3570f35688a1cf16e8e4419/v/4001f8a0b9bee7a60796b187/e/a1fbf0c2138da401bd4bce14?renderMode=0&uiState=66c665ab45ca2447cfe6c702
        config.WheelRadius = 1.5f * 0.0254f; // in meters
        config.Drive.GearRatio = 2.f / 5.f;
        config.Rotation.GearRatio = -1.f / 48.f;

        if (!LoadConfig(get_name(), config)) {
            SaveConfig(get_name(), config);
        }

        m_Drivetrain = std::make_shared<Drivetrain>(config, false);

        const auto& modules = m_Drivetrain->GetModules();
        for (size_t i = 0; i < modules.size(); i++) {
            auto modulePath = "/pancake/swerve/module/" + std::to_string(i);

            ModuleTelemetry telemetry;
            telemetry.Target = create_publisher<pancake::msg::ModuleState>(modulePath + "/target", 10);
            telemetry.State = create_publisher<pancake::msg::ModuleState>(modulePath + "/state", 10);
            
            m_ModuleTelemetry.push_back(telemetry);
        }

        m_RequestSubscriber = create_subscription<pancake::msg::SwerveRequest>(
            "/pancake/swerve/request", 10,
            std::bind(&Drivetrain::SetRequest, m_Drivetrain.get(), std::placeholders::_1));

        m_OdometryPublisher =
            create_publisher<pancake::msg::OdometryState>("/pancake/odometry/state", 10);

        m_ResetSubscriber = create_subscription<pancake::msg::OdometryState>(
            "/pancake/odometry/reset", 10,
            std::bind(&Drivetrain::ResetOdometry, m_Drivetrain.get(), std::placeholders::_1));

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
    }

    void Swerve::Update() {
        auto now = std::chrono::high_resolution_clock::now();
        auto delta = now - m_LastUpdate.value_or(now);
        m_LastUpdate = now;

        m_Drivetrain->Update(std::chrono::duration_cast<std::chrono::duration<float>>(delta));
        m_OdometryPublisher->publish(m_Drivetrain->GetOdometry());

        const auto& modules = m_Drivetrain->GetModules();
        for (size_t i = 0; i < modules.size(); i++) {
            const auto& module = modules[i].Module;
            const auto& telemetry = m_ModuleTelemetry[i];

            pancake::msg::ModuleState sentState, sentTarget;
            const auto& state = module->GetState();
            const auto& target = module->GetTarget();

            sentState.angle = state.WheelAngle;
            sentState.wheel_angular_velocity = state.WheelAngularVelocity;

            sentTarget.angle = target.WheelAngle;
            sentTarget.wheel_angular_velocity = target.WheelAngularVelocity;

            telemetry.State->publish(sentState);
            telemetry.Target->publish(sentTarget);
        }
    }
}; // namespace pancake::swerve