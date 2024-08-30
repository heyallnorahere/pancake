#include "pancake/swerve/swerve.h"

#include "pancake/config.h"

#include <chrono>

using namespace std::chrono_literals;

namespace pancake {
    void from_json(const nlohmann::json& src, Vector2& dst) {
        src["X"].get_to(dst.X);
        src["Y"].get_to(dst.Y);
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
    }

    void from_json(const nlohmann::json& src, Drivetrain::Config& dst) {
        src["WheelRadius"].get_to(dst.WheelRadius);
        src["Drive"].get_to(dst.Drive);
        src["Rotation"].get_to(dst.Rotation);
        src["Modules"].get_to(dst.Modules);
    }

    Swerve::Swerve() : Node("swerve") {
        Drivetrain::Config config;
        config.Drive = config.Rotation = {};

        config.WheelRadius = 1.5f * 0.0254f; // in meters
        config.Drive.GearRatio = 2.f / 5.f;
        config.Rotation.GearRatio = -1.f / 48.f;

        if (!LoadConfig(get_name(), config)) {
            // ???
        }

        m_Drivetrain = std::make_shared<Drivetrain>(config, false);

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
    }
}; // namespace pancake::swerve