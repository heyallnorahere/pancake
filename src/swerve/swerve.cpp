#include "pancake/swerve/swerve.h"

#include "pancake/config.h"

#include <chrono>
#include <string>

#include <rclcpp/serialization.hpp>

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

    Swerve::Swerve() : Node("swerve") {
        Drivetrain::Config config;
        config.Drive = config.Rotation = {};

        // measurements taken from CAD
        // https://cad.onshape.com/documents/a3570f35688a1cf16e8e4419/v/433e7b2f8e33ea9906b8780e/e/a1fbf0c2138da401bd4bce14
        config.Network = "can0";
        config.WheelRadius = 1.5f * 0.0254f; // in meters
        config.Drive.GearRatio = 2.f / 5.f;
        config.Rotation.GearRatio = -1.f / 48.f;
        config.EncoderConfig.Mode = RotationEncoderMode::Output;
        config.EncoderConfig.GearRatio = 1.f;

        if (!LoadConfig(get_name(), config)) {
            SaveConfig(get_name(), config);
        }

        m_Drivetrain = std::make_shared<Drivetrain>(config, false);

        const auto& modules = m_Drivetrain->GetModules();
        for (size_t i = 0; i < modules.size(); i++) {
            auto modulePath = "/pancake/swerve/module/mod" + std::to_string(i);

            ModuleTelemetry telemetry;
            telemetry.Target =
                create_generic_publisher(modulePath + "/target", "pancake/ModuleState", 10);
            telemetry.State =
                create_generic_publisher(modulePath + "/state", "pancake/ModuleState", 10);

            m_ModuleTelemetry.push_back(telemetry);
        }

        m_RequestSubscriber = create_generic_subscription(
            "/pancake/swerve/request", "pancake/SwerveRequest", 10,
            [&](std::shared_ptr<rclcpp::SerializedMessage> message) {
                rclcpp::Serialization<pancake::msg::SwerveRequest> serialization;
                pancake::msg::SwerveRequest request;
                serialization.deserialize_message(message.get(), &request);

                m_Drivetrain->SetRequest(request);
            });

        m_OdometryPublisher =
            create_generic_publisher("/pancake/odometry/state", "pancake/OdometryState", 10);

        m_ResetSubscriber = create_generic_subscription(
            "/pancake/odometry/reset", "pancake/OdometryState", 10,
            [&](std::shared_ptr<rclcpp::SerializedMessage> message) {
                rclcpp::Serialization<pancake::msg::OdometryState> serialization;
                pancake::msg::OdometryState state;
                serialization.deserialize_message(message.get(), &state);

                m_Drivetrain->ResetOdometry(state);
            });

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
    }

    void Swerve::Update() {
        auto now = std::chrono::high_resolution_clock::now();
        auto delta = now - m_LastUpdate.value_or(now);

        m_LastUpdate = now;
        m_Drivetrain->Update(std::chrono::duration_cast<std::chrono::duration<float>>(delta));

        rclcpp::Serialization<pancake::msg::OdometryState> odometrySerialization;
        rclcpp::SerializedMessage odometryMessage;

        const auto& odometry = m_Drivetrain->GetOdometry();
        odometrySerialization.serialize_message(&odometry, &odometryMessage);

        m_OdometryPublisher->publish(odometryMessage);

        rclcpp::Serialization<pancake::msg::ModuleState> stateSerialization;
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

            rclcpp::SerializedMessage stateMessage, targetMessage;
            stateSerialization.serialize_message(&sentState, &stateMessage);
            stateSerialization.serialize_message(&sentTarget, &targetMessage);

            telemetry.State->publish(stateMessage);
            telemetry.Target->publish(targetMessage);
        }
    }
}; // namespace pancake::swerve