#include "pancakepch.h"
#include "pancake/swerve/swerve.h"

#include "pancake/config.h"

#include <librevfree/CanBus.h>

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
        src["VoltageLimit"].get_to(dst.VoltageLimit);
        src["VoltageDeadzone"].get_to(dst.VoltageDeadzone);
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
        dst["VoltageLimit"] = src.VoltageLimit;
        dst["VoltageDeadzone"] = src.VoltageDeadzone;
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

    Swerve::Swerve(bool sim) : Node("swerve") {
        Drivetrain::Config config;
        config.Drive = config.Rotation = {};

        // measurements taken from CAD
        // https://cad.onshape.com/documents/a3570f35688a1cf16e8e4419/v/433e7b2f8e33ea9906b8780e/e/a1fbf0c2138da401bd4bce14
        config.Network = "can0";
        config.WheelRadius = 1.5f * 0.0254f; // in meters
        config.Drive.GearRatio = 2.f / 5.f;
        config.Rotation.GearRatio = -1.f / 48.f;
        config.Drive.VoltageLimit = config.Rotation.VoltageLimit = 12.f;
        config.Drive.VoltageDeadzone = config.Rotation.VoltageDeadzone = 0.02f;
        config.EncoderConfig.Mode = RotationEncoderMode::Output;
        config.EncoderConfig.GearRatio = 1.f;

        auto bus = rev::CanBus::Get(config.Network);
        sim |= !bus->IsOpen();

        if (!LoadConfig(get_name(), config)) {
            if (sim) {
                for (size_t i = 0; i < 4; i++) {
                    float angle = std::numbers::pi_v<float> * (1.f / 4.f + i / 2.f);

                    SwerveModuleDesc simModule;
                    simModule.CenterOffset = { std::cos(angle), std::sin(angle) };
                    simModule.RotationalOffset = angle;
                    simModule.Drive = 2 + i * 2;
                    simModule.Rotation = 1 + i * 2;

                    config.Modules.push_back(simModule);
                }
            }

            SaveConfig(get_name(), config);
        }

        m_Drivetrain = std::make_shared<Drivetrain>(config, sim);

        const auto& modules = m_Drivetrain->GetModules();
        for (size_t i = 0; i < modules.size(); i++) {
            auto modulePath = "/pancake/swerve/module/mod" + std::to_string(i);

            ModuleTelemetry telemetry;
            telemetry.Meta =
                create_publisher<pancake::msg::RobotTransform>(modulePath + "/meta", 10);
            telemetry.Target =
                create_publisher<pancake::msg::ModuleState>(modulePath + "/target", 10);
            telemetry.State =
                create_publisher<pancake::msg::ModuleState>(modulePath + "/state", 10);

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

        auto& drivetrainConfig = m_Drivetrain->GetConfig();
        m_DriveTuning =
            CreateTuningService("/pancake/config/drive/tuning", &drivetrainConfig.Drive.Constants);

        m_RotationTuning = CreateTuningService("/pancake/config/rotation/tuning",
                                               &drivetrainConfig.Rotation.Constants);

        m_SaveConfig = create_subscription<std_msgs::msg::Bool>(
            "/pancake/config/save/swerve", 10, [this](const std_msgs::msg::Bool& save) {
                const auto& config = m_Drivetrain->GetConfig();

                if (save.data) {
                    SaveConfig(get_name(), config);
                }
            });

        m_MetaPublisher =
            create_publisher<pancake::msg::DrivetrainMeta>("/pancake/swerve/meta", 10);

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));

        m_KillListener = create_subscription<std_msgs::msg::Bool>(
            "/pancake/client/kill", 10, [](const std_msgs::msg::Bool& kill) {
                if (kill.data) {
                    throw std::runtime_error("Client sent kill command!");
                }
            });
    }

    void Swerve::Update() {
        static auto lastLog = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();

        if (now - lastLog > 1s) {
            RCLCPP_INFO(get_logger(), "Swerve node is alive");
            lastLog = now;
        }

        auto delta = now - m_LastUpdate.value_or(now);
        m_LastUpdate = now;
        m_Drivetrain->Update(std::chrono::duration_cast<std::chrono::duration<float>>(delta));

        const auto& odometry = m_Drivetrain->GetOdometry();
        RCLCPP_INFO(get_logger(), "Rotation: %f degrees",
                    odometry.transform.rotation * 180.f / std::numbers::pi_v<float>);
        m_OdometryPublisher->publish(odometry);

        const auto& modules = m_Drivetrain->GetModules();
        const auto& config = m_Drivetrain->GetConfig();

        pancake::msg::DrivetrainMeta meta;
        meta.module_count = (uint32_t)modules.size();
        meta.wheel_radius = config.WheelRadius;
        m_MetaPublisher->publish(meta);

        for (size_t i = 0; i < modules.size(); i++) {
            const auto& moduleData = modules[i];
            const auto& module = moduleData.Module;
            const auto& telemetry = m_ModuleTelemetry[i];

            pancake::msg::RobotTransform moduleMeta;
            moduleMeta.x = moduleData.CenterOffset.X;
            moduleMeta.y = moduleData.CenterOffset.Y;
            moduleMeta.rotation = moduleData.RotationalOffset;

            pancake::msg::ModuleState sentState, sentTarget;
            const auto& state = module->GetState();
            const auto& target = module->GetTarget();

            sentState.angle = state.WheelAngle;
            sentState.wheel_angular_velocity = state.WheelAngularVelocity;

            sentTarget.angle = target.WheelAngle;
            sentTarget.wheel_angular_velocity = target.WheelAngularVelocity;

            telemetry.Meta->publish(moduleMeta);
            telemetry.State->publish(sentState);
            telemetry.Target->publish(sentTarget);
        }
    }

    std::unique_ptr<TuningService> Swerve::CreateTuningService(const std::string& path,
                                                               MotorConstants<float>* constants) {
        auto service = std::make_unique<TuningService>();
        service->Constants = constants;

        auto ptr = service.get();

        service->PIDPublisher = create_publisher<pancake::msg::PID>(path + "/pid/get", 10);
        service->PIDSubscriber = create_subscription<pancake::msg::PID>(
            path + "/pid/set", 10, [this, ptr](const pancake::msg::PID& pid) { SetPID(ptr, pid); });

        service->SVAPublisher = create_publisher<pancake::msg::SVA>(path + "/sva/get", 10);
        service->SVASubscriber = create_subscription<pancake::msg::SVA>(
            path + "/sva/set", 10, [this, ptr](const pancake::msg::SVA& sva) { SetSVA(ptr, sva); });

        return service;
    }

    void Swerve::SetPID(TuningService* service, const pancake::msg::PID& pid) {
        auto& dst = service->Constants->Feedback;

        dst.Proportional = pid.p;
        dst.Integral = pid.i;
        dst.Derivative = pid.d;

        service->PIDPublisher->publish(pid);

        RetuneModules();
    }

    void Swerve::SetSVA(TuningService* service, const pancake::msg::SVA& sva) {
        auto& dst = service->Constants->Feedforward;

        dst.Sign = sva.s;
        dst.Velocity = sva.v;
        dst.Acceleration = sva.a;

        service->SVAPublisher->publish(sva);

        RetuneModules();
    }

    void Swerve::RetuneModules() {
        const auto& config = m_Drivetrain->GetConfig();
        const auto& modules = m_Drivetrain->GetModules();

        for (const auto& module : modules) {
            module.Module->Retune(config.Drive.Constants, config.Rotation.Constants);
        }
    }
}; // namespace pancake::swerve