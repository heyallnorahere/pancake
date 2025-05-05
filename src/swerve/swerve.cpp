#include "pancakepch.h"
#include "pancake/swerve/swerve.h"

#include "pancake/config.h"

#include <librevfree/CanBus.h>

namespace pancake::swerve {
    Swerve::Swerve(bool sim) : Node("swerve") {
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
            } else {
                SaveConfig(get_name(), config);
            }
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

        /* freezes node
        auto& drivetrainConfig = m_Drivetrain->GetConfig();
        m_DriveTuning = CreateModuleGainService("/pancake/config/drive/gains",
                                                &drivetrainConfig.Drive.Constants);

        m_RotationTuning = CreateModuleGainService("/pancake/config/rotation/gains",
                                                   &drivetrainConfig.Rotation.Constants);
        */

        m_MetaPublisher =
            create_publisher<pancake::msg::DrivetrainMeta>("/pancake/swerve/meta", 10);

        m_LastUpdate = std::chrono::high_resolution_clock::now();
        m_UpdateTimer = create_wall_timer(20ms, std::bind(&Swerve::Update, this));
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

    rclcpp::Service<pancake::srv::PIDSVA>::SharedPtr Swerve::CreateModuleGainService(
        const std::string& path, MotorConstants<float>* constants) {
        auto callback = std::bind(&Swerve::ModuleGains, this, constants, std::placeholders::_1,
                                  std::placeholders::_2);

        return create_service<pancake::srv::PIDSVA>(path, callback, 10);
    }

    void Swerve::ModuleGains(MotorConstants<float>* constants,
                             std::shared_ptr<pancake::srv::PIDSVA_Request> request,
                             std::shared_ptr<pancake::srv::PIDSVA_Response> response) {
        if (request->set) {
            RCLCPP_INFO(get_logger(), "Setting gains:");

            constants->Feedforward.Sign = request->sva.s;
            constants->Feedforward.Velocity = request->sva.v;
            constants->Feedforward.Acceleration = request->sva.a;

            RCLCPP_INFO(get_logger(), "S: %f", request->sva.s);
            RCLCPP_INFO(get_logger(), "V: %f", request->sva.v);
            RCLCPP_INFO(get_logger(), "A: %f", request->sva.a);

            constants->Feedback.Proportional = request->pid.p;
            constants->Feedback.Integral = request->pid.i;
            constants->Feedback.Derivative = request->pid.d;

            RCLCPP_INFO(get_logger(), "P: %f", request->pid.p);
            RCLCPP_INFO(get_logger(), "I: %f", request->pid.i);
            RCLCPP_INFO(get_logger(), "D: %f", request->pid.d);

            const auto& modules = m_Drivetrain->GetModules();
            const auto& config = m_Drivetrain->GetConfig();

            for (const auto& module : modules) {
                module.Module->Retune(config.Drive.Constants, config.Rotation.Constants);
            }

            response->pid = request->pid;
            response->sva = request->sva;
            response->ack = true;
        } else {
            RCLCPP_INFO(get_logger(), "Returning constants");

            response->sva.s = constants->Feedforward.Sign;
            response->sva.v = constants->Feedforward.Velocity;
            response->sva.a = constants->Feedforward.Acceleration;

            response->pid.p = constants->Feedback.Proportional;
            response->pid.i = constants->Feedback.Integral;
            response->pid.d = constants->Feedback.Derivative;
        }
    }
}; // namespace pancake::swerve