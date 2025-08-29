#pragma once

#include "pancake/swerve/drivetrain.h"

namespace pancake::swerve {
    struct ModuleTelemetry {
        rclcpp::Publisher<pancake::msg::RobotTransform>::SharedPtr Meta;
        rclcpp::Publisher<pancake::msg::ModuleState>::SharedPtr Target, State;
    };

    struct TuningService {
        MotorConstants<float>* Constants;

        rclcpp::Publisher<pancake::msg::PID>::SharedPtr PIDPublisher;
        rclcpp::Subscription<pancake::msg::PID>::SharedPtr PIDSubscriber;

        rclcpp::Publisher<pancake::msg::SVA>::SharedPtr SVAPublisher;
        rclcpp::Subscription<pancake::msg::SVA>::SharedPtr SVASubscriber;
    };

    /*
     * Swerve ROS node. Performs all drivetrain operations.
     */
    class Swerve : public rclcpp::Node {
    public:
        Swerve(bool sim = false);
        ~Swerve() = default;

        Swerve(const Swerve&) = delete;
        Swerve& operator=(const Swerve&) = delete;

    private:
        /*
         * Periodic update. Runs every 20ms
         */
        void Update();

        /*
         * Create interface to tune PID and SVA values on the fly.
         */
        std::unique_ptr<TuningService> CreateTuningService(const std::string& path,
                                                           MotorConstants<float>* constants);

        // callbacks for network tuning
        void SetPID(TuningService* service, const pancake::msg::PID& pid);
        void SetSVA(TuningService* service, const pancake::msg::SVA& sva);

        /*
         * Pass new tuning values to swerve modules.
         */
        void RetuneModules();

        std::shared_ptr<Drivetrain> m_Drivetrain;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_KillListener, m_ReloadListener;

        rclcpp::Subscription<pancake::msg::SwerveRequest>::SharedPtr m_RequestSubscriber;
        rclcpp::Subscription<pancake::msg::OdometryState>::SharedPtr m_ResetSubscriber;
        rclcpp::Publisher<pancake::msg::OdometryState>::SharedPtr m_OdometryPublisher;
        rclcpp::Publisher<pancake::msg::DrivetrainMeta>::SharedPtr m_MetaPublisher;

        std::unique_ptr<TuningService> m_DriveTuning, m_RotationTuning;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_SaveConfig;

        rclcpp::TimerBase::SharedPtr m_UpdateTimer;
        std::optional<std::chrono::high_resolution_clock::time_point> m_LastUpdate;

        std::vector<ModuleTelemetry> m_ModuleTelemetry;
    };
} // namespace pancake::swerve
