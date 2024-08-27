#include "pancake/swerve/swerve.h"

#include <chrono>

using namespace std::chrono_literals;

namespace pancake::swerve {
    Swerve::Swerve() : Node("swerve") {
        m_Drivetrain = std::make_shared<Drivetrain>(false);

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