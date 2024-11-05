#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <pancakepch.h>
#include <pancake/vector2.h>
#include <pancake/swerve/drivetrain.h>
#include <pancake/swerve/pid_controller.h>

#include <numbers>
#include <chrono>
#include <thread>
#include <limits>

#include <cmath>

using namespace std::chrono_literals;

TEST(pancake, vector_math) {
    pancake::Vector2 a;
    a.X = 7.f;
    a.Y = 42.f;

    pancake::Vector2 aPrime1 = a.Rotate(std::numbers::pi_v<float> / 2.f);
    pancake::Vector2 aPrime2 = a.Rotate(std::numbers::pi_v<float> / 4.f);

    ASSERT_FLOAT_EQ(aPrime1.X, -a.Y);
    ASSERT_FLOAT_EQ(aPrime1.Y, a.X);

    float sqrt2Over2 = std::sqrt(2.f) / 2.f;
    ASSERT_FLOAT_EQ(aPrime2.X, sqrt2Over2 * (a.X - a.Y));
    ASSERT_FLOAT_EQ(aPrime2.Y, sqrt2Over2 * (a.X + a.Y));
}

TEST(pancake, module_test) {
    pancake::msg::SwerveRequest request;
    request.absolute = true;
    request.velocity.x = 0.25f;
    request.velocity.y = 0.f;
    request.velocity.angular_velocity = 0.f;

    pancake::Vector2 requestedLinear;
    requestedLinear.X = request.velocity.x;
    requestedLinear.Y = request.velocity.y;
    float requestedAngular = request.velocity.angular_velocity;

    pancake::swerve::Drivetrain::Config config;
    config.Modules.push_back({ 1, 2, { 1.f, 1.f }, std::numbers::pi_v<float> / 4.f });

    pancake::swerve::Drivetrain swerve({}, true);
    swerve.SetRequest(request);
    swerve.Update(std::chrono::duration_cast<std::chrono::duration<float>>(1ms));

    float wheelRadius = swerve.GetWheelRadius();
    const auto& modules = swerve.GetModules();

    for (const auto& meta : modules) {
        float moduleRotation = std::atan2(meta.CenterOffset.Y, meta.CenterOffset.X);
        if (request.absolute) {
            moduleRotation += swerve.GetOdometry().transform.rotation;
        }

        pancake::Vector2 linear = requestedLinear.Rotate(-moduleRotation);
        pancake::Vector2 angular = { 0.f, requestedAngular * meta.CenterOffset.Length() };
        pancake::Vector2 relativeVelocity = linear + angular;

        float velocityAngle = std::atan2(relativeVelocity.Y, relativeVelocity.X);
        float wheelVelocity = relativeVelocity.Length() / wheelRadius;

        const auto& target = meta.Module->GetTarget();
        ASSERT_FLOAT_EQ(target.WheelAngularVelocity, wheelVelocity);
        ASSERT_FLOAT_EQ(target.WheelAngle, velocityAngle);
    }
}

template <typename _Ty>
static _Ty signum(_Ty value) {
    return value > (_Ty)0 ? (_Ty)1 : (_Ty)-1;
}

TEST(pancake, pid) {
    std::vector<float> measurements = { 10.f, 30.f };
    pancake::swerve::PID<float> pid;
    pid.Proportional = 42.f;
    pid.Integral = 7.f;
    pid.Derivative = 20.f;

    float integralBound = std::numeric_limits<float>::infinity();
    pancake::swerve::PIDController<float> controller(pid);
    controller.SetSetpoint(40.f);
    controller.SetIntegralBound({ -integralBound, integralBound });

    float result = 0.f;
    float error = 0.f;
    float integral = 0.f;
    float derivative = 0.f;
    auto lastTimestamp = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < measurements.size(); i++) {
        float setpoint = controller.GetSetpoint();
        float sample = measurements[i];

        float lastError = error;
        error = setpoint - sample;

        auto now = std::chrono::high_resolution_clock::now();
        result = controller.Evaluate(sample, pancake::swerve::IntegrationType::RightRiemann);

        auto delta = std::chrono::duration_cast<std::chrono::duration<float>>(now - lastTimestamp);
        lastTimestamp = now;

        if (i > 0) {
            integral += error * delta.count();
            integral =
                std::clamp(integral, -integralBound / pid.Integral, integralBound / pid.Integral);

            derivative = (error - lastError) / delta.count();
        }
    }

    float ex = pid.Proportional * error + pid.Integral * integral + pid.Derivative * derivative;
    ASSERT_GT(result * signum(ex), std::abs(ex * 2.f / 3.f));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}