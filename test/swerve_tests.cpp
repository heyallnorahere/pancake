#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <pancakepch.h>
#include <pancake/vector2.h>
#include <pancake/swerve/drivetrain.h>
#include <pancake/swerve/pid_controller.h>
#include <pancake/swerve/swerve.h>

#include <numbers>
#include <chrono>
#include <thread>
#include <limits>
#include <future>

#include <cmath>

using namespace std::chrono_literals;

TEST(swerve, vector_math) {
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

TEST(swerve, module_targeting) {
    pancake::msg::SwerveRequest request;
    request.absolute = true;
    request.velocity.x = 0.25f;
    request.velocity.y = 0.f;
    request.velocity.angular_velocity = std::numbers::pi_v<float> / 2.f;

    pancake::Vector2 requestedLinear;
    requestedLinear.X = request.velocity.x;
    requestedLinear.Y = request.velocity.y;
    float requestedAngular = request.velocity.angular_velocity;

    pancake::swerve::Drivetrain::Config config;
    config.Modules.push_back({ 1, 2, { 1.f, 1.f }, std::numbers::pi_v<float> / 4.f });

    pancake::swerve::Drivetrain swerve({}, true);
    swerve.SetRequest(request);
    swerve.Update(std::chrono::duration_cast<std::chrono::duration<float>>(1ms));

    float wheelRadius = swerve.GetConfig().WheelRadius;
    const auto& modules = swerve.GetModules();

    for (const auto& meta : modules) {
        float moduleRotation = meta.RotationalOffset;
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

TEST(swerve, pid) {
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

TEST(swerve, multi_module) {
    auto t0 = std::chrono::high_resolution_clock::now();

    pancake::swerve::Drivetrain::Config config;
    config.Drive.GearRatio = 1.f;
    config.Rotation.GearRatio = 1.f;
    config.WheelRadius = 1.f;
    config.Network = "can0";
    config.EncoderConfig.Mode = pancake::swerve::RotationEncoderMode::Output;
    config.EncoderConfig.GearRatio = 1.f;

    for (size_t i = 0; i < 4; i++) {
        float angle = std::numbers::pi_v<float> * (1.f / 4.f + i / 2.f);

        pancake::swerve::SwerveModuleDesc desc;
        desc.CenterOffset = { std::cos(angle), std::sin(angle) };
        desc.RotationalOffset = angle;
        desc.Drive = 2 + i * 2;
        desc.Rotation = 1 + i * 2;

        config.Modules.push_back(desc);
    }

    pancake::msg::SwerveRequest request{};
    request.absolute = true;
    request.velocity.x = 1.f;

    pancake::swerve::Drivetrain drivetrain(config, true);
    drivetrain.SetRequest(request);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto delta = std::chrono::duration_cast<std::chrono::duration<float>>(t1 - t0);
    drivetrain.Update(delta);

    const auto& modules = drivetrain.GetModules();
    for (const auto& module : modules) {
        const auto& target = module.Module->GetTarget();

        float targetAngle = target.WheelAngle;
        if (target.WheelAngularVelocity < 0.f) {
            targetAngle += std::numbers::pi_v<float>; // see SwerveModule::SetTarget
        }

        ASSERT_FLOAT_EQ(pancake::swerve::SwerveModule::NormalizeAngle(targetAngle),
                        pancake::swerve::SwerveModule::NormalizeAngle(-module.RotationalOffset));

        ASSERT_FLOAT_EQ(
            std::abs(target.WheelAngularVelocity),
            std::sqrt(std::pow(request.velocity.x, 2.f) + std::pow(request.velocity.y, 2.f)) /
                config.WheelRadius);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    return RUN_ALL_TESTS();
}