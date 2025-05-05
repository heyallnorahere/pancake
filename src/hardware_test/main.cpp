#include "pancakepch.h"

#include <gtest/gtest.h>
#include <rclcpp/logging.hpp>

#include "pancake/config.h"
#include "pancake/swerve/swerve_module.h"
#include "pancake/swerve/drivetrain.h"

static rclcpp::Logger s_Logger;
static pancake::swerve::Drivetrain::Config s_Config;

static const float s_PI = std::numbers::pi_v<float>;
static constexpr float s_ErrorMargin = 0.1f;
static constexpr float s_UPS = 60.f;
static constexpr std::chrono::duration<float> s_Delta = 1s / s_UPS;

template <typename _Rep, typename _Period>
static void spin_for(
    const std::chrono::duration<_Rep, _Period>& duration,
    const std::function<void(const std::chrono::duration<float>& delta)>& callback) {
    std::chrono::nanoseconds t = 0ns;

    auto t0 = std::chrono::high_resolution_clock::now();
    while (t < duration) {
        auto t1 = std::chrono::high_resolution_clock::now();
        auto delta = t1 - t0;

        t0 = t1;
        t += std::chrono::duration_cast<std::chrono::nanoseconds>(delta);

        callback(std::chrono::duration_cast<std::chrono::duration<float>>(delta));
        if (delta < s_Delta) {
            std::this_thread::sleep_for(s_Delta - delta);
        }
    }
}

TEST(hardware, azimuth) {
    pancake::swerve::Drivetrain drivetrain(s_Config, false);
    pancake::msg::SwerveRequest request{};
    request.absolute = true;

    drivetrain.SetRequest(request);
    drivetrain.Update(1ms);

    const auto& modules = drivetrain.GetModules();
    std::vector<pancake::swerve::ModuleState> initialStates;

    static const float deltaAngle = s_PI / 2.f;
    for (const auto& module : modules) {
        const auto& state = module.Module->GetState();
        initialStates.push_back(state);

        pancake::swerve::ModuleState target;
        target.WheelAngle = state.WheelAngle + deltaAngle;
        target.WheelAngularVelocity = 0.f;
        module.Module->SetTarget(target);
    }

    spin_for(250ms, [&](const std::chrono::duration<float>& delta) {
        for (const auto& module : modules) {
            module.Module->Update();
        }
    });

    for (size_t i = 0; i < modules.size(); i++) {
        const auto& initialState = initialStates[i];
        const auto& state = modules[i].Module->GetState();

        float difference = std::abs(pancake::swerve::SwerveModule::NormalizeAngle(
            state.WheelAngle - initialState.WheelAngle));

        ASSERT_LT(std::abs((difference - deltaAngle) / deltaAngle), s_ErrorMargin);
    }
}

int main(int argc, char** argv) {
    s_Logger = rclcpp::get_logger("test");
    if (!pancake::LoadConfig("swerve", s_Config)) {
        RCLCPP_ERROR(s_Logger, "No swerve config present!");
        return 1;
    }

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}