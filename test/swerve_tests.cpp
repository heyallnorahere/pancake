#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <pancake/swerve/drivetrain.h>

#include <numbers>
#include <chrono>
#include <thread>
#include <limits>

using namespace std::chrono_literals;

template <typename _Ty>
static bool epsilon_equal(_Ty lhs, _Ty rhs) {
    return std::abs(lhs - rhs) < std::numeric_limits<_Ty>::epsilon();
}

TEST(pancake, all) {
    /*
    pancake::msg::SwerveRequest request;
    request.absolute = true;
    request.velocity.x = 0.25f;
    request.velocity.y = 0.f;
    request.velocity.angular_velocity = 0.f;

    pancake::swerve::Drivetrain swerve(true);
    swerve.SetRequest(request);

    static constexpr std::chrono::milliseconds interval = 20ms;
    static constexpr std::chrono::seconds totalDuration = 1s;

    std::chrono::duration<float> deltaTime = 0us;
    size_t updateCount = totalDuration / interval;
    for (size_t i = 0; i < updateCount; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        swerve.Update(deltaTime);

        auto t1 = std::chrono::high_resolution_clock::now();
        auto delta = t1 - t0;

        if (interval > delta) {
            deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(interval);
            std::this_thread::sleep_for(interval - delta);
        } else {
            deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(delta);
        }
    }

    const auto& odometry = swerve.GetOdometry();
    auto total = std::chrono::duration_cast<std::chrono::duration<float>>(totalDuration);

    ASSERT_TRUE(epsilon_equal(odometry.transform.x, request.velocity.x * total.count()));
    ASSERT_TRUE(epsilon_equal(odometry.transform.rotation,
                              request.velocity.angular_velocity * total.count()));

    ASSERT_TRUE(epsilon_equal(odometry.velocity.x, request.velocity.x));
    ASSERT_TRUE(
        epsilon_equal(odometry.velocity.angular_velocity, request.velocity.angular_velocity));
    */
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}