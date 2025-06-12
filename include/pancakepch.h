#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include "pancake/msg/input.hpp"
#include "pancake/msg/module_state.hpp"
#include "pancake/msg/odometry_state.hpp"
#include "pancake/msg/robot_transform.hpp"
#include "pancake/msg/drivetrain_meta.hpp"
#include "pancake/msg/swerve_request.hpp"

#include "pancake/msg/pid.hpp"
#include "pancake/msg/sva.hpp"

#include <string>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <optional>
#include <limits>
#include <type_traits>
#include <utility>
#include <numbers>
#include <algorithm>
#include <memory>
#include <tuple>
#include <stdexcept>
#include <functional>

using namespace std::chrono_literals;