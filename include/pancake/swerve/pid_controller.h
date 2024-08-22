#pragma once
#include <vector>
#include <chrono>
#include <optional>
#include <type_traits>

namespace pancake::swerve {
    template <typename T>
    struct PID {
        // Proportional coefficient (in seconds)
        T Proportional;

        // Integral coefficient (proportional)
        T Integral;

        // Derivative (in s^2 units)
        T Derivative;
    };

    template <typename T, class Enabled = void>
    class PIDController {
        // nothing
    };

    template <typename T>
    class PIDController<T, std::enable_if_t<std::is_floating_point_v<T>>> {
    public:
        PIDController(const PID<T>& pid) : m_PID(pid) {}

        inline PID<T>& GetPID() { return m_PID; }
        inline const PID<T>& GetPID() const { return m_PID; }

        inline void SetSetpoint(T setpoint) {
            m_Setpoint = setpoint;
            m_LastSample.reset();
        }

        inline T GetSetpoint() const { return m_Setpoint; }

        inline T Evaluate(T measurement) {
            T delta = (T)0;

            auto timestamp = std::chrono::high_resolution_clock::now();
            if (m_LastSample.has_value()) {
                delta =
                    std::chrono::duration_cast<std::chrono::duration<T>>(timestamp - m_Timestamp)
                        .count();
            }

            T error = m_Setpoint - measurement;
            T integral = error * delta;
            T derivative = (T)0;

            if (m_LastSample.has_value()) {
                T lastError = m_Setpoint - m_LastSample.value();
                derivative = (error - lastError) / delta;
            }

            m_Timestamp = timestamp;
            m_LastSample = measurement;

            return m_PID.Proportional * error + m_PID.Integral * integral +
                   m_PID.Derivative * derivative;
        }

    private:
        PID<T> m_PID;
        T m_Setpoint;

        std::chrono::high_resolution_clock::time_point m_Timestamp;
        std::optional<T> m_LastSample;
    };
}; // namespace pancake::swerve