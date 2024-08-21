#pragma once
#include <vector>
#include <chrono>
#include <optional>

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

    template <typename T>
    class PIDController {
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
            float delta = 0.f;

            auto timestamp = std::chrono::high_resolution_clock::now();
            if (m_LastSample.has_value()) {
                delta = std::chrono::duration_cast<std::chrono::duration<float>>(timestamp -
                                                                                 m_Timestamp)
                            .count();
            }

            float error = m_Setpoint - measurement;
            float integral = error * delta;
            float derivative = (measurement - m_LastSample.value_or(0.f)) / delta;

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