#pragma once

namespace pancake::swerve {
    template <typename _Ty>
    struct PID {
        // Proportional coefficient (in seconds)
        _Ty Proportional;

        // Integral coefficient (proportional)
        _Ty Integral;

        // Derivative coefficient (in s^2 units)
        _Ty Derivative;
    };

    enum class IntegrationType { RightRiemann, LeftRiemann };

    /*
     * Controller object that adjusts voltage fed to the device based on the error between the
     * desired setpoint and the actual output.
     * 
     * Used with motor controllers primarily, however it can be used to tune any number of controls.
     */
    template <typename _Ty>
    class PIDController {
    public:
        PIDController(const PID<_Ty>& pid)
            : m_PID(pid), m_Setpoint((_Ty)0), m_IntegralBound({ (_Ty)-1, (_Ty)1 }),
              m_MaxSamples(10) {
            static_assert(std::is_floating_point_v<_Ty>,
                          "Template type is not a floating-point number!");
        }

        ~PIDController() = default;

        PIDController(const PIDController<_Ty>& other) = delete;
        PIDController& operator=(const PIDController<_Ty>& other) = delete;

        inline PID<_Ty>& GetPID() { return m_PID; }
        inline const PID<_Ty>& GetPID() const { return m_PID; }

        inline void SetMaxSamples(size_t maxSamples) {
            m_MaxSamples = maxSamples;
            ValidateSampleCount();
        }

        /*
         * Sets the limits of the integral term in Evaluate(_Ty, IntegrationType)
         */
        inline void SetIntegralBound(const std::tuple<_Ty, _Ty>& bound) {
            _Ty min = std::get<0>(bound);
            _Ty max = std::get<1>(bound);

            if (min > max) {
                throw std::out_of_range("Integral bound is not valid (min is greater than max)");
            }

            m_IntegralBound = bound;
        }

        /*
         * Sets the callback to bound the error on evaluation.
         * This is used for applications such as angle, when there is a clear -2pi to 2pi limit.
         */
        inline void SetErrorBound(const std::function<_Ty(_Ty)>& bound) { m_ErrorBound = bound; }

        /*
         * Sets the goal value of the PID controller.
         */
        inline void SetSetpoint(_Ty setpoint) { m_Setpoint = setpoint; }

        /*
         * Returns the goal value of the PID controller.
         */
        inline _Ty GetSetpoint() const { return m_Setpoint; }

        /*
         * Evaluates the PID controller.
         * Provide the instantaneous measurement of the output.
         * Returns the value to send to the device.
         */
        inline _Ty Evaluate(_Ty measurement,
                            IntegrationType integration = IntegrationType::RightRiemann) {
            // add the 
            Sample sample;
            sample.Value = measurement;
            sample.Timestamp = std::chrono::high_resolution_clock::now();

            size_t intervalCount = m_Samples.size();
            m_Samples.push_back(sample);

            _Ty error = m_Setpoint - measurement;
            if (m_ErrorBound.has_value()) {
                error = m_ErrorBound.value()(error);
            }

            _Ty integral = (_Ty)0;
            _Ty derivative = (_Ty)0;

            for (size_t i = 0; i < intervalCount; i++) {
                auto it = m_Samples.begin();
                std::advance(it, i);
                const auto& lastSample = *it;

                it++;
                const auto& currentSample = *it;

                _Ty lastError = m_Setpoint - lastSample.Value;
                _Ty currentError = m_Setpoint - currentSample.Value;

                if (m_ErrorBound.has_value()) {
                    const auto& errorBound = m_ErrorBound.value();

                    lastError = errorBound(lastError);
                    currentError = errorBound(currentError);
                }

                auto delta = std::chrono::duration_cast<std::chrono::duration<_Ty>>(
                    currentSample.Timestamp - lastSample.Timestamp);

                switch (integration) {
                case IntegrationType::RightRiemann:
                    integral += currentError * delta.count();
                    break;
                case IntegrationType::LeftRiemann:
                    integral += lastError * delta.count();
                    break;
                default:
                    throw std::runtime_error("Invalid integration type!");
                }

                _Ty min = std::get<0>(m_IntegralBound) / m_PID.Integral;
                _Ty max = std::get<1>(m_IntegralBound) / m_PID.Integral;
                integral = std::clamp(integral, min, max);

                if (i == intervalCount - 1) {
                    derivative += (currentError - lastError) / delta.count();
                }
            }

            ValidateSampleCount();
            return m_PID.Proportional * error + m_PID.Integral * integral +
                   m_PID.Derivative * derivative;
        }

    private:
        // clamps m_Samples to m_MaxSamples elements. removes elements from the front if necessary.
        inline void ValidateSampleCount() {
            if (m_Samples.size() <= m_MaxSamples) {
                return;
            }

            auto begin = m_Samples.begin();
            auto end = begin;
            std::advance(end, m_Samples.size() - m_MaxSamples);

            m_Samples.erase(begin, end);
        }

        struct Sample {
            _Ty Value;
            std::chrono::high_resolution_clock::time_point Timestamp;
        };

        PID<_Ty> m_PID;
        _Ty m_Setpoint;
        std::tuple<_Ty, _Ty> m_IntegralBound;

        // we want an iterable queue of samples. constant time insertion and removal :3c
        std::list<Sample> m_Samples;
        size_t m_MaxSamples;

        std::optional<std::function<_Ty(_Ty)>> m_ErrorBound;
    };
}; // namespace pancake::swerve