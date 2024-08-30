#pragma once

namespace pancake::swerve {
    template <typename _Ty>
    struct SVA {
        _Ty Sign, Velocity, Acceleration;
    };

    template <typename _Ty>
    class Feedforward {
    public:
        Feedforward(const SVA<_Ty>& sva) : m_SVA(sva) {}
        ~Feedforward() = default;

        Feedforward(const Feedforward&) = delete;
        Feedforward& operator=(const Feedforward&) = delete;

        inline SVA<_Ty>& GetSVA() { return m_SVA; }
        inline const SVA<_Ty>& GetSVA() const { return m_SVA; }

        inline _Ty Evaluate(_Ty velocity, _Ty acceleration = (_Ty)0) {
            _Ty sign = velocity > (_Ty)0 ? (_Ty)1 : (_Ty)-1;
            return m_SVA.Sign * sign + m_SVA.Velocity * velocity +
                   m_SVA.Acceleration * acceleration;
        }

    private:
        SVA<_Ty> m_SVA;
    };
} // namespace pancake::swerve