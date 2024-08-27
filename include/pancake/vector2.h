#pragma once
#include <cmath>

namespace pancake {
    struct Vector2 {
        Vector2() : X(0.f), Y(0.f) {}
        Vector2(float scalar) : X(scalar), Y(scalar) {}
        Vector2(float x, float y) : X(x), Y(y) {}

        float X, Y;

        inline float Length2() const { return X * X + Y * Y; }
        inline float Length() const { return std::sqrt(Length2()); }

        inline float Dot(const Vector2& other) const { return X * other.X + Y * other.Y; }

        Vector2 Normalize() const;

        inline Vector2 Rotate(float radians) const {
            float cos = std::cos(radians);
            float sin = std::sin(radians);

            Vector2 result;
            result.X = X * cos - Y * sin;
            result.Y = X * sin + Y * cos;

            return result;
        }
    };

    inline Vector2 operator+(const Vector2& lhs, const Vector2& rhs) {
        Vector2 result;
        result.X = lhs.X + rhs.X;
        result.Y = lhs.Y + rhs.Y;
        return result;
    }

    inline Vector2 operator*(const Vector2& lhs, float rhs) {
        Vector2 result;
        result.X = lhs.X * rhs;
        result.Y = lhs.Y * rhs;
        return result;
    }

    inline Vector2 operator*(float lhs, const Vector2& rhs) { return rhs * lhs; }
    inline Vector2 operator/(const Vector2& lhs, float rhs) { return lhs * (1.f / rhs); }

    inline Vector2 Vector2::Normalize() const { return *this / Length(); }
} // namespace pancake