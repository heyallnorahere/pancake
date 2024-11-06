#include "pancakepch.h"

#include <imgui.h>

#include "pancake/client/drivetrain_view.h"

namespace pancake::client {
    DrivetrainView::DrivetrainView(rclcpp::Node* node)
        : m_Node(node), m_WheelRadius(0.f), m_RecomputeBounds(true) {
        m_MetaSubscriber = m_Node->create_subscription<pancake::msg::DrivetrainMeta>(
            "/pancake/swerve/meta", 10,
            std::bind(&DrivetrainView::UpdateMeta, this, std::placeholders::_1));

        m_OdometrySubscriber = m_Node->create_subscription<pancake::msg::OdometryState>(
            "/pancake/odometry/state", 10,
            [&](const pancake::msg::OdometryState& odometry) { m_Odometry = odometry; });

        IsOpen() = true;
    }

    void DrivetrainView::UpdateView(bool* open) {
        if (m_RecomputeBounds) {
            m_RecomputeBounds = false;

            m_TopRight = pancake::Vector2(0.f);
            m_BottomLeft = pancake::Vector2(0.f);

            for (const auto& module : m_Modules) {
                auto offset = module.Offset;
                if (offset.X > m_TopRight.X) {
                    m_TopRight.X = offset.X;
                }

                if (offset.Y > m_TopRight.Y) {
                    m_TopRight.Y = offset.Y;
                }

                if (offset.X < m_BottomLeft.X) {
                    m_BottomLeft.X = offset.X;
                }

                if (offset.Y < m_BottomLeft.Y) {
                    m_BottomLeft.Y = offset.Y;
                }
            }
        }

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.f, 0.f));
        ImGui::Begin("Drivetrain", open);
        ImGui::PopStyleVar();

        auto viewOrigin = ImGui::GetCursorScreenPos();
        auto viewSize = ImGui::GetContentRegionAvail();

        ImGui::BeginChild("drivetrain-view", viewSize, ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoMove);

        float aspectRatio = viewSize.x / viewSize.y;
        static constexpr float cameraMargin = 2.f; // meters

        float centerToEdge = aspectRatio > 1.f
                                 ? std::max(std::abs(m_TopRight.Y), std::abs(m_BottomLeft.Y))
                                 : std::max(std::abs(m_TopRight.X), std::abs(m_BottomLeft.X));

        float viewScale = std::min(viewSize.x, viewSize.y) / (2.f * (centerToEdge + cameraMargin));
        pancake::Vector2 scale = pancake::Vector2(1.f, -1.f) * viewScale;

        pancake::Vector2 centerOffset = pancake::Vector2(viewSize.x, viewSize.y) / 2.f;
        pancake::Vector2 center = pancake::Vector2(viewOrigin.x, viewOrigin.y) + centerOffset;

        if (aspectRatio > 1.f) {
            scale.X /= aspectRatio;
        } else {
            scale.Y *= aspectRatio;
        }

        float rotation = m_Odometry.transform.rotation;
        auto topRight = center + scale * m_TopRight.Rotate(rotation);
        auto bottomLeft = center + scale * m_BottomLeft.Rotate(rotation);
        auto topLeft =
            center + scale * pancake::Vector2(m_BottomLeft.X, m_TopRight.Y).Rotate(rotation);
        auto bottomRight =
            center + scale * pancake::Vector2(m_TopRight.X, m_BottomLeft.Y).Rotate(rotation);

        auto drawList = ImGui::GetWindowDrawList();
        drawList->AddQuad(topRight, topLeft, bottomLeft, bottomRight, 0xFFFFFFFF);

        for (const auto& module : m_Modules) {
            auto modulePosition = module.Offset;
            float rotationalOffset = module.RotationalOffset;

            float targetSpeed = module.Target.wheel_angular_velocity * m_WheelRadius;
            float targetAngle = module.Target.angle + rotationalOffset;
            pancake::Vector2 targetVector =
                targetSpeed * pancake::Vector2(std::cos(targetAngle), std::sin(targetAngle));

            float actualSpeed = module.State.wheel_angular_velocity * m_WheelRadius;
            float actualAngle = module.State.angle + rotationalOffset;
            pancake::Vector2 actualVector =
                actualSpeed * pancake::Vector2(std::cos(actualAngle), std::sin(actualAngle));

            auto p0 = center + scale * modulePosition.Rotate(rotation);
            auto p1 = center + scale * (modulePosition + targetVector).Rotate(rotation);
            auto p2 = center + scale * (modulePosition + actualVector).Rotate(rotation);

            drawList->AddCircle(p0, m_WheelRadius * viewScale, 0xFFFFFFFF);

            static constexpr uint32_t targetColor = 0xFF0000FF;
            float vectorEndRadius = 0.025 * viewScale;
            drawList->AddLine(p0, p1, targetColor);

            if (targetVector.Length2() >= std::numeric_limits<float>::epsilon()) {
                drawList->AddCircleFilled(p1, vectorEndRadius, targetColor);
            }

            static constexpr uint32_t actualColor = 0x0000FFFF;
            drawList->AddLine(p0, p2, actualColor);

            if (actualVector.Length2() >= std::numeric_limits<float>::epsilon()) {
                drawList->AddCircleFilled(p2, vectorEndRadius, actualColor);
            }
        }

        ImGui::EndChild();
        ImGui::End();
    }

    void DrivetrainView::UpdateMeta(const pancake::msg::DrivetrainMeta& meta) {
        m_WheelRadius = meta.wheel_radius;

        size_t originalCount = m_Modules.size();
        m_Modules.resize((size_t)meta.module_count);

        for (size_t i = originalCount; i < m_Modules.size(); i++) {
            auto modulePath = "/pancake/swerve/module/mod" + std::to_string(i);

            auto& module = m_Modules[i];
            module.MetaSubscriber = m_Node->create_subscription<pancake::msg::RobotTransform>(
                modulePath + "/meta", 10, [&](const pancake::msg::RobotTransform& meta) {
                    m_RecomputeBounds |=
                        std::abs(module.Offset.X - meta.x) >=
                            std::numeric_limits<float>::epsilon() ||
                        std::abs(module.Offset.Y - meta.y) >= std::numeric_limits<float>::epsilon();

                    module.Offset.X = meta.x;
                    module.Offset.Y = meta.y;
                    module.RotationalOffset = meta.rotation;
                });

            module.TargetSubscriber = m_Node->create_subscription<pancake::msg::ModuleState>(
                modulePath + "/target", 10,
                [&](const pancake::msg::ModuleState& target) { module.Target = target; });

            module.StateSubscriber = m_Node->create_subscription<pancake::msg::ModuleState>(
                modulePath + "/state", 10,
                [&](const pancake::msg::ModuleState& state) { module.State = state; });
        }
    }
} // namespace pancake::client