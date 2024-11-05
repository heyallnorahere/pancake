#include "pancake/client/drivetrain_view.h"

#include <imgui.h>

namespace pancake::client {
    DrivetrainView::DrivetrainView(rclcpp::Node* node) : m_Node(node), m_WheelRadius(0.f) {
        m_MetaSubscriber = m_Node->create_subscription<pancake::msg::DrivetrainMeta>(
            "/pancake/swerve/meta", 10,
            std::bind(&DrivetrainView::UpdateMeta, this, std::placeholders::_1));

        IsOpen() = true;
    }

    void DrivetrainView::UpdateView(bool* open) {
        ImGui::Begin("Drivetrain");

        // todo: render

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
                modulePath + "/meta", 10, [&module](const pancake::msg::RobotTransform& meta) {
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