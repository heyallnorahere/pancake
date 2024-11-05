#pragma once
#include "pancake/client/view.h"

#include "pancake/vector2.h"
#include "pancake/msg/module_state.hpp"
#include "pancake/msg/odometry_state.hpp"
#include "pancake/msg/robot_transform.hpp"
#include "pancake/msg/drivetrain_meta.hpp"

#include <rclcpp/node.hpp>

namespace pancake::client {
    struct ModuleInfo {
        pancake::Vector2 Offset;
        float RotationalOffset;
        rclcpp::Subscription<pancake::msg::RobotTransform>::SharedPtr MetaSubscriber;

        pancake::msg::ModuleState Target, State;
        rclcpp::Subscription<pancake::msg::ModuleState>::SharedPtr TargetSubscriber,
            StateSubscriber;
    };

    class DrivetrainView : public View {
    public:
        DrivetrainView(rclcpp::Node* node);
        virtual ~DrivetrainView() override = default;

        virtual std::string GetID() const override { return "drivetrain_view"; }

    private:
        virtual void UpdateView(bool* open) override;

        void UpdateMeta(const pancake::msg::DrivetrainMeta& meta);

        rclcpp::Node* m_Node;
        float m_WheelRadius;

        std::vector<ModuleInfo> m_Modules;
        rclcpp::Subscription<pancake::msg::DrivetrainMeta>::SharedPtr m_MetaSubscriber;
    };
} // namespace pancake::client