#pragma once
#include "pancake/client/view.h"

#include "pancake/vector2.h"

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

        bool m_RecomputeBounds;
        pancake::Vector2 m_TopRight, m_BottomLeft;

        pancake::msg::OdometryState m_Odometry;
        std::vector<ModuleInfo> m_Modules;

        rclcpp::Subscription<pancake::msg::DrivetrainMeta>::SharedPtr m_MetaSubscriber;
        rclcpp::Subscription<pancake::msg::OdometryState>::SharedPtr m_OdometrySubscriber;
    };
} // namespace pancake::client