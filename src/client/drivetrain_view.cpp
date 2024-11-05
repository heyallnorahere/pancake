#include "pancake/client/drivetrain_view.h"

namespace pancake::client {
    DrivetrainView::DrivetrainView(rclcpp::Node* node) : m_Node(node) {
        m_MetaSubscriber = m_Node->create_subscription<pancake::msg::DrivetrainMeta>(
            "/pancake/swerve/meta", 10,
            std::bind(&DrivetrainView::UpdateMeta, this, std::placeholders::_1));
    }

    // todo: more
} // namespace pancake::client