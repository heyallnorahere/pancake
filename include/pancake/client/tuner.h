#pragma once
#include "pancake/client/view.h"

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>

#include <optional>

namespace pancake::client {
    struct ClientData {
        std::string Path;

        pancake::msg::PID PID;
        pancake::msg::SVA SVA;

        rclcpp::Publisher<pancake::msg::PID>::SharedPtr PIDPublisher;
        rclcpp::Subscription<pancake::msg::PID>::SharedPtr PIDSubscriber;

        rclcpp::Publisher<pancake::msg::SVA>::SharedPtr SVAPublisher;
        rclcpp::Subscription<pancake::msg::SVA>::SharedPtr SVASubscriber;
    };

    class Tuner : public View {
    public:
        Tuner(rclcpp::Node* node);
        virtual ~Tuner() = default;

        virtual std::string GetID() const override { return "tuner"; }

    private:
        virtual void UpdateView(bool* open) override;

        size_t InstantiateClient(const std::string& path);

        rclcpp::Node* m_Node;
        size_t m_ClientIndex;
        std::vector<ClientData> m_Clients;
        std::unordered_set<std::string> m_ServiceBlocklist, m_ServiceList;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_SaveConfig;
    };
}; // namespace pancake::client