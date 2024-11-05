#pragma once
#include "pancake/client/view.h"


#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>

#include <optional>

namespace pancake::client {
    struct ClientData {
        std::string Path;
        rclcpp::Client<pancake::srv::PIDSVA>::SharedPtr Client;

        pancake::msg::PID PID;
        pancake::msg::SVA SVA;
    };

    class Tuner : public View {
    public:
        Tuner(rclcpp::Node* node) : m_Node(node), m_ClientIndex(0) {}
        virtual ~Tuner() = default;

        virtual std::string GetID() const override { return "tuner"; }

    private:
        virtual void UpdateView(bool* open) override;

        std::optional<size_t> InstantiateClient(const std::string& path);
        void UpdateGains(size_t index);

        rclcpp::Node* m_Node;
        size_t m_ClientIndex;
        std::vector<ClientData> m_Clients;
        std::unordered_set<std::string> m_ServiceBlocklist, m_ServiceList;
    };
}; // namespace pancake::client