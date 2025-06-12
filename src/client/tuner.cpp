#include "pancakepch.h"
#include "pancake/client/tuner.h"

#include <regex>

#include <imgui.h>

namespace pancake::client {
    Tuner::Tuner(rclcpp::Node* node) {
        m_Node = node;
        m_ClientIndex = 0;

        m_SaveConfig =
            m_Node->create_publisher<std_msgs::msg::Bool>("/pancake/config/save/swerve", 10);

        IsOpen() = true;
    }

    void Tuner::UpdateView(bool* open) {
        ImGui::Begin("Tuner", open);

        try {
            static const std::unordered_set<std::string> acceptedTypes = { "pancake/msg/PID",
                                                                           "pancake/msg/SVA" };

            auto topics = m_Node->get_topic_names_and_types();
            for (const auto& [topicPath, types] : topics) {
                bool isTunable = false;
                for (const auto& type : types) {
                    if (acceptedTypes.contains(type)) {
                        isTunable = true;
                        break;
                    }
                }

                if (isTunable) {
                    static const std::regex nameRegex("(.*)/(pid|sva)/[gs]et");

                    std::smatch match;
                    if (!std::regex_match(topicPath, match, nameRegex)) {
                        continue;
                    }

                    auto servicePath = match[1].str();
                    if (m_ServiceList.contains(servicePath) ||
                        m_ServiceBlocklist.contains(servicePath)) {
                        continue;
                    }

                    InstantiateClient(servicePath);
                    m_ServiceList.insert(servicePath);
                }
            }
        } catch (const std::runtime_error& exc) {
            RCLCPP_ERROR(rclcpp::get_logger("client"), "Caught error: %s", exc.what());
            ImGui::End();
            return;
        }

        std::vector<size_t> closedClients;
        for (size_t i = 0; i < m_Clients.size(); i++) {
            const auto& pidListenTopic = m_Clients[i].PIDSubscriber->get_topic_name();
            size_t publisherCount = m_Node->count_publishers(pidListenTopic);

            if (publisherCount == 0) {
                closedClients.push_back(i);
            }
        }

        for (size_t i = 0; i < closedClients.size(); i++) {
            size_t index = closedClients[i] - i;
            m_Clients.erase(m_Clients.begin() + index);

            if (m_ClientIndex >= index && m_ClientIndex > 0) {
                m_ClientIndex--;
            }
        }

        if (m_Clients.size() == 0) {
            ImGui::Text("No clients have been connected.");
        } else {
            if (ImGui::BeginCombo("Motor/module", m_Clients[m_ClientIndex].Path.c_str())) {
                for (size_t i = 0; i < m_Clients.size(); i++) {
                    const auto& client = m_Clients[i];

                    bool selected = i == m_ClientIndex;
                    if (ImGui::Selectable(client.Path.c_str(), &selected)) {
                        m_ClientIndex = i;
                    }

                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }

            auto& selectedClient = m_Clients[m_ClientIndex];
            bool update = false;

            ImGui::Text("Feedback/PID");
            update |= ImGui::InputFloat("P", &selectedClient.PID.p);
            update |= ImGui::InputFloat("I", &selectedClient.PID.i);
            update |= ImGui::InputFloat("D", &selectedClient.PID.d);

            if (update) {
                selectedClient.PIDPublisher->publish(selectedClient.PID);
            }

            update = false;
            ImGui::Text("Feedforward/SVA");
            update |= ImGui::InputFloat("S", &selectedClient.SVA.s);
            update |= ImGui::InputFloat("V", &selectedClient.SVA.v);
            update |= ImGui::InputFloat("A", &selectedClient.SVA.a);

            if (update) {
                selectedClient.SVAPublisher->publish(selectedClient.SVA);
            }

            if (ImGui::Button("Save config")) {
                std_msgs::msg::Bool save;
                save.data = true;

                m_SaveConfig->publish(save);
            }
        }

        ImGui::End();
    }

    size_t Tuner::InstantiateClient(const std::string& path) {
        size_t index = m_Clients.size();

        ClientData data;
        data.Path = path;

        data.PIDPublisher = m_Node->create_publisher<pancake::msg::PID>(path + "/pid/set", 10);
        data.PIDSubscriber = m_Node->create_subscription<pancake::msg::PID>(
            path + "/pid/get", 10, [this, index](const pancake::msg::PID& pid) {
                auto& client = m_Clients[index];
                client.PID = pid;
            });

        data.SVAPublisher = m_Node->create_publisher<pancake::msg::SVA>(path + "/sva/set", 10);
        data.SVASubscriber = m_Node->create_subscription<pancake::msg::SVA>(
            path + "/sva/get", 10, [this, index](const pancake::msg::SVA& sva) {
                auto& client = m_Clients[index];
                client.SVA = sva;
            });

        m_Clients.push_back(data);

        return index;
    }
}; // namespace pancake::client