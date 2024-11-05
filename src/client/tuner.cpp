#include "pancakepch.h"
#include "pancake/client/tuner.h"

#include <imgui.h>

namespace pancake::client {
    void Tuner::UpdateView(bool* open) {
        ImGui::Begin("Tuner", open);

        try {
            auto services = m_Node->get_service_names_and_types();
            for (const auto& [path, types] : services) {
                bool isTunable = false;
                for (const auto& type : types) {
                    if (type == "pancake/srv/PIDSVA") {
                        isTunable = true;
                        break;
                    }
                }

                if (isTunable) {
                    if (m_ServiceList.contains(path) || m_ServiceBlocklist.contains(path)) {
                        continue;
                    }

                    auto index = InstantiateClient(path);
                    if (index.has_value()) {
                        m_ServiceList.insert(path);
                    } else {
                        m_ServiceBlocklist.insert(path);
                    }
                }
            }
        } catch (const std::runtime_error& exc) {
            RCLCPP_ERROR(rclcpp::get_logger("client"), "Caught error: %s", exc.what());
            ImGui::End();
            return;
        }

        std::vector<size_t> closedClients;
        for (size_t i = 0; i < m_Clients.size(); i++) {
            if (!m_Clients[i].Client->service_is_ready()) {
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

            ImGui::Text("Feedforward/SVA");
            update |= ImGui::InputFloat("S", &selectedClient.SVA.s);
            update |= ImGui::InputFloat("V", &selectedClient.SVA.v);
            update |= ImGui::InputFloat("A", &selectedClient.SVA.a);

            if (update) {
                UpdateGains(m_ClientIndex);
            }
        }

        ImGui::End();
    }

    std::optional<size_t> Tuner::InstantiateClient(const std::string& path) {
        ClientData data;
        data.Path = path;
        data.Client = m_Node->create_client<pancake::srv::PIDSVA>(path, 10);

        size_t index = m_Clients.size();
        m_Clients.push_back(data);

        std::thread futureThread([this, index]() mutable {
            auto request = std::make_shared<pancake::srv::PIDSVA_Request>();
            request->set = false;

            auto& client = m_Clients[index];
            auto future = client.Client->async_send_request(request);

            future.wait();
            auto response = future.get();

            client.PID = response->pid;
            client.SVA = response->sva;
        });

        futureThread.detach();
        return index;
    }

    void Tuner::UpdateGains(size_t index) {
        const auto& client = m_Clients[index];

        auto request = std::make_shared<pancake::srv::PIDSVA_Request>();
        request->set = true;
        request->pid = client.PID;
        request->sva = client.SVA;

        client.Client->async_send_request(request);
    }
}; // namespace pancake::client