#include "imgui.h"
#include "StatusIndicators.hpp"
#include <string>
#include <vector>


void RenderExampleIndicator(ImGuiIO& io) {
    ImGui::Begin("Example Indicators");
    ImGui::Text("System Status: %s", io.WantCaptureMouse ? "Active" : "Inactive");
    ImGui::Text("Mouse Position: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    ImGui::Text("Delta Time: %.3f ms", io.DeltaTime * 1000.0f);
    ImGui::End();
}


void MasterStatusLight(const std::string& master_status) {
    ImGui::Text("Master Status: %s", master_status.c_str());
}

// TODO: Implement the rendering logic for system status indicators
void SystemStatusLight(const SystemStatus& system_status) {
    ImGui::Text("Status: %s", system_status.status.c_str());
    ImGui::Text("Message: %s", system_status.message.c_str());
}

void RenderStatusIndicators(const std::string& master_status, const std::vector<SystemStatus>& system_statuses) {
    ImGui::Begin("System Status Indicators");

    MasterStatusLight(master_status);

    for (size_t i = 0; i < system_statuses.size(); ++i) {
        ImGui::Separator();
        ImGui::Text("System %zu:", i + 1);
        SystemStatusLight(system_statuses[i]);
    }

    ImGui::End();
}