#include "imgui.h"
#include "StatusIndicators.hpp"
#include <string>


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

void SystemStatusLight(const std::string& system_status) {
    ImGui::Text("System Status: %s", system_status.c_str());
}

void RenderStatusIndicators() {
    ImGui::Begin("System Status Indicators");

    static std::string master_status = "warning"; // Example status string
    static std::string system_status = "error";   // Example status string
    MasterStatusLight(master_status);
    SystemStatusLight(system_status);

    ImGui::End();
}