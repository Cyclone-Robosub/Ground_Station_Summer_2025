#include "imgui.h"
#include "StatusIndicators.hpp"
#include <string>


void RenderExampleIndicator(ImGuiIO& io) {
    ImGui::Begin("Example Indicators");

    // Example status indicators
    ImGui::Text("System Status: %s", io.WantCaptureMouse ? "Active" : "Inactive");
    ImGui::Text("Mouse Position: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    ImGui::Text("Delta Time: %.3f ms", io.DeltaTime * 1000.0f);

    ImGui::End();
}

// Converts a status code to a human-readable string.
std::string StatusCodeToString(int status_code) {
    switch (status_code) {
        case 0: return "success";
        case 1: return "warning";
        case 2: return "error";
        default: return "unknown";
    }
}

void MasterStatusLight(int& master_status) {
    ImGui::Text("Master Status: %s", StatusCodeToString(master_status).c_str());
}

void SystemStatusLight(int& system_status) {
    ImGui::Text("System Status: %s", StatusCodeToString(system_status).c_str());
}

void RenderStatusIndicators() {
    ImGui::Begin("System Status Indicators");

    static int master_status = 1; // Example status code
    static int system_status = 2; // Example status code
    MasterStatusLight(master_status);
    SystemStatusLight(system_status);

    ImGui::End();
}