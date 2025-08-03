#include "imgui.h"
#include "StatusIndicators.hpp"

void RenderExampleIndicator(ImGuiIO& io) {
    ImGui::Begin("Status Indicators");

    // Example status indicators
    ImGui::Text("System Status: %s", io.WantCaptureMouse ? "Active" : "Inactive");
    ImGui::Text("Mouse Position: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    ImGui::Text("Delta Time: %.3f ms", io.DeltaTime * 1000.0f);

    ImGui::End();
}

// void RenderMasterStatusLight()