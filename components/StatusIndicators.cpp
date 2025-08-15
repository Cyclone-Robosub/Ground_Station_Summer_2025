#include "imgui.h"
#include <string>
#include <vector>
#include <mutex>
#include "StatusIndicators.hpp"

void RenderExampleIndicator(ImGuiIO &io)
{
    ImGui::Begin("Example Indicators");
    ImGui::Text("System Status: %s", io.WantCaptureMouse ? "Active" : "Inactive");
    ImGui::Text("Mouse Position: (%.1f, %.1f)", io.MousePos.x, io.MousePos.y);
    ImGui::Text("Delta Time: %.3f ms", io.DeltaTime * 1000.0f);
    ImGui::End();
}

// Utility function to get color for status string
ImVec4 StatusToColor(Status givenstatus)
{
    if (givenstatus == Success)
        return ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
    else if (givenstatus == Danger)
        return ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red
    else if (givenstatus == Standby)
        return ImVec4(0.0f, 0.0f, 1.0f, 1.0f); // Blue
    else if (givenstatus == Warning)
        return ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
    return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);     // Gray for unknown
}

// Render a status light (filled circle) and label
void RenderStatusLight(Status givenstatus, const char *label, float radius)
{
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImVec4 color = StatusToColor(givenstatus);
    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(pos.x + radius, pos.y + radius), radius, ImGui::GetColorU32(color));
    ImGui::Dummy(ImVec2(radius * 2, radius * 2)); // Reserve space for the circle
    ImGui::SameLine();
    // ImGui::Text("%s", label);
}

// Master status with indicator light
void MasterStatusLight(Status master_status)
{
    RenderStatusLight(master_status, ("Master Status:" + StatusToString(master_status)).c_str(), 12.0f);
    ImGui::Text("Master Status: %s", StatusToString(master_status).c_str());
}

// System status with indicator light and message
void SystemStatusLight(const SystemStatus &system_status)
{
    RenderStatusLight(system_status.status, (system_status.name + ": " + StatusToString(system_status.status)).c_str(), 8.0f);
    ImGui::Text("Message: %s", system_status.message.c_str());
}

void RenderStatusIndicators(Status master_status, std::vector<SystemStatus> SystemStatusData)
{
    ImGui::Begin("System Status Indicators");

    MasterStatusLight(master_status);
    for (size_t i = 1; i < SystemStatusData.size(); ++i)
    {
        if (ImGui::CollapsingHeader(SystemStatusData[i].name.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
        {
            SystemStatusLight(SystemStatusData[i]);
        }
    }

    ImGui::End();
}
