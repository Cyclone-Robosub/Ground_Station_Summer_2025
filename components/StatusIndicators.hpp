#pragma once

#include <string>
#include <vector>

struct SystemStatus {
    std::string name;     // Name of the system (e.g., "Manipulator", "Vision")
    std::string status;   // "error", "warning", "success"
    std::string message;  // e.g., "system is connected"
};

// Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

void RenderStatusIndicators(const std::string& master_status, const std::vector<SystemStatus>& system_statuses);