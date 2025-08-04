#pragma once

#include <string>
#include <vector>

struct SystemStatus {
    std::string status;   // "error", "warning", "success"
    std::string message;  // e.g., "system is connected"
};

// Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

void MasterStatusLight(const std::string& master_status);
void SystemStatusLight(const SystemStatus& system_status);
void RenderStatusIndicators(const std::string& master_status, const std::vector<SystemStatus>& system_statuses);