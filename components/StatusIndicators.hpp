#pragma once

#include <string>
#include <vector>

// Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

void RenderStatusIndicators(const std::string& master_status, const std::vector<SystemStatus>& system_statuses);