#pragma once
#include "../StructComponents.hpp"
#include <string>
#include <vector>

// Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

void RenderStatusIndicators(Status master_status, const std::vector<SystemStatus> SystemStatusData);